/**
  ******************************************************************************
  * @file    comm.c
  * @brief   Protocole texte entre le STM32 et le PC (pont Python <-> Firebase).
  *
  *  Trames ASCII terminees par '\n' (ou '\r'), insensibles a la casse.
  *
  *  Commandes PC -> STM32                     Reponse
  *  -------------------------------------------------------------------------
  *  PING                                      PONG
  *  MOVE <X|Y|Z> <um>     deplacement relatif OK MOVE / ERR ...
  *  GOTO <X|Y|Z> <um>     deplacement absolu  OK GOTO / ERR ...
  *  STOP [X|Y|Z]          arret (tous si omis)
  *  HOME [X|Y|Z|ALL]      prise d'origine (ALL si omis)
  *  ZERO <X|Y|Z>          position courante = 0
  *  SPEED <X|Y|Z> <um/s>  vitesse de l'axe
  *  MOTORS ON|OFF         activation des drivers TMC2209 (EN)
  *  CL ON|OFF             boucle fermee encodeur
  *  TSET <degC>           consigne de temperature (20..42)
  *  HEAT ON|OFF           chauffage ITO
  *  PID <kp> <ki> <kd>    gains du regulateur
  *  STREAM ON|OFF         envoi periodique du statut (200 ms)
  *  STATUS                envoi immediat du statut
  *
  *  Trame de statut STM32 -> PC :
  *  STA X=<um> Y=<um> Z=<um> EX=<um> EY=<um> EZ=<um> T=<degC> T1=<degC>
  *      T2=<degC> SP=<degC> PWR=<%> HEAT=<0|1> TST=<OFF|HEATING|STABLE|FAULT>
  *      MOV=<xyz> HOMED=<xyz> LIM=<x y1 y2> CL=<0|1> FLT=<code>
  *
  *  Evenements asynchrones : EVT DONE <axe> | EVT LIMIT <X|Y1|Y2> |
  *      EVT HOME <axe> OK|FAIL | EVT AXIS_FAULT <axe> ERR=<um> |
  *      EVT THERM_FAULT <code> | EVT TEMP_LOW | EVT TEMP_OK
  ******************************************************************************
  */
#include "comm.h"
#include "main.h"
#include "app_config.h"
#include "motion.h"
#include "thermal.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#define RX_RING_SIZE 256U

static volatile uint8_t  rx_ring[RX_RING_SIZE];
static volatile uint16_t rx_head = 0;
static uint16_t          rx_tail = 0;
static uint8_t           rx_byte;

static char     line[CMD_LINE_MAX];
static uint16_t line_len = 0;
static bool     stream_on = true;
static uint32_t last_status_ms = 0;
static bool     low_alert_prev = false;

static const char axis_name[AXIS_COUNT] = { 'X', 'Y', 'Z' };

/* ------------------------------------------------------------------------- */
void Comm_Printf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if (n > (int)sizeof(buf) - 1) n = sizeof(buf) - 1;
  HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 50);
}

/* Formatage d'un flottant sans printf("%f") (newlib-nano) */
static const char *fx(char *out, float v, int decimals)
{
  if (isnan(v)) { strcpy(out, "NaN"); return out; }
  int32_t scale = 1;
  for (int i = 0; i < decimals; i++) scale *= 10;
  int32_t iv = (int32_t)lroundf(v * (float)scale);
  const char *sign = (iv < 0) ? "-" : "";
  if (iv < 0) iv = -iv;
  if (decimals == 0) sprintf(out, "%s%ld", sign, (long)iv);
  else sprintf(out, "%s%ld.%0*ld", sign, (long)(iv / scale), decimals, (long)(iv % scale));
  return out;
}

/* Lecture d'un decimal "[-]123.456" sans scanf("%f") */
static bool parse_float(const char *s, float *out)
{
  if (s == NULL || *s == '\0') return false;
  float sign = 1.0f, val = 0.0f, frac = 0.1f;
  bool digits = false, dot = false;
  if (*s == '-') { sign = -1.0f; s++; } else if (*s == '+') { s++; }
  for (; *s; s++) {
    if (isdigit((unsigned char)*s)) {
      digits = true;
      if (!dot) val = val * 10.0f + (float)(*s - '0');
      else { val += frac * (float)(*s - '0'); frac *= 0.1f; }
    } else if ((*s == '.' || *s == ',') && !dot) {
      dot = true;
    } else {
      return false;
    }
  }
  if (!digits) return false;
  *out = sign * val;
  return true;
}

static bool parse_axis(const char *s, axis_id_t *ax)
{
  if (s == NULL || s[1] != '\0') return false;
  switch (s[0]) {
    case 'X': *ax = AXIS_X; return true;
    case 'Y': *ax = AXIS_Y; return true;
    case 'Z': *ax = AXIS_Z; return true;
    default:  return false;
  }
}

static bool parse_onoff(const char *s, bool *v)
{
  if (s == NULL) return false;
  if (!strcmp(s, "ON")  || !strcmp(s, "1")) { *v = true;  return true; }
  if (!strcmp(s, "OFF") || !strcmp(s, "0")) { *v = false; return true; }
  return false;
}

/* ------------------------------------------------------------------------- */
static void send_status(void)
{
  char b[12][16];
  static const char *tst[] = { "OFF", "HEATING", "STABLE", "FAULT" };
  uint8_t lim = Motion_LimitMask();

  Comm_Printf("STA X=%s Y=%s Z=%s EX=%s EY=%s EZ=%s T=%s T1=%s T2=%s SP=%s PWR=%s "
              "HEAT=%d TST=%s MOV=%d%d%d HOMED=%d%d%d LIM=%d%d%d CL=%d FLT=%u\r\n",
              fx(b[0], Motion_GetCmdPosUm(AXIS_X), 1),
              fx(b[1], Motion_GetCmdPosUm(AXIS_Y), 1),
              fx(b[2], Motion_GetCmdPosUm(AXIS_Z), 1),
              fx(b[3], Motion_GetEncPosUm(AXIS_X), 1),
              fx(b[4], Motion_GetEncPosUm(AXIS_Y), 1),
              fx(b[5], Motion_GetEncPosUm(AXIS_Z), 1),
              fx(b[6], Thermal_GetTemp(), 2),
              fx(b[7], Thermal_GetSensor(0), 2),
              fx(b[8], Thermal_GetSensor(1), 2),
              fx(b[9], Thermal_GetSetpoint(), 2),
              fx(b[10], Thermal_GetPowerPct(), 0),
              Thermal_IsEnabled() ? 1 : 0,
              tst[Thermal_GetState()],
              Motion_IsMoving(AXIS_X), Motion_IsMoving(AXIS_Y), Motion_IsMoving(AXIS_Z),
              Motion_IsHomed(AXIS_X), Motion_IsHomed(AXIS_Y), Motion_IsHomed(AXIS_Z),
              (lim & 1U) ? 1 : 0, (lim & 2U) ? 1 : 0, (lim & 4U) ? 1 : 0,
              Motion_ClosedLoop() ? 1 : 0,
              (unsigned)Thermal_FaultCode());
}

static void handle_line(char *s)
{
  for (char *p = s; *p; p++) *p = (char)toupper((unsigned char)*p);

  char *argv[5] = { 0 };
  int argc = 0;
  for (char *tok = strtok(s, " \t"); tok && argc < 5; tok = strtok(NULL, " \t")) argv[argc++] = tok;
  if (argc == 0) return;

  const char *cmd = argv[0];
  axis_id_t ax;
  float v;
  bool on;

  if (!strcmp(cmd, "PING")) {
    Comm_Printf("PONG\r\n");
  }
  else if (!strcmp(cmd, "MOVE") || !strcmp(cmd, "GOTO")) {
    if (argc < 3 || !parse_axis(argv[1], &ax) || !parse_float(argv[2], &v)) {
      Comm_Printf("ERR SYNTAX %s <X|Y|Z> <um>\r\n", cmd); return;
    }
    bool ok = (cmd[0] == 'M') ? Motion_MoveRel(ax, v) : Motion_MoveAbs(ax, v);
    if (ok) Comm_Printf("OK %s %c\r\n", cmd, axis_name[ax]);
    else    Comm_Printf("ERR %s %c REFUSED\r\n", cmd, axis_name[ax]);
  }
  else if (!strcmp(cmd, "STOP")) {
    if (argc >= 2 && parse_axis(argv[1], &ax)) Motion_Stop(ax);
    else Motion_StopAll();
    Comm_Printf("OK STOP\r\n");
  }
  else if (!strcmp(cmd, "HOME")) {
    if (argc >= 2 && strcmp(argv[1], "ALL") != 0) {
      if (!parse_axis(argv[1], &ax)) { Comm_Printf("ERR SYNTAX HOME [X|Y|Z|ALL]\r\n"); return; }
      Comm_Printf(Motion_StartHoming(ax) ? "OK HOME\r\n" : "ERR HOME REFUSED\r\n");
    } else {
      bool ok = Motion_StartHoming(AXIS_Z);
      ok &= Motion_StartHoming(AXIS_X);
      ok &= Motion_StartHoming(AXIS_Y);
      Comm_Printf(ok ? "OK HOME\r\n" : "ERR HOME REFUSED\r\n");
    }
  }
  else if (!strcmp(cmd, "ZERO")) {
    if (argc < 2 || !parse_axis(argv[1], &ax)) { Comm_Printf("ERR SYNTAX ZERO <X|Y|Z>\r\n"); return; }
    Motion_SetZero(ax);
    Comm_Printf("OK ZERO %c\r\n", axis_name[ax]);
  }
  else if (!strcmp(cmd, "SPEED")) {
    if (argc < 3 || !parse_axis(argv[1], &ax) || !parse_float(argv[2], &v) || !Motion_SetSpeed(ax, v)) {
      Comm_Printf("ERR SYNTAX SPEED <X|Y|Z> <um/s>\r\n"); return;
    }
    Comm_Printf("OK SPEED %c\r\n", axis_name[ax]);
  }
  else if (!strcmp(cmd, "MOTORS")) {
    if (argc < 2 || !parse_onoff(argv[1], &on)) { Comm_Printf("ERR SYNTAX MOTORS ON|OFF\r\n"); return; }
    Motion_EnableDrivers(on);
    Comm_Printf("OK MOTORS %s\r\n", on ? "ON" : "OFF");
  }
  else if (!strcmp(cmd, "CL")) {
    if (argc < 2 || !parse_onoff(argv[1], &on)) { Comm_Printf("ERR SYNTAX CL ON|OFF\r\n"); return; }
    Motion_SetClosedLoop(on);
    Comm_Printf("OK CL %s\r\n", on ? "ON" : "OFF");
  }
  else if (!strcmp(cmd, "TSET")) {
    if (argc < 2 || !parse_float(argv[1], &v) || !Thermal_SetSetpoint(v)) {
      Comm_Printf("ERR TSET (%d..%d degC)\r\n", (int)TEMP_SETPOINT_MIN_C, (int)TEMP_SETPOINT_MAX_C); return;
    }
    Comm_Printf("OK TSET\r\n");
  }
  else if (!strcmp(cmd, "HEAT")) {
    if (argc < 2 || !parse_onoff(argv[1], &on)) { Comm_Printf("ERR SYNTAX HEAT ON|OFF\r\n"); return; }
    Thermal_Enable(on);
    if (on && !Thermal_IsEnabled()) Comm_Printf("ERR HEAT FAULT=%u\r\n", (unsigned)Thermal_FaultCode());
    else Comm_Printf("OK HEAT %s\r\n", on ? "ON" : "OFF");
  }
  else if (!strcmp(cmd, "PID")) {
    float p, i, d;
    if (argc < 4 || !parse_float(argv[1], &p) || !parse_float(argv[2], &i) || !parse_float(argv[3], &d)) {
      Comm_Printf("ERR SYNTAX PID <kp> <ki> <kd>\r\n"); return;
    }
    Thermal_SetPid(p, i, d);
    Comm_Printf("OK PID\r\n");
  }
  else if (!strcmp(cmd, "STREAM")) {
    if (argc < 2 || !parse_onoff(argv[1], &on)) { Comm_Printf("ERR SYNTAX STREAM ON|OFF\r\n"); return; }
    stream_on = on;
    Comm_Printf("OK STREAM %s\r\n", on ? "ON" : "OFF");
  }
  else if (!strcmp(cmd, "STATUS") || !strcmp(cmd, "?")) {
    send_status();
  }
  else {
    Comm_Printf("ERR UNKNOWN %s\r\n", cmd);
  }
}

/* ------------------------------------------------------------------------- */
void Comm_Init(void)
{
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
  Comm_Printf("\r\nREADY PFA-MICROSCOPE STM32F411 FW 1.0\r\n");
  last_status_ms = HAL_GetTick();
}

void Comm_RxByteISR(void)
{
  uint16_t next = (uint16_t)((rx_head + 1U) % RX_RING_SIZE);
  if (next != rx_tail) {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  }
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void Comm_RxErrorISR(void)
{
  /* Erreur de trame / overrun : on relance la reception */
  HAL_UART_Receive_IT(&huart2, &rx_byte, 1);
}

void Comm_Process(void)
{
  while (rx_tail != rx_head) {
    char c = (char)rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % RX_RING_SIZE);

    if (c == '\n' || c == '\r') {
      if (line_len > 0U) {
        line[line_len] = '\0';
        handle_line(line);
        line_len = 0;
      }
    } else if (line_len < CMD_LINE_MAX - 1U) {
      line[line_len++] = c;
    } else {
      line_len = 0;   /* ligne trop longue : rejetee */
      Comm_Printf("ERR LINE TOO LONG\r\n");
    }
  }

  /* Alerte temperature insuffisante (sur transition) */
  bool low = Thermal_IsEnabled() && Thermal_LowAlert();
  if (low != low_alert_prev) {
    low_alert_prev = low;
    Comm_Printf(low ? "EVT TEMP_LOW\r\n" : "EVT TEMP_OK\r\n");
  }

  uint32_t now = HAL_GetTick();
  if (stream_on && (now - last_status_ms) >= STATUS_PERIOD_MS) {
    last_status_ms = now;
    send_status();
  }
}

/* ------------------------------------------------------------------------- */
/* Evenements des modules                                                    */
void Motion_OnLimitHit(axis_id_t ax, uint8_t bit)
{
  (void)ax;
  const char *n = (bit == 1U) ? "X" : (bit == 2U) ? "Y1" : "Y2";
  Comm_Printf("EVT LIMIT %s\r\n", n);
}

void Motion_OnMoveDone(axis_id_t ax)
{
  Comm_Printf("EVT DONE %c\r\n", axis_name[ax]);
}

void Motion_OnHomeDone(axis_id_t ax, bool ok)
{
  Comm_Printf("EVT HOME %c %s\r\n", axis_name[ax], ok ? "OK" : "FAIL");
}

void Motion_OnAxisFault(axis_id_t ax, float err_um)
{
  char b[16];
  Comm_Printf("EVT AXIS_FAULT %c ERR=%s\r\n", axis_name[ax], fx(b, err_um, 1));
}

void Thermal_OnFault(uint8_t code)
{
  Comm_Printf("EVT THERM_FAULT %u\r\n", (unsigned)code);
}
