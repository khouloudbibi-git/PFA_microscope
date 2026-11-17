/**
  ******************************************************************************
  * @file    motion.c
  * @brief   Pilotage des axes X / Y / Z du plateau de microscope.
  *
  *  - Generation des impulsions STEP par interruption TIM5 (40 kHz) avec un
  *    accumulateur de phase (DDA) et une rampe trapezoidale
  *    acceleration / deceleration.
  *  - Lecture des encodeurs (TIM2 = X, TIM1 = Y, TIM3 = Z) etendue a 32 bits.
  *  - Fins de course a effet Hall US5881 (PC0 = X, PC1 = Y cote 1,
  *    PC2 = Y cote 2), actives a l'etat bas : arret immediat de l'axe.
  *  - Homing (prise d'origine) sur les capteurs X et Y1.
  *  - Boucle fermee : a la fin de chaque deplacement, la position mesuree par
  *    l'encodeur est comparee a la consigne et une correction est appliquee.
  ******************************************************************************
  */
#include "motion.h"
#include "main.h"
#include "app_config.h"
#include <math.h>
#include <stdlib.h>

/* ------------------------------------------------------------------------- */
typedef struct {
  /* Materiel */
  GPIO_TypeDef      *step_port;  uint16_t step_pin;
  GPIO_TypeDef      *dir_port;   uint16_t dir_pin;
  GPIO_TypeDef      *en_port;    uint16_t en_pin;
  TIM_HandleTypeDef *enc_tim;
  int8_t             dir_sign;
  int8_t             enc_sign;
  float              steps_per_um;
  float              counts_per_um;

  /* Fins de course : bit du masque (0 = aucun) cote negatif / positif */
  uint8_t            lim_neg_bit;
  uint8_t            lim_pos_bit;

  /* Etat generateur de pas (partage avec l'ISR) */
  volatile int32_t   pos;          /* position commandee (pas)        */
  volatile int32_t   target;       /* cible (pas)                     */
  volatile int8_t    dir;          /* +1 / -1                         */
  volatile float     rate;         /* vitesse instantanee (pas/s)     */
  volatile float     phase;        /* accumulateur DDA                */
  volatile bool      step_high;
  volatile bool      moving;
  volatile bool      evt_done;     /* fin de mouvement a signaler     */
  volatile uint8_t   evt_limit;    /* fin de course declenchee        */

  float              max_rate;     /* pas/s                           */
  float              accel;        /* pas/s^2                         */
  float              dv_per_tick;  /* accel / STEP_TICK_HZ            */

  /* Encodeur etendu */
  volatile int32_t   enc_count;
  uint16_t           enc_last;

  /* Boucle fermee */
  int32_t            goal;         /* consigne finale demandee (pas)  */
  uint8_t            cl_retries;
  bool               cl_active;
  bool               fault;

  /* Homing */
  home_state_t       home;
  bool               homed;
} axis_t;

static axis_t axes[AXIS_COUNT];
static bool   drivers_enabled = false;
static bool   closed_loop     = (CLOSED_LOOP_DEFAULT != 0);
static uint32_t enc_div       = 0;

#define LIM_X   0x01U
#define LIM_Y1  0x02U
#define LIM_Y2  0x04U

/* ------------------------------------------------------------------------- */
static inline uint8_t read_limits(void)
{
  uint8_t m = 0;
  if (HAL_GPIO_ReadPin(LIMIT_X_GPIO_Port,  LIMIT_X_Pin)  == GPIO_PIN_RESET) m |= LIM_X;
  if (HAL_GPIO_ReadPin(LIMIT_Y1_GPIO_Port, LIMIT_Y1_Pin) == GPIO_PIN_RESET) m |= LIM_Y1;
  if (HAL_GPIO_ReadPin(LIMIT_Y2_GPIO_Port, LIMIT_Y2_Pin) == GPIO_PIN_RESET) m |= LIM_Y2;
  return m;
}

static void set_speed_um(axis_t *a, float um_s, float accel_um_s2)
{
  float r = um_s * a->steps_per_um;
  if (r > STEP_MAX_RATE) r = STEP_MAX_RATE;
  if (r < STEP_MIN_RATE) r = STEP_MIN_RATE;
  a->max_rate    = r;
  a->accel       = accel_um_s2 * a->steps_per_um;
  a->dv_per_tick = a->accel / (float)STEP_TICK_HZ;
}

/* Lance un mouvement vers 'target' (pas). Appele hors ISR. */
static bool start_move(axis_t *a, int32_t target)
{
  if (!drivers_enabled || a->fault) return false;

  int8_t dir = (target > a->pos) ? +1 : (target < a->pos ? -1 : 0);
  if (dir == 0) { a->evt_done = true; return true; }

  /* Refus si l'on part vers un capteur deja actif */
  uint8_t lim = read_limits();
  if ((dir < 0 && (lim & a->lim_neg_bit)) || (dir > 0 && (lim & a->lim_pos_bit))) {
    return false;
  }

  __disable_irq();
  /* Positionnement de DIR avant la premiere impulsion */
  GPIO_PinState lvl = ((dir * a->dir_sign) > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(a->dir_port, a->dir_pin, lvl);
  if (!a->moving || a->dir != dir) {
    a->rate  = STEP_MIN_RATE;
    a->phase = 0.0f;
  }
  a->dir      = dir;
  a->target   = target;
  a->moving   = true;
  a->evt_done = false;
  __enable_irq();
  return true;
}

static void abort_axis_isr(axis_t *a)
{
  a->target = a->pos;
  a->rate   = 0.0f;
  a->moving = false;
}

static void encoder_update(axis_t *a)
{
  uint16_t now  = (uint16_t)__HAL_TIM_GET_COUNTER(a->enc_tim);
  int16_t  diff = (int16_t)(now - a->enc_last);
  a->enc_last   = now;
  a->enc_count += (int32_t)diff * a->enc_sign;
}

static inline int32_t enc_to_steps(const axis_t *a)
{
  return (int32_t)lroundf((float)a->enc_count / a->counts_per_um * a->steps_per_um);
}

/* ------------------------------------------------------------------------- */
void Motion_Init(void)
{
  const float spu_x = MOTOR_FULL_STEPS * MICROSTEPS / LEAD_X_UM;
  const float spu_y = MOTOR_FULL_STEPS * MICROSTEPS / LEAD_Y_UM;
  const float spu_z = MOTOR_FULL_STEPS * MICROSTEPS / LEAD_Z_UM;

  axes[AXIS_X] = (axis_t){
    .step_port = X_STEP_GPIO_Port, .step_pin = X_STEP_Pin,
    .dir_port  = X_DIR_GPIO_Port,  .dir_pin  = X_DIR_Pin,
    .en_port   = X_EN_GPIO_Port,   .en_pin   = X_EN_Pin,
    .enc_tim   = &htim2, .dir_sign = X_DIR_SIGN, .enc_sign = X_ENC_SIGN,
    .steps_per_um = spu_x, .counts_per_um = ENCODER_CPR / LEAD_X_UM,
    .lim_neg_bit = LIM_X, .lim_pos_bit = 0,
  };
  axes[AXIS_Y] = (axis_t){
    .step_port = Y_STEP_GPIO_Port, .step_pin = Y_STEP_Pin,
    .dir_port  = Y_DIR_GPIO_Port,  .dir_pin  = Y_DIR_Pin,
    .en_port   = Y_EN_GPIO_Port,   .en_pin   = Y_EN_Pin,
    .enc_tim   = &htim1, .dir_sign = Y_DIR_SIGN, .enc_sign = Y_ENC_SIGN,
    .steps_per_um = spu_y, .counts_per_um = ENCODER_CPR / LEAD_Y_UM,
    .lim_neg_bit = LIM_Y1, .lim_pos_bit = LIM_Y2,
  };
  axes[AXIS_Z] = (axis_t){
    .step_port = Z_STEP_GPIO_Port, .step_pin = Z_STEP_Pin,
    .dir_port  = Z_DIR_GPIO_Port,  .dir_pin  = Z_DIR_Pin,
    .en_port   = Z_EN_GPIO_Port,   .en_pin   = Z_EN_Pin,
    .enc_tim   = &htim3, .dir_sign = Z_DIR_SIGN, .enc_sign = Z_ENC_SIGN,
    .steps_per_um = spu_z, .counts_per_um = ENCODER_CPR / LEAD_Z_UM,
    .lim_neg_bit = 0, .lim_pos_bit = 0,
  };

  set_speed_um(&axes[AXIS_X], DEFAULT_SPEED_XY_UM_S, DEFAULT_ACCEL_XY_UM_S2);
  set_speed_um(&axes[AXIS_Y], DEFAULT_SPEED_XY_UM_S, DEFAULT_ACCEL_XY_UM_S2);
  set_speed_um(&axes[AXIS_Z], DEFAULT_SPEED_Z_UM_S,  DEFAULT_ACCEL_Z_UM_S2);

  for (int i = 0; i < AXIS_COUNT; i++) {
    HAL_GPIO_WritePin(axes[i].step_port, axes[i].step_pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(axes[i].enc_tim, 0);
    axes[i].enc_last = 0;
  }

  /* Demarrage des encodeurs (TI1 + TI2, x4) */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

  Motion_EnableDrivers(true);

  /* Base de temps des pas */
  HAL_TIM_Base_Start_IT(&htim5);
}

void Motion_EnableDrivers(bool en)
{
  if (!en) Motion_StopAll();
  drivers_enabled = en;
  for (int i = 0; i < AXIS_COUNT; i++) {
    HAL_GPIO_WritePin(axes[i].en_port, axes[i].en_pin,
                      en ? DRIVER_ENABLE_LEVEL : DRIVER_DISABLE_LEVEL);
  }
}

bool Motion_DriversEnabled(void) { return drivers_enabled; }

/* ------------------------------------------------------------------------- */
/* ISR TIM5 : 40 kHz                                                         */
void Motion_TickISR(void)
{
  const uint8_t lim = read_limits();

  for (int i = 0; i < AXIS_COUNT; i++) {
    axis_t *a = &axes[i];

    /* Fin de l'impulsion STEP (largeur = 1 tick = 25 us) */
    if (a->step_high) {
      a->step_high = false;
      a->step_port->BSRR = (uint32_t)a->step_pin << 16U;
      continue;
    }
    if (!a->moving) continue;

    /* Protection fin de course (en complement de l'EXTI) */
    if ((a->dir < 0 && (lim & a->lim_neg_bit)) || (a->dir > 0 && (lim & a->lim_pos_bit))) {
      abort_axis_isr(a);
      a->evt_limit = (a->dir < 0) ? a->lim_neg_bit : a->lim_pos_bit;
      a->evt_done  = true;
      continue;
    }

    int32_t remaining = (a->target - a->pos) * a->dir;
    if (remaining <= 0) {
      a->moving   = false;
      a->rate     = 0.0f;
      a->evt_done = true;
      continue;
    }

    /* Rampe trapezoidale : distance de freinage = v^2 / (2a) */
    float v = a->rate;
    float brake = (v * v) / (2.0f * a->accel);
    if ((float)remaining <= brake) {
      v -= a->dv_per_tick;
      if (v < STEP_MIN_RATE) v = STEP_MIN_RATE;
    } else if (v < a->max_rate) {
      v += a->dv_per_tick;
      if (v > a->max_rate) v = a->max_rate;
    } else if (v > a->max_rate) {
      v -= a->dv_per_tick;
    }
    a->rate = v;

    /* DDA */
    a->phase += v;
    if (a->phase >= (float)STEP_TICK_HZ) {
      a->phase -= (float)STEP_TICK_HZ;
      a->step_port->BSRR = a->step_pin;
      a->step_high = true;
      a->pos += a->dir;
    }
  }

  if (++enc_div >= ENCODER_SAMPLE_DIV) {
    enc_div = 0;
    for (int i = 0; i < AXIS_COUNT; i++) encoder_update(&axes[i]);
  }
}

/* EXTI PC0 / PC1 / PC2 : arret immediat de l'axe concerne */
void Motion_LimitISR(uint16_t gpio_pin)
{
  axis_t *a = NULL;
  uint8_t bit = 0;
  GPIO_TypeDef *port = NULL;

  if (gpio_pin == LIMIT_X_Pin)       { a = &axes[AXIS_X]; bit = LIM_X;  port = LIMIT_X_GPIO_Port;  }
  else if (gpio_pin == LIMIT_Y1_Pin) { a = &axes[AXIS_Y]; bit = LIM_Y1; port = LIMIT_Y1_GPIO_Port; }
  else if (gpio_pin == LIMIT_Y2_Pin) { a = &axes[AXIS_Y]; bit = LIM_Y2; port = LIMIT_Y2_GPIO_Port; }
  else return;

  /* Front descendant = aimant detecte (B >= Bop) */
  if (HAL_GPIO_ReadPin(port, gpio_pin) != GPIO_PIN_RESET) return;

  bool toward = (a->dir < 0 && bit == a->lim_neg_bit) || (a->dir > 0 && bit == a->lim_pos_bit);
  if (a->moving && toward) {
    abort_axis_isr(a);
    a->evt_limit = bit;
    a->evt_done  = true;
  }
}

/* ------------------------------------------------------------------------- */
static void homing_step(axis_id_t id)
{
  axis_t *a = &axes[id];
  if (a->moving) return;

  float spu = a->steps_per_um;
  switch (a->home) {
    case HOME_SEEK:
      if (read_limits() & a->lim_neg_bit) {
        a->home = HOME_BACKOFF;
        start_move(a, a->pos + (int32_t)(HOMING_BACKOFF_UM * spu));
      } else {
        /* Le mouvement s'est arrete sans capteur : echec */
        a->home = HOME_FAILED;
        set_speed_um(a, (id == AXIS_Z) ? DEFAULT_SPEED_Z_UM_S : DEFAULT_SPEED_XY_UM_S,
                     (id == AXIS_Z) ? DEFAULT_ACCEL_Z_UM_S2 : DEFAULT_ACCEL_XY_UM_S2);
        Motion_OnHomeDone(id, false);
      }
      break;

    case HOME_BACKOFF:
      if (read_limits() & a->lim_neg_bit) {
        /* Toujours sur l'aimant : on recule encore */
        start_move(a, a->pos + (int32_t)(HOMING_BACKOFF_UM * spu));
        break;
      }
      Motion_SetZero(id);
      a->homed = true;
      a->home  = HOME_DONE;
      set_speed_um(a, DEFAULT_SPEED_XY_UM_S, DEFAULT_ACCEL_XY_UM_S2);
      Motion_OnHomeDone(id, true);
      break;

    default:
      break;
  }
}

static void closed_loop_check(axis_id_t id)
{
  axis_t *a = &axes[id];
  if (!a->cl_active || a->moving) return;

  int32_t meas = enc_to_steps(a);
  int32_t err  = a->goal - meas;
  float   err_um = (float)err / a->steps_per_um;

  if (fabsf(err_um) <= CL_TOLERANCE_UM) {
    a->cl_active = false;
    Motion_OnMoveDone(id);
    return;
  }
  if (fabsf(err_um) > CL_FAULT_ERROR_UM) {
    a->cl_active = false;
    a->fault     = true;
    Motion_OnAxisFault(id, err_um);
    return;
  }
  if (a->cl_retries >= CL_MAX_RETRIES) {
    a->cl_active = false;
    Motion_OnAxisFault(id, err_um);
    return;
  }
  a->cl_retries++;
  /* Resynchronisation de la position commandee sur la mesure, puis correction */
  __disable_irq();
  a->pos = meas;
  __enable_irq();
  if (!start_move(a, a->goal)) {
    a->cl_active = false;
    Motion_OnMoveDone(id);
  }
}

void Motion_Process(void)
{
  for (int i = 0; i < AXIS_COUNT; i++) {
    axis_t *a = &axes[i];

    if (a->evt_limit) {
      uint8_t bit;
      __disable_irq(); bit = a->evt_limit; a->evt_limit = 0; __enable_irq();
      if (a->home != HOME_SEEK) {
        a->cl_active = false;            /* pas de correction contre une butee */
        a->goal      = a->pos;
        Motion_OnLimitHit((axis_id_t)i, bit);
      }
    }

    if (a->evt_done) {
      a->evt_done = false;
      if (a->home == HOME_SEEK || a->home == HOME_BACKOFF) {
        homing_step((axis_id_t)i);
      } else if (a->cl_active) {
        /* laisse l'encodeur se stabiliser 1 ms avant la mesure */
        HAL_Delay(2);
        closed_loop_check((axis_id_t)i);
      } else {
        Motion_OnMoveDone((axis_id_t)i);
      }
    }
  }
}

/* ------------------------------------------------------------------------- */
bool Motion_MoveAbs(axis_id_t ax, float pos_um)
{
  if (ax >= AXIS_COUNT) return false;
  axis_t *a = &axes[ax];
  if (a->home == HOME_SEEK || a->home == HOME_BACKOFF) return false;
  int32_t tgt = (int32_t)lroundf(pos_um * a->steps_per_um);
  a->goal       = tgt;
  a->cl_retries = 0;
  a->cl_active  = closed_loop;
  /* En boucle fermee, on part de la position mesuree */
  if (closed_loop && !a->moving) {
    __disable_irq(); a->pos = enc_to_steps(a); __enable_irq();
  }
  return start_move(a, tgt);
}

bool Motion_MoveRel(axis_id_t ax, float delta_um)
{
  if (ax >= AXIS_COUNT) return false;
  axis_t *a = &axes[ax];
  /* 'goal' = derniere consigne valide (remise a jour apres STOP / butee) */
  float base_um = (float)a->goal / a->steps_per_um;
  return Motion_MoveAbs(ax, base_um + delta_um);
}

void Motion_Stop(axis_id_t ax)
{
  if (ax >= AXIS_COUNT) return;
  axis_t *a = &axes[ax];
  __disable_irq();
  abort_axis_isr(a);
  a->goal = a->pos;
  __enable_irq();
  a->cl_active = false;
  if (a->home == HOME_SEEK || a->home == HOME_BACKOFF) a->home = HOME_IDLE;
}

void Motion_StopAll(void)
{
  for (int i = 0; i < AXIS_COUNT; i++) Motion_Stop((axis_id_t)i);
}

bool Motion_SetSpeed(axis_id_t ax, float um_per_s)
{
  if (ax >= AXIS_COUNT || um_per_s <= 0.0f) return false;
  float acc = (ax == AXIS_Z) ? DEFAULT_ACCEL_Z_UM_S2 : DEFAULT_ACCEL_XY_UM_S2;
  set_speed_um(&axes[ax], um_per_s, acc);
  return true;
}

void Motion_SetZero(axis_id_t ax)
{
  if (ax >= AXIS_COUNT) return;
  axis_t *a = &axes[ax];
  __disable_irq();
  a->pos = 0; a->target = 0; a->goal = 0;
  a->enc_count = 0;
  a->enc_last  = (uint16_t)__HAL_TIM_GET_COUNTER(a->enc_tim);
  a->fault     = false;
  __enable_irq();
}

bool Motion_StartHoming(axis_id_t ax)
{
  if (ax >= AXIS_COUNT) return false;
  axis_t *a = &axes[ax];
  a->homed = false;
  a->cl_active = false;
  a->fault = false;

  if (a->lim_neg_bit == 0) {
    /* Axe Z : pas de capteur -> la position actuelle devient l'origine */
    Motion_SetZero(ax);
    a->homed = true;
    a->home  = HOME_DONE;
    Motion_OnHomeDone(ax, true);
    return true;
  }

  set_speed_um(a, HOMING_SPEED_UM_S, DEFAULT_ACCEL_XY_UM_S2);
  if (read_limits() & a->lim_neg_bit) {
    a->home = HOME_BACKOFF;
    return start_move(a, a->pos + (int32_t)(HOMING_BACKOFF_UM * a->steps_per_um));
  }
  a->home = HOME_SEEK;
  if (!start_move(a, a->pos - (int32_t)(HOMING_MAX_TRAVEL_UM * a->steps_per_um))) {
    a->home = HOME_FAILED;
    return false;
  }
  return true;
}

void Motion_SetClosedLoop(bool en) { closed_loop = en; }
bool Motion_ClosedLoop(void)       { return closed_loop; }

float Motion_GetCmdPosUm(axis_id_t ax) { return (float)axes[ax].pos / axes[ax].steps_per_um; }
float Motion_GetEncPosUm(axis_id_t ax) { return (float)axes[ax].enc_count / axes[ax].counts_per_um; }
bool  Motion_IsMoving(axis_id_t ax)    { return axes[ax].moving || axes[ax].cl_active
                                                || axes[ax].home == HOME_SEEK
                                                || axes[ax].home == HOME_BACKOFF; }
bool  Motion_AnyMoving(void)
{
  for (int i = 0; i < AXIS_COUNT; i++) if (Motion_IsMoving((axis_id_t)i)) return true;
  return false;
}
bool  Motion_IsHomed(axis_id_t ax)     { return axes[ax].homed; }
bool  Motion_IsFault(axis_id_t ax)     { return axes[ax].fault; }
home_state_t Motion_HomeState(axis_id_t ax) { return axes[ax].home; }
uint8_t Motion_LimitMask(void)         { return read_limits(); }
