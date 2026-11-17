/**
  ******************************************************************************
  * @file    thermal.c
  * @brief   Regulation PID de la temperature du plateau (verre ITO) a 37 degC.
  *
  *  ADC1 : scan IN4 (PA4) + IN5 (PA5), conversion continue, interruption a
  *  chaque fin de conversion. Les echantillons sont accumules dans l'ISR et
  *  moyennes toutes les PID_PERIOD_MS dans la boucle principale.
  *  Le watchdog analogique (canal 4, seuil haut 3500) coupe le chauffage
  *  materiellement en cas de surchauffe.
  ******************************************************************************
  */
#include "thermal.h"
#include "main.h"
#include "app_config.h"
#include <math.h>

/* ---- Acquisition (partage ISR) ---- */
static volatile uint32_t acc[2];
static volatile uint32_t cnt[2];
static volatile uint8_t  rank_idx = 0;
static volatile bool     awd_tripped = false;

/* ---- Etat ---- */
static float    temp_c[2]   = { NAN, NAN };
static float    temp_avg    = NAN;
static float    setpoint    = TEMP_SETPOINT_DEFAULT_C;
static float    kp = PID_KP_DEFAULT, ki = PID_KI_DEFAULT, kd = PID_KD_DEFAULT;
static float    integ       = 0.0f;
static float    prev_meas   = NAN;
static float    power_pct   = 0.0f;
static bool     enabled     = false;
static uint8_t  fault       = THERM_FAULT_NONE;
static uint32_t last_ms     = 0;

/* ------------------------------------------------------------------------- */
static void heater_set(float pct)
{
  if (pct < 0.0f)   pct = 0.0f;
  if (pct > 100.0f) pct = 100.0f;
  power_pct = pct;
  uint32_t ccr = (uint32_t)(pct * (float)HEATER_PWM_PERIOD / 100.0f);
  if (ccr > HEATER_PWM_PERIOD - 1U && pct < 100.0f) ccr = HEATER_PWM_PERIOD - 1U;
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, ccr);
}

static float adc_to_celsius(float adc)
{
  if (adc < (float)ADC_OPEN_LOW || adc > (float)ADC_OPEN_HIGH) return NAN;
  float r_ntc;
#if NTC_NTC_ON_HIGH_SIDE
  /* V = Vcc * Rf / (Rntc + Rf)  ->  Rntc = Rf * (FS/adc - 1) */
  r_ntc = NTC_R_FIXED_OHM * (ADC_FULL_SCALE / adc - 1.0f);
#else
  /* V = Vcc * Rntc / (Rntc + Rf) ->  Rntc = Rf * adc / (FS - adc) */
  r_ntc = NTC_R_FIXED_OHM * adc / (ADC_FULL_SCALE - adc);
#endif
  if (r_ntc <= 0.0f) return NAN;
  /* Equation Beta : 1/T = 1/T25 + ln(R/R25)/B */
  float inv_t = (1.0f / 298.15f) + logf(r_ntc / NTC_R25_OHM) / NTC_BETA;
  return (1.0f / inv_t) - 273.15f;
}

static void set_fault(uint8_t code)
{
  heater_set(0.0f);
  integ = 0.0f;
  if (fault != code) {
    fault = code;
    Thermal_OnFault(code);
  }
}

/* ------------------------------------------------------------------------- */
void Thermal_Init(void)
{
  heater_set(0.0f);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  rank_idx = 0;
  HAL_ADC_Start_IT(&hadc1);
  last_ms = HAL_GetTick();
}

/* Une interruption par canal converti (EOC a chaque conversion, rang 1 puis 2) */
void Thermal_AdcConvCpltISR(void)
{
  uint32_t v = HAL_ADC_GetValue(&hadc1);
  uint8_t  i = rank_idx;
  acc[i] += v;
  cnt[i]++;
  rank_idx = i ^ 1U;
}

void Thermal_AdcWatchdogISR(void)
{
  /* Coupure immediate, puis on masque l'IT AWD (elle se redeclencherait en
     continu tant que le seuil est depasse). Reactivee par Thermal_Process. */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  __HAL_ADC_DISABLE_IT(&hadc1, ADC_IT_AWD);
  awd_tripped = true;
}

void Thermal_AdcErrorISR(void)
{
  /* Overrun : on resynchronise la sequence sur le rang 1 */
  rank_idx = 0;
  HAL_ADC_Stop_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc1);
}

/* ------------------------------------------------------------------------- */
void Thermal_Process(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - last_ms) < PID_PERIOD_MS) return;
  float dt = (float)(now - last_ms) / 1000.0f;
  last_ms = now;

  /* --- Moyennage des echantillons --- */
  uint32_t a[2], n[2];
  __disable_irq();
  for (int i = 0; i < 2; i++) { a[i] = acc[i]; n[i] = cnt[i]; acc[i] = 0; cnt[i] = 0; }
  __enable_irq();
  for (int i = 0; i < 2; i++) {
    temp_c[i] = (n[i] > 0U) ? adc_to_celsius((float)a[i] / (float)n[i]) : NAN;
  }

  if (!isnan(temp_c[0]) && !isnan(temp_c[1])) temp_avg = 0.5f * (temp_c[0] + temp_c[1]);
  else if (!isnan(temp_c[0]))                 temp_avg = temp_c[0];
  else if (!isnan(temp_c[1]))                 temp_avg = temp_c[1];
  else                                        temp_avg = NAN;

  /* --- Securites --- */
  if (awd_tripped) {
    awd_tripped = false;
    set_fault(THERM_FAULT_AWD);
  }
  if (isnan(temp_avg)) {
    if (enabled) set_fault(THERM_FAULT_SENSOR);
    else heater_set(0.0f);
    return;
  }
  if (temp_avg > TEMP_CUTOFF_C) {
    set_fault(THERM_FAULT_OVERTEMP);
  }
  if (fault != THERM_FAULT_NONE) {
    heater_set(0.0f);
    /* Rearmement du watchdog analogique une fois refroidi */
    if (temp_avg < TEMP_REARM_C) {
      __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD);
      __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD);
    }
    return;
  }

  if (!enabled) {
    heater_set(0.0f);
    integ = 0.0f;
    prev_meas = temp_avg;
    return;
  }

  /* --- PID (derivee sur la mesure, anti-windup par saturation) --- */
  float err  = setpoint - temp_avg;
  float deriv = isnan(prev_meas) ? 0.0f : -(temp_avg - prev_meas) / dt;
  prev_meas = temp_avg;

  float p = kp * err;
  float d = kd * deriv;
  float new_integ = integ + ki * err * dt;
  float out = p + new_integ + d;

  if (out > 100.0f)      { out = 100.0f; if (err < 0.0f) integ = new_integ; }
  else if (out < 0.0f)   { out = 0.0f;   if (err > 0.0f) integ = new_integ; }
  else                   { integ = new_integ; }
  if (integ > 100.0f) integ = 100.0f;
  if (integ < 0.0f)   integ = 0.0f;

  heater_set(out);
}

/* ------------------------------------------------------------------------- */
void Thermal_Enable(bool en)
{
  if (en && fault != THERM_FAULT_NONE) {
    /* Acquittement du defaut uniquement si la temperature est revenue */
    if (!isnan(temp_avg) && temp_avg < TEMP_REARM_C) {
      fault = THERM_FAULT_NONE;
      __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD);
      __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_AWD);
    } else {
      return;
    }
  }
  enabled = en;
  integ = 0.0f;
  prev_meas = NAN;
  if (!en) heater_set(0.0f);
}

bool  Thermal_IsEnabled(void) { return enabled; }

bool Thermal_SetSetpoint(float c)
{
  if (c < TEMP_SETPOINT_MIN_C || c > TEMP_SETPOINT_MAX_C) return false;
  setpoint = c;
  return true;
}

float Thermal_GetSetpoint(void) { return setpoint; }

void Thermal_SetPid(float p, float i, float d)
{
  kp = p; ki = i; kd = d;
  integ = 0.0f;
}

void Thermal_GetPid(float *p, float *i, float *d) { *p = kp; *i = ki; *d = kd; }

float Thermal_GetTemp(void)            { return temp_avg; }
float Thermal_GetSensor(uint8_t idx)   { return (idx < 2U) ? temp_c[idx] : NAN; }
float Thermal_GetPowerPct(void)        { return power_pct; }
uint8_t Thermal_FaultCode(void)        { return fault; }

therm_state_t Thermal_GetState(void)
{
  if (fault != THERM_FAULT_NONE) return THERM_FAULT;
  if (!enabled)                  return THERM_OFF;
  if (!isnan(temp_avg) && fabsf(temp_avg - setpoint) <= TEMP_STABLE_BAND_C) return THERM_STABLE;
  return THERM_HEATING;
}

bool Thermal_LowAlert(void)
{
  return !isnan(temp_avg) && temp_avg < TEMP_LOW_ALERT_C;
}
