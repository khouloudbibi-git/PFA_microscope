/**
  ******************************************************************************
  * @file    thermal.h
  * @brief   Regulation thermique du verre ITO : 2 thermistances NTC
  *          TK95F103V (ADC1 IN4/IN5), PID, PWM TIM4_CH4, securites.
  ******************************************************************************
  */
#ifndef THERMAL_H
#define THERMAL_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  THERM_OFF = 0,     /* chauffage desactive               */
  THERM_HEATING,     /* montee en temperature             */
  THERM_STABLE,      /* dans la bande +-0.1 degC          */
  THERM_FAULT        /* surchauffe / capteur defectueux   */
} therm_state_t;

void          Thermal_Init(void);
void          Thermal_Process(void);          /* boucle principale        */

void          Thermal_Enable(bool en);
bool          Thermal_IsEnabled(void);
bool          Thermal_SetSetpoint(float c);
float         Thermal_GetSetpoint(void);
void          Thermal_SetPid(float kp, float ki, float kd);
void          Thermal_GetPid(float *kp, float *ki, float *kd);

float         Thermal_GetTemp(void);          /* moyenne des 2 capteurs   */
float         Thermal_GetSensor(uint8_t idx); /* 0 = IN4, 1 = IN5         */
float         Thermal_GetPowerPct(void);
therm_state_t Thermal_GetState(void);
bool          Thermal_LowAlert(void);
uint8_t       Thermal_FaultCode(void);        /* 0 = OK                   */

/* Appeles depuis les callbacks HAL ADC */
void          Thermal_AdcConvCpltISR(void);
void          Thermal_AdcWatchdogISR(void);
void          Thermal_AdcErrorISR(void);

/* Evenement (implemente dans comm.c) */
void          Thermal_OnFault(uint8_t code);

#define THERM_FAULT_NONE       0U
#define THERM_FAULT_OVERTEMP   1U   /* > TEMP_CUTOFF_C (logiciel)    */
#define THERM_FAULT_AWD        2U   /* watchdog analogique ADC1 CH4  */
#define THERM_FAULT_SENSOR     3U   /* thermistances hors plage      */

#endif /* THERMAL_H */
