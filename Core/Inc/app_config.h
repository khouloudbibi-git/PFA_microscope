/**
  ******************************************************************************
  * @file    app_config.h
  * @brief   Parametres applicatifs du plateau de microscope automatise.
  *          !!! Toutes les valeurs marquees [A CALIBRER] dependent de la
  *          mecanique OPTOSCOPIA et doivent etre verifiees sur le prototype.
  ******************************************************************************
  */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* ===================== Base de temps des pas (TIM5) ===================== */
#define STEP_TICK_HZ             40000U   /* frequence ISR TIM5 (1 MHz / 25)   */
#define STEP_MAX_RATE            15000.0f /* pas/s max (impulsion = 1 tick)    */
#define STEP_MIN_RATE            100.0f   /* vitesse de demarrage (pas/s)      */
#define ENCODER_SAMPLE_DIV       40U      /* lecture encodeurs a 1 kHz         */

/* ===================== Mecanique / moteurs ===================== */
#define MOTOR_FULL_STEPS         200.0f   /* 1.8 deg/pas (NEMA 8 et NEMA 14)  */
/* TMC2209 en mode standalone : MS2 MS1 = 00:1/8  01:1/32  10:1/64  11:1/16 */
#define MICROSTEPS               16.0f

/* Avance par tour de la vis / du mecanisme, en micrometres  [A CALIBRER] */
#define LEAD_X_UM                1000.0f
#define LEAD_Y_UM                1000.0f
#define LEAD_Z_UM                500.0f

/* Encodeurs integres : 1000 lignes -> x4 en quadrature = 4000 points/tour */
#define ENCODER_LINES            1000.0f
#define ENCODER_CPR              (ENCODER_LINES * 4.0f)

/* Sens : mettre -1 si l'axe ou l'encodeur tourne a l'envers   [A CALIBRER] */
#define X_DIR_SIGN               (+1)
#define Y_DIR_SIGN               (+1)
#define Z_DIR_SIGN               (+1)
#define X_ENC_SIGN               (+1)
#define Y_ENC_SIGN               (+1)
#define Z_ENC_SIGN               (+1)

/* TMC2209 : broche EN active a l'etat bas */
#define DRIVER_ENABLE_LEVEL      GPIO_PIN_RESET
#define DRIVER_DISABLE_LEVEL     GPIO_PIN_SET

/* Vitesses / accelerations par defaut (um/s, um/s^2) */
#define DEFAULT_SPEED_XY_UM_S    2000.0f
#define DEFAULT_SPEED_Z_UM_S     500.0f
#define DEFAULT_ACCEL_XY_UM_S2   10000.0f
#define DEFAULT_ACCEL_Z_UM_S2    2000.0f
#define HOMING_SPEED_UM_S        1000.0f
#define HOMING_BACKOFF_UM        300.0f
#define HOMING_MAX_TRAVEL_UM     250000.0f /* > course de la platine 235 mm */

/* Boucle fermee (correction par encodeur apres chaque deplacement) */
#define CLOSED_LOOP_DEFAULT      1
#define CL_TOLERANCE_UM          1.0f     /* cahier des charges : +-1 um     */
#define CL_MAX_RETRIES           3
#define CL_FAULT_ERROR_UM        2000.0f  /* ecart aberrant -> defaut axe     */

/* ===================== Thermique ===================== */
/* Montage : 3V3 -- NTC -- [PA4/PA5] -- 10k -- GND  (tension croit avec T) */
#define NTC_R_FIXED_OHM          10000.0f
#define NTC_R25_OHM              10000.0f
#define NTC_BETA                 3977.0f  /* TK95F103V (courbe Amphenol)  [A VERIFIER] */
#define NTC_NTC_ON_HIGH_SIDE     1        /* 0 si la NTC est cote GND       */
#define ADC_FULL_SCALE           4095.0f
#define ADC_OPEN_LOW             30U      /* en dessous : capteur ouvert/CC */
#define ADC_OPEN_HIGH            4065U

#define TEMP_SETPOINT_DEFAULT_C  37.0f
#define TEMP_SETPOINT_MIN_C      20.0f
#define TEMP_SETPOINT_MAX_C      42.0f
#define TEMP_CUTOFF_C            45.0f    /* coupure securite (cahier des charges) */
#define TEMP_REARM_C             40.0f
#define TEMP_STABLE_BAND_C       0.1f     /* +-0.1 degC                     */
#define TEMP_LOW_ALERT_C         36.0f    /* alerte "temperature insuffisante" */

#define PID_PERIOD_MS            100U
#define PID_KP_DEFAULT           25.0f    /* %/degC                [A REGLER] */
#define PID_KI_DEFAULT           0.8f     /* %/(degC.s)            [A REGLER] */
#define PID_KD_DEFAULT           10.0f    /* %.s/degC              [A REGLER] */
#define HEATER_PWM_PERIOD        1000U    /* = ARR + 1 de TIM4 (1 kHz)       */

/* Seuils du watchdog analogique ADC1 (canal 4) */
#define AWD_HIGH_THRESHOLD       3500U
#define AWD_LOW_THRESHOLD        0U

/* ===================== Communication ===================== */
#define STATUS_PERIOD_MS         200U
#define CMD_LINE_MAX             96U

#endif /* APP_CONFIG_H */
