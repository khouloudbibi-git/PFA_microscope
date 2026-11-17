/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   Automatisation d'un plateau de microscope (Clever C320)
  *                   PFA ENICarthage 2025/2026 - Carte NUCLEO-F411RE
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdint.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Exported variables --------------------------------------------------------*/
extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim1;   /* Encodeur axe Y          */
extern TIM_HandleTypeDef  htim2;   /* Encodeur axe X          */
extern TIM_HandleTypeDef  htim3;   /* Encodeur axe Z          */
extern TIM_HandleTypeDef  htim4;   /* PWM chauffage ITO (CH4) */
extern TIM_HandleTypeDef  htim5;   /* Base de temps des pas   */
extern UART_HandleTypeDef huart2;  /* Liaison PC 115200 8N1   */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
/* --- Fins de course (US5881, actif bas, EXTI) --- */
#define LIMIT_X_Pin            GPIO_PIN_0
#define LIMIT_X_GPIO_Port      GPIOC
#define LIMIT_X_EXTI_IRQn      EXTI0_IRQn
#define LIMIT_Y1_Pin           GPIO_PIN_1
#define LIMIT_Y1_GPIO_Port     GPIOC
#define LIMIT_Y1_EXTI_IRQn     EXTI1_IRQn
#define LIMIT_Y2_Pin           GPIO_PIN_2
#define LIMIT_Y2_GPIO_Port     GPIOC
#define LIMIT_Y2_EXTI_IRQn     EXTI2_IRQn

/* --- Moteur M1 / axe X (TMC2209) --- */
#define X_STEP_Pin             GPIO_PIN_0
#define X_STEP_GPIO_Port       GPIOA
#define X_DIR_Pin              GPIO_PIN_1
#define X_DIR_GPIO_Port        GPIOA
#define X_EN_Pin               GPIO_PIN_0
#define X_EN_GPIO_Port         GPIOB

/* --- Moteur M2 / axe Y (TMC2209) --- */
#define Y_STEP_Pin             GPIO_PIN_6
#define Y_STEP_GPIO_Port       GPIOA
#define Y_DIR_Pin              GPIO_PIN_7
#define Y_DIR_GPIO_Port        GPIOA
#define Y_EN_Pin               GPIO_PIN_1
#define Y_EN_GPIO_Port         GPIOB

/* --- Moteur M3 / axe Z (TMC2209) --- */
#define Z_STEP_Pin             GPIO_PIN_6
#define Z_STEP_GPIO_Port       GPIOB
#define Z_DIR_Pin              GPIO_PIN_7
#define Z_DIR_GPIO_Port        GPIOB
#define Z_EN_Pin               GPIO_PIN_8
#define Z_EN_GPIO_Port         GPIOB

/* --- Thermistances NTC TK95F103V --- */
#define NTC1_Pin               GPIO_PIN_4   /* ADC1_IN4 */
#define NTC1_GPIO_Port         GPIOA
#define NTC2_Pin               GPIO_PIN_5   /* ADC1_IN5 */
#define NTC2_GPIO_Port         GPIOA

/* --- PWM verre ITO --- */
#define HEATER_PWM_Pin         GPIO_PIN_9   /* TIM4_CH4 */
#define HEATER_PWM_GPIO_Port   GPIOB

/* --- USART2 (ST-LINK VCP) --- */
#define USART_TX_Pin           GPIO_PIN_2
#define USART_TX_GPIO_Port     GPIOA
#define USART_RX_Pin           GPIO_PIN_3
#define USART_RX_GPIO_Port     GPIOA

/* USER CODE BEGIN Private defines */
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
