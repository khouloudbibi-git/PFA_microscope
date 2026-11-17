/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  TIM4->CCR4 = 0;   /* securite : chauffage coupe */
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
  HAL_IncTick();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/** EXTI line0 : fin de course X (PC0) */
void EXTI0_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT_X_Pin);
}

/** EXTI line1 : fin de course Y cote 1 (PC1) */
void EXTI1_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT_Y1_Pin);
}

/** EXTI line2 : fin de course Y cote 2 (PC2) */
void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(LIMIT_Y2_Pin);
}

/** ADC1 global interrupt : fin de conversion NTC + watchdog analogique */
void ADC_IRQHandler(void)
{
  HAL_ADC_IRQHandler(&hadc1);
}

/** TIM5 global interrupt : generation des pas (40 kHz) */
void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}

/** USART2 global interrupt : liaison PC */
void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}
