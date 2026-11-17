/**
  ******************************************************************************
  * @file    comm.h
  * @brief   Protocole texte USART2 (115200 8N1) avec le pont Python / IHM.
  ******************************************************************************
  */
#ifndef COMM_H
#define COMM_H

#include <stdint.h>

void Comm_Init(void);
void Comm_Process(void);          /* traitement des commandes + statut periodique */
void Comm_RxByteISR(void);        /* appele par HAL_UART_RxCpltCallback          */
void Comm_RxErrorISR(void);
void Comm_Printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif /* COMM_H */
