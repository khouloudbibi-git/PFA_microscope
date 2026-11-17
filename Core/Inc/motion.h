/**
  ******************************************************************************
  * @file    motion.h
  * @brief   Pilotage des 3 axes (X, Y, Z) : moteurs pas a pas TMC2209
  *          (STEP/DIR/EN), encodeurs en mode Timer, fins de course a effet
  *          Hall, homing et correction en boucle fermee.
  ******************************************************************************
  */
#ifndef MOTION_H
#define MOTION_H

#include <stdint.h>
#include <stdbool.h>

typedef enum { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2, AXIS_COUNT = 3 } axis_id_t;

typedef enum {
  HOME_IDLE = 0,
  HOME_SEEK,       /* recherche du capteur de fin de course  */
  HOME_BACKOFF,    /* degagement du capteur                  */
  HOME_DONE,
  HOME_FAILED
} home_state_t;

void     Motion_Init(void);
void     Motion_TickISR(void);            /* appele par TIM5 a STEP_TICK_HZ */
void     Motion_Process(void);            /* appele en boucle principale    */
void     Motion_LimitISR(uint16_t gpio_pin);

void     Motion_EnableDrivers(bool en);
bool     Motion_DriversEnabled(void);

bool     Motion_MoveRel(axis_id_t ax, float delta_um);
bool     Motion_MoveAbs(axis_id_t ax, float pos_um);
void     Motion_Stop(axis_id_t ax);
void     Motion_StopAll(void);
bool     Motion_SetSpeed(axis_id_t ax, float um_per_s);
void     Motion_SetZero(axis_id_t ax);
bool     Motion_StartHoming(axis_id_t ax);
void     Motion_SetClosedLoop(bool en);
bool     Motion_ClosedLoop(void);

float    Motion_GetCmdPosUm(axis_id_t ax);   /* position commandee (pas)   */
float    Motion_GetEncPosUm(axis_id_t ax);   /* position mesuree (encodeur)*/
bool     Motion_IsMoving(axis_id_t ax);
bool     Motion_AnyMoving(void);
bool     Motion_IsHomed(axis_id_t ax);
bool     Motion_IsFault(axis_id_t ax);
home_state_t Motion_HomeState(axis_id_t ax);
uint8_t  Motion_LimitMask(void);             /* bit0=X bit1=Y1 bit2=Y2     */

/* Callbacks evenements (implementes dans comm.c) */
void     Motion_OnLimitHit(axis_id_t ax, uint8_t limit_bit);
void     Motion_OnMoveDone(axis_id_t ax);
void     Motion_OnHomeDone(axis_id_t ax, bool ok);
void     Motion_OnAxisFault(axis_id_t ax, float err_um);

#endif /* MOTION_H */
