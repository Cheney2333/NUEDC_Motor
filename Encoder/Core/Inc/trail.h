/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    trail.h
  * @brief   This file contains all the function prototypes for
  *          the motor.c file
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TRAIL_H__
#define __TRAIL_H__

#include "main.h"
#include "motor.h"
#include "pid.h"


#ifdef __cplusplus
extern "C" {
#endif

#define L2_Pin GPIO_PIN_12
#define L2_Port GPIOB
#define L1_Pin GPIO_PIN_13
#define L1_Port GPIOB
#define center_Pin GPIO_PIN_14
#define center_Port GPIOB
#define R1_Pin GPIO_PIN_15
#define R1_Port GPIOB
#define R2_Pin GPIO_PIN_10
#define R2_Port GPIOA

#define L2 HAL_GPIO_ReadPin(L2_Port,L2_Pin)
#define L1 HAL_GPIO_ReadPin(L1_Port,L1_Pin)
#define center HAL_GPIO_ReadPin(center_Port,center_Pin)
#define R1 HAL_GPIO_ReadPin(R1_Port,R1_Pin)
#define R2 HAL_GPIO_ReadPin(R2_Port,R2_Pin)

void trailModule(void);
float getTrailStatus(void);

#ifdef __cplusplus
}
#endif
#endif /*__TRAIL_H__ */

