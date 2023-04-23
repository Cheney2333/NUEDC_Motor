/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   This file contains all the function prototypes for
  *          the motor.c file
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// left wheel corresponding pin definition
#define LIN_Port GPIOB
#define LIN1 GPIO_PIN_0
#define LIN2 GPIO_PIN_1
// right wheel corresponding pin definition
#define RIN_Port GPIOA
#define RIN1 GPIO_PIN_6
#define RIN2 GPIO_PIN_7
// timer and channel definition
#define motor_TIM &htim1
#define rightMotorChannel TIM_CHANNEL_1
#define leftMotorChannel TIM_CHANNEL_2

void LeftMotor_Go(void);
void LeftMotor_Back(void);
void LeftMotor_Stop(void);

void RightMotor_Go(void);
void RightMotor_Back(void);
void RightMotor_Stop(void);

void MotorControl(char motorDirection, int leftMotorPWM, int rightMotorPWM);

float CalActualSpeed(int pulse);

#ifdef __cplusplus
}
#endif
#endif /*__GPIO_H__ */

