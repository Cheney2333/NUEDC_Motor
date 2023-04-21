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

//左轮对应引脚定义
#define LIN_Port GPIOB
#define LIN1 GPIO_PIN_0
#define LIN2 GPIO_PIN_1
//右轮对应引脚定义
#define RIN_Port GPIOA
#define RIN1 GPIO_PIN_6
#define RIN2 GPIO_PIN_7
//定时器和通道定义
#define motor_TIM &htim1
#define rightMotorChannel TIM_CHANNEL_1
#define leftMotorChannel TIM_CHANNEL_2

void LeftMotor_Go(void); 	//左电机正转 LIN1=1 LIN2=0 即PB0高电平PB1低电平
void LeftMotor_Back(void); //左电机反转 LIN1=0 LIN2=1 即PB0低电平 PB1高电平
void LeftMotor_Stop(void); //左电机停转 LIN1和LIN2电平相同

void RightMotor_Go(void); 	//右电机正转 RIN1=0 RIN2=1 即PA6低电平 PA7高电平
void RightMotor_Back(void); //右电机反转 RIN1=1 RIN2=0 即PA6高电平 PA7低电平
void RightMotor_Stop(void); //右电机停转 RIN1和RIN2电平相同

void MotorControl(char motorDirection, int leftMotorPWM, int rightMotorPWM);

float CalActualSpeed(int pulse);

#ifdef __cplusplus
}
#endif
#endif /*__GPIO_H__ */

