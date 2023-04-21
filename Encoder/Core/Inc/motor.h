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

#ifdef __cplusplus
extern "C" {
#endif

//左轮
#define LIN_Port GPIOB
#define LIN1 GPIO_PIN_0
#define LIN2 GPIO_PIN_1
//右轮
#define RIN_Port GPIOA
#define RIN1 GPIO_PIN_6
#define RIN2 GPIO_PIN_7
//定时器以及通道
#define motor_TIM &htim1
#define rightMotorChannel TIM_CHANNEL_1
#define leftMotorChannel TIM_CHANNEL_2

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */
