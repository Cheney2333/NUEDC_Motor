/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    motor.h
  * @brief   This file contains all the function prototypes for
  *          the pid.c file
  ******************************************************************************
  * @attention
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

/* USER CODE BEGIN Includes */
#include "main.h"
#include "pid.h"
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef struct
{
	//相关速度PID参数
	float Kp;
	float Ki;
	float Kd;
	float Ur;		//限幅值
  float targetSpeed;

  int EN;         //PID使能
	float Un;	    //期望输出值
	float En_1;		//上一次的误差值
	float En_2;		//上上次的误差值
	int PWM;		//输出PWM值
}PID_InitDefStruct;
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void PID_Init(PID_InitDefStruct* p); //PID值初始化
void Velocity_PID(float targetVelocity,float currentVelocity,PID_InitDefStruct* p); //计算PID速度
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__PID_H__ */

