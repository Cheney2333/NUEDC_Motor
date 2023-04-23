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
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
typedef struct
{
	float Kp;
	float Ki;
	float Kd;
	float Ur;		  // limiting value

  int EN;       // PID enable
	float Un;	    // expected output value
	float En_1;		// last error value
	float En_2;		// last but one error value
	int PWM;		  // outpur PWM value
}PID_InitDefStruct;
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */
void PID_Init(PID_InitDefStruct* p);  // PID  initialization
void Velocity_PID(float targetVelocity,float currentVelocity,PID_InitDefStruct* p); // calculate PID value
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__PID_H__ */

