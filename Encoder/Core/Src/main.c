/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Car main control program
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "pid.h"
#include "stdio.h"
#include "trail.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
short encoderPulse[2]={0};
float targetVelocity = 0.3; // target speed
float leftPWM, rightPWM;

PID_InitDefStruct leftMotor_PID;  
PID_InitDefStruct rightMotor_PID;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void trailModule(void);
void recoverSpeed(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  PID_Init(&leftMotor_PID);
  PID_Init(&rightMotor_PID);
	rightMotor_PID.Kp = 600;
  rightMotor_PID.Ki = 150;
	rightMotor_PID.Kd = 20;
	rightMotor_PID.Un	= 670;
  
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);      // set 50ms interrupt
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);   // set TIM1_CH1 PWM -- right wheel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);   // set TIM1_CH2 PWM -- left wheel
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);   
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);   
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);   
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2);   // start encoder timer

	__HAL_TIM_ENABLE_IT(&htim3,TIM_IT_UPDATE);      
	__HAL_TIM_ENABLE_IT(&htim4,TIM_IT_UPDATE);  // start encoder timer to update interrupts and prevent overflow processing
  
  __HAL_TIM_SET_COUNTER(&htim3, 30000);
	__HAL_TIM_SET_COUNTER(&htim4, 30000);   // initialize encoder timing and set it to 3000

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //MotorControl(0,leftMotor_PID.PWM,rightMotor_PID.PWM);
    
    trailModule();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
// printf redirect
int fputc(int ch, FILE *f)
{
 uint8_t temp[1] = {ch};
 HAL_UART_Transmit(&huart2, temp, 1, 2);
 return ch;
}

// Encoder speed measurement -------------------------------------------------------------
/**
  * @brief  read the count value(encoder pulse value) of TIM2 and TIM3, TIM3 -- rightWheel, TIM4 -- leftWheel
  * @param  None
  * @retval None
  */
void GetEncoderPulse()
{    
  encoderPulse[0] = -((short)__HAL_TIM_GET_COUNTER(&htim3));
  encoderPulse[1] = -((short)__HAL_TIM_GET_COUNTER(&htim4));

  __HAL_TIM_GET_COUNTER(&htim3) = 0;   
  __HAL_TIM_GET_COUNTER(&htim4) = 0;  // reset pulse count value
}

// interrupt handler
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  float c_leftSpeed, c_rightSpeed, c_leftSpeed_afterPID;
  
  if(htim->Instance == TIM2)
  {
    GetEncoderPulse(); 
    c_leftSpeed = CalActualSpeed(encoderPulse[1]);   // calculate current speed
    c_rightSpeed = CalActualSpeed(encoderPulse[0]);
    //printf("leftSpeed = %.2f m/s, rightSpeed = %.2f m/s, deltaSpeed = %.2f m/s\n\r", c_leftSpeed, c_rightSpeed, c_leftSpeed-c_rightSpeed);
    printf("%.2f,%.2f\n\r", c_leftSpeed, c_rightSpeed);
		
    Velocity_PID(targetVelocity,c_leftSpeed,&leftMotor_PID); // calculate the PID parameters for the left motor
    // Velocity_PID(targetVelocity,c_rightSpeed,&rightMotor_PID);
    c_leftSpeed_afterPID = CalActualSpeed(encoderPulse[1]);
    Velocity_PID(c_leftSpeed_afterPID,c_rightSpeed,&rightMotor_PID);  // calculate the PID of the right motor based on the speed of the left motor 
    
    // printf("LeftMotor_PID.pwm_add = %.2f m/s, RightMotor_PID.pwm_add = %.2f m/s\n\r", LeftMotor_PID.pwm_add, RightMotor_PID.pwm_add);
    //trailModule();
  }
}
// trail module
void trailModule()
{
  if(L2==GPIO_PIN_SET && L1==GPIO_PIN_RESET && center==GPIO_PIN_RESET && R1==GPIO_PIN_RESET && R2==GPIO_PIN_RESET){   // L2
    //MotorControl(2,0,0);
    while(L1==GPIO_PIN_RESET) { MotorControl(0,0,690); }
    recoverSpeed();
  }
  else if(L2==GPIO_PIN_RESET && L1==GPIO_PIN_SET && center==GPIO_PIN_RESET && R1==GPIO_PIN_RESET && R2==GPIO_PIN_RESET){    // L1
    while(center==GPIO_PIN_RESET) { MotorControl(0,0,680); }
    // while(center==GPIO_PIN_RESET) { MotorControl(0,680,) }
    recoverSpeed();
  }
  else if(L2==GPIO_PIN_RESET && L1==GPIO_PIN_RESET && center==GPIO_PIN_SET && R1==GPIO_PIN_RESET && R2==GPIO_PIN_RESET){    // center
    MotorControl(0,leftMotor_PID.PWM,rightMotor_PID.PWM);
  }
  else if(L2==GPIO_PIN_RESET && L1==GPIO_PIN_RESET && center==GPIO_PIN_RESET && R1==GPIO_PIN_SET && R2==GPIO_PIN_RESET){    // R1
    while(center==GPIO_PIN_RESET) { MotorControl(0,680,0); }
    recoverSpeed();
  }
  else if(L2==GPIO_PIN_RESET && L1==GPIO_PIN_RESET && center==GPIO_PIN_RESET && R1==GPIO_PIN_RESET && R2==GPIO_PIN_SET){    // R2
		while(R1==GPIO_PIN_RESET) { MotorControl(0,690,640); }
    recoverSpeed();
  }
  else{
    MotorControl(0,leftMotor_PID.PWM,rightMotor_PID.PWM);
  }
}
void recoverSpeed() {
  leftMotor_PID.Un = 680;
  rightMotor_PID.Un = 680;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
