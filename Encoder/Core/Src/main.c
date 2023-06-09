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
#include "HC_SR04.h"
#include "oled.h"
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
short encoderPulse[2] = {0};
float leftPWM, rightPWM;
float thisTrailStatus = 0;
float lastTrailStatus = 0;
float Trail_PID_PWM = 0;
int outRight = 0;
int outLeft = 0;
float distance = 100;
int count = 0;
int oledFlag = 0;
int sign = 0;
int signplus = 0;
int countplus = -50;
int direction;

int stage1 = 0;
int stage2 = 0;
int stage3 = 0;
int stage4 = 0;
int stage5 = 0;

int flag = 0;

char distanceStr[20] = {0};
char string1[11] = "distance: ";

PID_InitDefStruct leftMotor_PID;
PID_InitDefStruct rightMotor_PID;
PID_Trail trail;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void trailModule(void);
void recoverSpeed(void);
void beepOn(void);
void beepOff(void);
void stage1and2and3(void);
void stageFour(void);
void stageFive(void);
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
  PID_Trail_Init(&trail);

  // rightMotor_PID.Kp = 25;
  // rightMotor_PID.Ki = 50;
  // rightMotor_PID.Kd = 20;

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  OLED_Init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);            // set 20ms interrupt
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // set TIM1_CH1 PWM -- right wheel
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // set TIM1_CH2 PWM -- left wheel
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_2); // start encoder timer

  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim4, TIM_IT_UPDATE); // start encoder timer to update interrupts and prevent overflow processing

  __HAL_TIM_SET_COUNTER(&htim3, 30000);
  __HAL_TIM_SET_COUNTER(&htim4, 30000); // initialize encoder timing and set it to 3000

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if (oledFlag == 0)
    {
      OLED_Init();
      HAL_Delay(20);
      oledFlag = 1;
    }

    sprintf(distanceStr, "%s%.2f", string1, distance);

    OLED_ShowString(0, 0, (uint8_t *)distanceStr, 16, 1);
    HAL_Delay(1);
    OLED_Refresh();

    stage1and2and3();
    stageFour();
		stageFive();
    // MotorControl(0,leftMotor_PID.PWM,rightMotor_PID.PWM);
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  __HAL_TIM_GET_COUNTER(&htim4) = 0; // reset pulse count value
}

// interrupt handler
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  float c_leftSpeed, c_rightSpeed;

  if ((htim->Instance == TIM2))
  {
    if ((stage1 == 1 && stage2 == 0 && stage3 == 0) || (stage1 == 0 && stage2 == 1 && stage3 == 0) || stage4 == 1 || stage5 == 1)
    {
      count++;
      GetEncoderPulse();
      c_leftSpeed = CalActualSpeed(encoderPulse[1]); // calculate current speed
      c_rightSpeed = CalActualSpeed(encoderPulse[0]);
      // printf("leftSpeed = %.2f m/s, rightSpeed = %.2f m/s, deltaSpeed = %.2f m/s\n\r", c_leftSpeed, c_rightSpeed, c_leftSpeed-c_rightSpeed);
      printf("%.2f,%.2f,%.3f,%.3f\n\r", c_leftSpeed, c_rightSpeed,(double)leftMotor_PID.PWM/1000,(double)rightMotor_PID.PWM/1000);

      Velocity_PID(leftMotor_PID.targetSpeed, c_leftSpeed, &leftMotor_PID); // calculate the PID parameters for the left motor
      Velocity_PID(rightMotor_PID.targetSpeed, c_rightSpeed, &rightMotor_PID);
		
			
      // trailModule();
      // c_leftSpeed_afterPID = CalActualSpeed(encoderPulse[1]);
      // Velocity_PID(c_leftSpeed_afterPID,c_rightSpeed,&rightMotor_PID);  // calculate the PID of the right motor based on the speed of the left motor

      // printf("LeftMotor_PID.pwm_add = %.2f m/s, RightMotor_PID.pwm_add = %.2f m/s\n\r", LeftMotor_PID.pwm_add, RightMotor_PID.pwm_add);

      if (sign == 0 && stage4 == 0)
      {
        trailModule();
        if (distance < 31)
        {
          countplus = -50;
          sign = 1;
        }
      }
      if (sign == 1 && stage4 == 0)
      {
        countplus++;
        if (countplus < 0)
          direction = 2; // stop
        if (countplus < 50 && countplus >= -15)
        {
          direction = 0;
          outLeft = leftMotor_PID.PWM - 180;
          outRight = rightMotor_PID.PWM + 285;
        } // left
        if (countplus < 170 && countplus > 49)
        {
          outLeft = leftMotor_PID.PWM;
          outRight = rightMotor_PID.PWM + 5;
        } // straight
        if (countplus < 195 && countplus > 169)
          direction = 2; // stop

        if ((countplus < 300 && countplus > 194)||(countplus>549&&R1==0&&signplus<50))
        {
          direction = 0;
          outLeft = leftMotor_PID.PWM + 275;
          outRight = rightMotor_PID.PWM - 180;
					if(R2==1)signplus++;
        } // right
        if (countplus<550&&countplus > 299 && R2 == 0 && signplus == 0)
        {
          outLeft = leftMotor_PID.PWM;
          outRight = rightMotor_PID.PWM;
        }
        if ((countplus > 299 && R2 == 1) || signplus > 0)
        {
          signplus++;
          outLeft = leftMotor_PID.PWM - 120;
          outRight = rightMotor_PID.PWM + 185;
          if (signplus > 100||center==1)
          {
            sign = 0;
          }
        }
        MotorControl(direction, outLeft, outRight);
      }

      if (count > 4)
      { // 每100ms计算一次
        count = 0;
        distance = HC_SR04_Read();
        // printf("distance: %.2f cm\r\n", distance);
      }
      if (stage4 == 1 || stage5 == 1)
      {
        MotorControl(0, leftMotor_PID.PWM, rightMotor_PID.PWM);
      }
    }
  }
}

void trailModule()
{
  thisTrailStatus = getTrailStatus();
  Trail_PID(0, thisTrailStatus, &trail);
  Trail_PID_PWM = trail.PWM;
  outRight = rightMotor_PID.PWM - Trail_PID_PWM;
  outLeft = leftMotor_PID.PWM + Trail_PID_PWM;
  if (thisTrailStatus != lastTrailStatus || ((lastTrailStatus == thisTrailStatus) && center == 0))
  {
    MotorControl(0, outLeft, outRight);
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  }
  else
    MotorControl(0, leftMotor_PID.PWM, rightMotor_PID.PWM);
  lastTrailStatus = thisTrailStatus;
}

void beepOn()
{
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
}
void beepOff()
{
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
}
void stage1and2and3(void)
{
  if (((L2 == 1 && L1 == 1 && center == 1) || (R2 == 1 && R1 == 1 && center == 1) || (L1 == 1 && center == 1 && R1 == 1)) && stage1 == 0 && stage2 == 0 && stage3 == 0)
  {
    MotorControl(2, 0, 0);		// 起始线到暂停点1
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(800);
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(700);
    beepOn();
    HAL_Delay(500);
    beepOff();
    MotorControl(0, 620, 595);
    HAL_Delay(700);
    stage1 = 1;
  }
  else if (((L2 == 1 && L1 == 1 && center == 1) || (R2 == 1 && R1 == 1 && center == 1)) && stage1 == 1 && stage2 == 0 && stage3 == 0 && stage4 == 0)
  {
    stage2 = 1;			// 暂停点1到避障前
    stage1 = 1;
    MotorControl(2, 0, 0);
    HAL_Delay(21000);		// 暂停点1等待30s
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(700);
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(700);
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(5000);
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(700);
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(700);
    beepOn();
    HAL_Delay(500);
    beepOff();
    MotorControl(0, 585, 595);
    HAL_Delay(1000);
    stage1 = 0;
  }
  else if (((L2 == 1 && L1 == 1 && center == 1) || (R2 == 1 && R1 == 1 && center == 1) || (L1 == 1 && center == 1 && R1 == 1)) && stage1 == 0 && stage2 == 1 && stage3 == 0 && stage4 == 0)
  {
    stage3 = 1;									// 避障后到暂停点2
    MotorControl(2, 0, 0);
    HAL_Delay(500);
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(700);
    beepOn();
    HAL_Delay(500);
    beepOff();
    HAL_Delay(700);
    beepOn();
    HAL_Delay(500);
    beepOff();
    stage3 = 2;
  }
}
void stageFour()				// 暂停点2到暂停点3
{
  if (stage1 == 0 && stage2 == 1 && stage3 == 2)
  {
    if (stage4 == 0)
    {
      HAL_Delay(5000);
			stage4 = 1;
      leftMotor_PID.targetSpeed = 0.14;
      rightMotor_PID.targetSpeed = 0.105;
      MotorControl(0, leftMotor_PID.PWM + 50, rightMotor_PID.PWM - 50);
			HAL_Delay(1000);
    }
    if ((R2 == 1 || R1 == 1 || center == 1 || L2 == 1 || L1 == 1) && stage4 == 1)
    {
			stage4 = 2;
      HAL_Delay(550);
      leftMotor_PID.targetSpeed = 0;
      rightMotor_PID.targetSpeed = 0;
      MotorControl(2, 0, 0);
      HAL_Delay(50);
      beepOn();
      HAL_Delay(200);
      beepOff();
      HAL_Delay(500);
      beepOn();
      HAL_Delay(200);
      beepOff();
      HAL_Delay(500);
      beepOn();
      HAL_Delay(200);
      beepOff();
			stage5 = 1;
    }
		else if (stage4 == 1)
		{
			trailModule();
		}
  }
}
void stageFive(void) 				// 暂停点3回到起始线
{
	if (stage1 == 0 && stage2 == 1 && stage3 == 2 && stage5 == 1 && flag == 0 && stage4 == 2)
	{
		leftMotor_PID.targetSpeed = 0;
    rightMotor_PID.targetSpeed = 0;
		HAL_Delay(5000);
		beepOn();
    HAL_Delay(200);
    beepOff();
    HAL_Delay(500);
    beepOn();
    HAL_Delay(200);
    beepOff();
    HAL_Delay(500);
    beepOn();
    HAL_Delay(200);
    beepOff();
		
		leftMotor_PID.Un = 580;
		rightMotor_PID.Un= 570;
		leftMotor_PID.targetSpeed = 0.14;
		rightMotor_PID.targetSpeed = 0.12;
    // MotorControl(0, leftMotor_PID.PWM + 50, rightMotor_PID.PWM - 50);
		HAL_Delay(500);	// right
		
		HAL_Delay(6000);	
		
		MotorControl(2,0,0);
		HAL_Delay(200);
		leftMotor_PID.targetSpeed = 0;
		rightMotor_PID.targetSpeed = 0;
		HAL_Delay(500);		// stop
		
		leftMotor_PID.Un = 600;
		rightMotor_PID.Un= 600;
		leftMotor_PID.targetSpeed = 0.14;
		rightMotor_PID.targetSpeed = 0.145;
		HAL_Delay(8800);	// forward
		
		leftMotor_PID.targetSpeed = 0;
		rightMotor_PID.targetSpeed = 0;
		MotorControl(2,0,0);
		HAL_Delay(200);		// stop
		
		leftMotor_PID.targetSpeed = 0.14;
		rightMotor_PID.targetSpeed = 0.0;
		HAL_Delay(1500);	// right
		
		while (L2 == 0 && L1 == 0 && center == 0 && R1 == 0 && R2 == 0)
		{
			leftMotor_PID.targetSpeed = 0.129;
			rightMotor_PID.targetSpeed = 0.14;
		}
			
		leftMotor_PID.targetSpeed = 0;
		rightMotor_PID.targetSpeed = 0;
		MotorControl(2,0,0);
		HAL_Delay(500);	//stop
			
		beepOn();
		HAL_Delay(200);
		beepOff();
		HAL_Delay(500);
		beepOn();
		HAL_Delay(200);
		beepOff();
		HAL_Delay(500);
		beepOn();
		HAL_Delay(200);
		beepOff();
		
		flag = 1;
	}
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

#ifdef USE_FULL_ASSERT
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
