#include "motor.h"
#include "tim.h"

void LeftMotor_Go() 	// LeftPostive:LIN1=1,LIN2=0,PB0=1,PB1=0
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_RESET);
}
void LeftMotor_Back() // LeftNegative,LIN1=0,LIN2=1,PB0=1,PB1=1
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_SET);
}
void LeftMotor_Stop() // LeftStop,LIN1=LIN2
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_SET);
}
//-------------------------------------------------
void RightMotor_Go() 	// RightPositive,RIN1=0,RIN2=1,PA6=0,PA7=1
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_SET);
}
void RightMotor_Back() // RightNegative,RIN1=1,RIN2=0,PA6=1,PA7=0
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_RESET);
}
void RightMotor_Stop() // RightStop,RIN1=RIN2
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_SET);
}

/**
*    @brief Control car's movement status
*    @param motorDirection,LeftMortorPWM,rightMotorPWm
*    @retval None
*/
void MotorControl(char motorDirection, int leftMotorPWM, int rightMotorPWM)
{
  switch (motorDirection)
  {
    case 0:   // forward
      LeftMotor_Go();
      RightMotor_Go();
      __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, rightMotorPWM);
      __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, leftMotorPWM);
      break;
    case 1:   // backward
      LeftMotor_Back();
      RightMotor_Back();
      __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, rightMotorPWM);
      __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, leftMotorPWM);
      break;
    case 2:   // stop
      LeftMotor_Stop();
      RightMotor_Stop();
      __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, 0);
      __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, 0);
      break;
    case 3:   // left
      LeftMotor_Back();
      RightMotor_Go();
      __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, rightMotorPWM);
      __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, leftMotorPWM);
    case 4:   // right
      LeftMotor_Go();
      RightMotor_Back();
      __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, rightMotorPWM);
      __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, leftMotorPWM);
    default: break;
  }
}

/**
  * @brief  calculate speed based on the obtained encoder pulse value, unit m/s
  * @param pulse
  * @retval speed
  */
float CalActualSpeed(int pulse)
{
    return (float)(0.003925 * pulse);
}

