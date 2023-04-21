#include "motor.h"
#include "tim.h"

void LeftMotor_Go() 	//左电机正转 LIN1=1 LIN2=0 即PB0高电平PB1低电平
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_RESET);
}
void LeftMotor_Back() //左电机反转 LIN1=0 LIN2=1 即PB0低电平 PB1高电平
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_SET);
}
void LeftMotor_Stop() //左电机停转 LIN1和LIN2电平相同
{
  HAL_GPIO_WritePin(LIN_Port, LIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LIN_Port, LIN2, GPIO_PIN_SET);
}
//-------------------------------------------------
void RightMotor_Go() 	//右电机正转 RIN1=0 RIN2=1 即PA6低电平 PA7高电平
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_SET);
}
void RightMotor_Back() //右电机反转 RIN1=1 RIN2=0 即PA6高电平 PA7低电平
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_RESET);
}
void RightMotor_Stop() //右电机停转 RIN1和RIN2电平相同
{
  HAL_GPIO_WritePin(RIN_Port, RIN1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RIN_Port, RIN2, GPIO_PIN_SET);
}

/**
*    @brief 控制电机进行速度控制
*    @param 运动方向，左右电机的PWM值
*    @retval None
*/
void MotorControl(char motorDirection, int leftMotorPWM, int rightMotorPWM)
{
  switch (motorDirection)
  {
  case 0:   //前行
    LeftMotor_Go();
    RightMotor_Go();
    __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, rightMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, leftMotorPWM);
    break;
  case 1:   //后退
    LeftMotor_Back();
    RightMotor_Back();
    __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, rightMotorPWM);
    __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, leftMotorPWM);
    break;
  case 2:   //停车
    LeftMotor_Stop();
    RightMotor_Stop();
    __HAL_TIM_SET_COMPARE(motor_TIM, rightMotorChannel, 0);
    __HAL_TIM_SET_COMPARE(motor_TIM, leftMotorChannel, 0);
    break;
  default: break;
  }
}

/**
  * @brief  根据得到的编码器脉冲值计算速度 单位:m/s
  * @retval 速度值
  */
float CalActualSpeed(int pulse)
{
    return (float)(0.019625 * pulse);
}

