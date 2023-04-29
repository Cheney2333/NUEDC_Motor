#include "pid.h"

/**
*    @brief PID参数初始化
*    @param PID结构的指针
*    @retval None
*/
void PID_Init(PID_InitDefStruct* p) //PID值初始化
{
	p->Kp = 25;
	p->Ki = 50;
	p->Kd = 20;
	p->Ur = 700;
	p->EN = 1;
	p->Un = 600;
	p->En_1 = 0;
	p->En_2 = 0;
	p->PWM = 0;
	p->targetSpeed = 0.2;
	p->targetSpeed= 0.2;
}

/**
*    @brief 利用PID算法计算速度值
*    @param 目标速度，此刻速度，所要计算的PID结构的指针
*    @retval None
*/
void Velocity_PID(float targetVelocity, float currentVelocity, PID_InitDefStruct* p)
{
	if(p->EN == 1)
	{
		float En = targetVelocity - currentVelocity;//误差值                                                     
	
		p->Un += p->Kp*(En - p->En_1) + p->Ki*En + p->Kd*(En - 2*p->En_1 + p->En_2); //增量式PID
		
		if(p->Un > p->Ur) p->Un = p->Ur;
		if(p->Un < 0) p->Un = 600;
		
		p->En_2 = p->En_1;
		p->En_1 = En;
		
		p->PWM = (int)p->Un;    //将Un强制转化为整数，PWM只能为整数
		
		/* 输出限幅 */
		if(p->PWM > p->Ur) p->PWM = p->Ur;
		if(p->PWM < 0) p->PWM = 0;
	}
	else PID_Init(p);
}

