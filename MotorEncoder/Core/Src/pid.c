#include "pid.h"

/**
*    @brief PID参数初始化
*    @param PID结构的指针
*    @retval None
*/
void PID_Init(PID_InitDefStruct* p) //PID值初始化
{
	p->Kp = 2.5;
	p->Ki = 1.0;
	p->Kd = 0;
	p->Ur = 100;
	p->EN = 1;
	p->Un = 70;
	p->En_1 = 0;
	p->En_2 = 0;
	p->PWM = 0;
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
		
		p->En_2 = p->En_1;
		p->En_1 = En;
		
		p->PWM = (int)p->Un;
		
		/* 输出限幅 */
		if(p->PWM > p->Ur) p->PWM = p->Ur;
		if(p->PWM < -p->Ur) p->PWM = -p->Ur;
	}
	else PID_Init(p);
}

