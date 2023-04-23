#include "pid.h"

/**
*    @brief PID value initialization
*    @param PID
*    @retval None
*/
void PID_Init(PID_InitDefStruct* p)	// PID value initialization
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
*    @brief calculate speed value using PID algorithm
*    @param targetVelocity, currentVelocity, PID
*    @retval None
*/
void Velocity_PID(float targetVelocity, float currentVelocity, PID_InitDefStruct* p)
{
	if(p->EN == 1)
	{
		float En = targetVelocity - currentVelocity;	// error value                                                  
	
		p->Un += p->Kp*(En - p->En_1) + p->Ki*En + p->Kd*(En - 2*p->En_1 + p->En_2);	// incremental PID
		
		p->En_2 = p->En_1;
		p->En_1 = En;
		
		p->PWM = (int)p->Un;
		
		/* output limiting */
		if(p->PWM > p->Ur) p->PWM = p->Ur;
		if(p->PWM < -p->Ur) p->PWM = -p->Ur;
	}
	else PID_Init(p);
}

