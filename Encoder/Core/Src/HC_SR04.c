#include "HC_SR04.h"

/*******************
*  @brief  us����ʱ ����΢���ʱ
*  @param  usdelay:Ҫ��ʱ��usʱ��
*  @return  
*
*******************/
void Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock /8U/1000U/1000);//SystemCoreClock:ϵͳƵ��
  do
  {
    __NOP();//ʹ�ÿ�ָ����ʱ����ֲ��ͬ��Ƭ��ע��__NOP(); ִ��ʱ��
  }
  while (Delay --);
}
/*******************
*  @brief  HC_SR04��ȡ����������
*  @param  ��
*  @return �ϰ�����뵥λ:cm (��ֹ����ƽ�����ȸ���) 
*
*******************/
float HC_SR04_Read(void)
{
	uint32_t i = 0;
	float Distance;
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_SET);//���15us�ߵ�ƽ
	Delayus(15);
	HAL_GPIO_WritePin(Trig_GPIO_Port,Trig_Pin,GPIO_PIN_RESET);//�ߵ�ƽ�������������Ϊ�͵�ƽ
	
	while(HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin) == GPIO_PIN_RESET)//�ȴ�����ߵ�ƽ
	{
		i++;
		Delayus(1);
		if(i>100000) return -1;//��ʱ�˳�ѭ������ֹ����������
	}
	i = 0;
	while(HAL_GPIO_ReadPin(Echo_GPIO_Port,Echo_Pin) == GPIO_PIN_SET)//�����ѭ����2us
	{
		i = i+1;
		Delayus(1);//1us ��ʱ����������ѭ�����2us����
		if(i >100000) return -2;//��ʱ�˳�ѭ��
	}
	Distance = i*2*0.033/2;//�����2��ԭ����������2΢��
	return Distance	;
}

