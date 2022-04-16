#include "BasicTime.h"


static void BASIC_TIME_NVIC_Config(void)
{
	NVIC_InitTypeDef  NVIC_InitStruct;

	//����USARTΪ�ж�Դ
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
	//�������ȼ�
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
	//�����ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
	//ʹ���ж�
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStruct);
}


//                                        1000-1              72-1
void BASIC_TIME_Config(uint16_t TimPeriod,uint16_t TimPrescaler)
{
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  //������ʱ��ʱ�ӣ����ڲ�ʱ��CK_INT = 72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  
  //�Զ���װ��ֵ
  TIM_TimeBaseInitStruct.TIM_Period = TimPeriod;
  //ʱ��Ԥ��Ƶ
  TIM_TimeBaseInitStruct.TIM_Prescaler = TimPrescaler;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  //��ʼ����ʱ��
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
  
  //�����������ж�
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  BASIC_TIME_NVIC_Config();  
  //ʹ�ܼ�����
  TIM_Cmd(TIM3, ENABLE);

}






