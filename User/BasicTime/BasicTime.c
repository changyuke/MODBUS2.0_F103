#include "BasicTime.h"


static void BASIC_TIME_NVIC_Config(void)
{
	NVIC_InitTypeDef  NVIC_InitStruct;

	//配置USART为中断源
	NVIC_InitStruct.NVIC_IRQChannel = TIM3_IRQn;
	//抢断优先级
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
	//子优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
	//使能中断
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStruct);
}


//                                        1000-1              72-1
void BASIC_TIME_Config(uint16_t TimPeriod,uint16_t TimPrescaler)
{
  
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  //开启定时器时钟，即内部时钟CK_INT = 72M
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  
  //自动重装载值
  TIM_TimeBaseInitStruct.TIM_Period = TimPeriod;
  //时钟预分频
  TIM_TimeBaseInitStruct.TIM_Prescaler = TimPrescaler;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  //初始化定时器
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStruct);
  
  //开启计数器中断
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

  BASIC_TIME_NVIC_Config();  
  //使能计数器
  TIM_Cmd(TIM3, ENABLE);

}






