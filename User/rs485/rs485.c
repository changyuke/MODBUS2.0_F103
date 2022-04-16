#include "rs485.h"


static void NVIC_Config(void)
{
	NVIC_InitTypeDef  NVIC_InitStruct;

	//配置USART为中断源
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	//抢断优先级
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	//子优先级
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	//使能中断
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	//初始化配置NVIC
	NVIC_Init(&NVIC_InitStruct);
}


void RS485_Config(uint32_t bound)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;	

	//打开串口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	//打开串口GPIO时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	
	
	//将USART TX的GPIO配置：复用推挽输出
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//将USART RX的GPIO配置：浮空输入	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	

	//485收发控制管脚,配置：普通推挽输出
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//配置串口工作参数
	//配置波特率
	USART_InitStruct.USART_BaudRate = bound;
	//配置一针数据字长：8位
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	//配置停止位：1位
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	//配置校验位:无
	USART_InitStruct.USART_Parity = USART_Parity_No;
	//配置硬件流控制：无
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//配置工作模式，收发一起	
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	//完成串口的初始化配置	
	USART_Init(USART1, &USART_InitStruct);

	//串口中断优先级配置	
	NVIC_Config();
	//使能串口接收中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);		
  
 	//使能串口	
	USART_Cmd(USART1, ENABLE); 
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_2);
}

/*****************  发送指定长度数组 **********************/
void Usart_Send_Array(uint8_t *array, uint8_t len)
{
	static uint8_t i;

	GPIO_SetBits(GPIOB,GPIO_Pin_2);

  for(i = 0; i < len; i ++)
  {
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束   
    USART_SendData(USART1, array[i]);//向串口1发送数据
  }
  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束

	GPIO_ResetBits(GPIOB,GPIO_Pin_2); 

}








/*
void Usart_Send_Array(USART_TypeDef* USARTx, uint8_t *array, uint8_t arraylen)
{
	uint8_t i = 0;
	
	RS485Delay(1000);	
	GPIO_SetBits(GPIOB,GPIO_Pin_2);   
	RS485Delay(1000);
	
	do
	{
		Usart_Send_OneByte(USARTx, array[i]);
		i++;		
	}while(i < arraylen);
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) != SET);
	
	RS485Delay(1000);	
	GPIO_ResetBits(GPIOB,GPIO_Pin_2);
	RS485Delay(1000);
}
*/

/*****************  发送字符串 **********************/
/*
void RS485_SendString(USART_TypeDef* USARTx, uint8_t *String)
{
	uint8_t i = 0;
	
	RS485Delay(1000);	
	GPIO_SetBits(GPIOB,GPIO_Pin_2);   
	RS485Delay(1000);
	
	do
	{
		Usart_Send_OneByte(USARTx, String[i]);	
		i++;
	}while(String[i] != '\0');
	
	while(USART_GetFlagStatus(USARTx, USART_FLAG_TC) != SET);
	
	RS485Delay(1000);	
	GPIO_ResetBits(GPIOB,GPIO_Pin_2);
	RS485Delay(1000);
}

*/


