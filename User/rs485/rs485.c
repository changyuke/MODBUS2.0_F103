#include "rs485.h"


static void NVIC_Config(void)
{
	NVIC_InitTypeDef  NVIC_InitStruct;

	//����USARTΪ�ж�Դ
	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
	//�������ȼ�
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	//�����ȼ�
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 1;
	//ʹ���ж�
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	//��ʼ������NVIC
	NVIC_Init(&NVIC_InitStruct);
}


void RS485_Config(uint32_t bound)
{
	GPIO_InitTypeDef   GPIO_InitStruct;
	USART_InitTypeDef  USART_InitStruct;	

	//�򿪴���ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	//�򿪴���GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);	
	
	//��USART TX��GPIO���ã������������
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	//��USART RX��GPIO���ã���������	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStruct);	

	//485�շ����ƹܽ�,���ã���ͨ�������
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	//���ô��ڹ�������
	//���ò�����
	USART_InitStruct.USART_BaudRate = bound;
	//����һ�������ֳ���8λ
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	//����ֹͣλ��1λ
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	//����У��λ:��
	USART_InitStruct.USART_Parity = USART_Parity_No;
	//����Ӳ�������ƣ���
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	//���ù���ģʽ���շ�һ��	
	USART_InitStruct.USART_Mode = USART_Mode_Rx|USART_Mode_Tx;
	//��ɴ��ڵĳ�ʼ������	
	USART_Init(USART1, &USART_InitStruct);

	//�����ж����ȼ�����	
	NVIC_Config();
	//ʹ�ܴ��ڽ����ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);		
  
 	//ʹ�ܴ���	
	USART_Cmd(USART1, ENABLE); 
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_2);
}

/*****************  ����ָ���������� **********************/
void Usart_Send_Array(uint8_t *array, uint8_t len)
{
	static uint8_t i;

	GPIO_SetBits(GPIOB,GPIO_Pin_2);

  for(i = 0; i < len; i ++)
  {
    while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���   
    USART_SendData(USART1, array[i]);//�򴮿�1��������
  }
  while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���

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

/*****************  �����ַ��� **********************/
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


