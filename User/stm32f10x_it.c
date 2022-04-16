/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 
uint8_t   Gu8_RS485_RX_buff[USART_BUFF_SIZE];   //接收内容
uint8_t   Gu8_RS485_RX_cnt = 0;   
uint8_t   Gu8_RS485_FrameFlag = 0;
uint32_t  Gu32_sec = 0;
uint8_t   start_time_flag = 0;             //接收空闲时间启动标志
uint16_t  start_time_cnt = 0;           //包与包之间的时间间隔              
uint8_t   RS485_RX_cnt = 0;


void  TIM3_IRQHandler (void)  //1ms  这里一定是1ms   
{ 
	static uint32_t sec_cnt = 0;
	if ( TIM_GetITStatus( TIM3, TIM_IT_Update) != RESET ) 
	{		
		if(start_time_flag == 1)
		{
			start_time_cnt ++;  //累加定时时间
			if(start_time_cnt > 50)  //判断时间是否超过设定的最大接收空闲时间  30-50ms
			{
				start_time_cnt = 0;
				start_time_flag = 0;
				Gu8_RS485_FrameFlag = 1;  //这一包数据接收完成
				Gu8_RS485_RX_cnt = RS485_RX_cnt;
				RS485_RX_cnt = 0;
		//        LED = ~LED;
			}
		}
		
		
		sec_cnt ++;
		if(sec_cnt >= MILLISECOND)  //2s
		{
			sec_cnt = 0;
			Gu32_sec = MILLISECOND;
		}

		
		TIM_ClearITPendingBit(TIM3 , TIM_IT_Update); 
		
	}		 	
}
void USART1_IRQHandler(void)
{
	uint8_t res = 0;
  
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{      
    start_time_flag = 1;  //每接收一帧数据，打开软件定时，去计数			
		res = USART_ReceiveData(USART1);		   
		if((RS485_RX_cnt < USART_BUFF_SIZE)&&(Gu8_RS485_FrameFlag == 0))  //接收缓冲区，尚未用完
		{
			Gu8_RS485_RX_buff[RS485_RX_cnt] = res;
			RS485_RX_cnt++;		
		}
		else
		{
			RS485_RX_cnt = 0;
		}
		start_time_cnt = 0;  //只要接收到数据，就长不大。除非空闲时候
	}
	USART_ClearITPendingBit(USART1, USART_IT_RXNE);
}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
