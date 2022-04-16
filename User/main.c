#include "stm32f10x.h"
#include "led.h"
#include "BasicTime.h"
#include "rs485.h"
#include "modbus.h"
#include "systick.h"


static void Delay(uint32_t count)
{
	for(; count!=0; count--);
}


void SystemInitial(void)
{
	LED_GPIO_Config();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //嵌套向量中断控制组选择
	BASIC_TIME_Config(1000-1, 72-1);                //1ms进一次中断
	RS485_Config(115200);
	delay_init();

}

void PeripheralInitial(void) //外围初始化
{


  
}



int main(void)
{ 	
	SystemInitial(); 
	Delay(0xFFFF);	
	PeripheralInitial(); 

	while(1)
	{
		modbus_handle(); 	//modbus处理函数

	}  
}

