#include "modbus.h"
#include "rs485.h"
#include "stm32f10x_it.h" 
#include "led.h"

#define MODBUS_LENGTH 100 //Usart.h

unsigned short modbus_reg[MODBUS_LENGTH];
uint8_t   Gu8_RS485_Addr = 1;                     //�ӻ���ַ
uint8_t   Gu8_RS485_TX_buff[USART_BUFF_SIZE];  //��������
uint16_t  Gu16_modbus_outputIO[USART_BUFF_SIZE];   //modbus�Ĵ����͵�Ƭ���Ĵ�����ӳ���ϵ
                                                 //  ����������Ĵ���ָ�루������λ��������

//0x01,0x03,0x00,0x07,0x00,0x01    crc��� 0x35,0xcb
static uint16_t get_crc16(uint8_t *pbuff, uint16_t len)   //��֤��ȷ
{

	uint8_t i;
	uint16_t ret1 = 0;
	uint16_t crc = 0xFFFF;

	if (len == 0)
	{
		len = 1;
	}
	while (len--)
	{
		crc ^= *pbuff;
		for (i = 0; i<8; i++)
		{
			if (crc & 1)
			{
				crc >>= 1;
				crc ^= 0xA001;
			}
			else
			{
				crc >>= 1;
			}
		}
		pbuff++;
	}
	ret1 = crc >> 8;
	ret1 = ret1 | (crc << 8);
	return(ret1);
}


////Modbus������03�������///////////////////////////////////////////////////////////////////////////////////////����֤����OK
////�����ּĴ���
static void modbus_03_solve(void)
{
	static uint16_t reg_num = 0;
	static uint16_t reg_start_addr = 0;	
	static uint8_t  i;
	static uint16_t crc_temp = 0;
	
	reg_start_addr = ((uint16_t)Gu8_RS485_RX_buff[2]<<8)|Gu8_RS485_RX_buff[3];	
	reg_num = ((uint16_t)Gu8_RS485_RX_buff[4]<<8)|Gu8_RS485_RX_buff[5];	
	
	if((reg_start_addr + reg_num) > REG_LAST_ADDR)  //�Ĵ�����ַ+����������Χ
	{
		Gu8_RS485_TX_buff[0] = Gu8_RS485_RX_buff[0];      //ID
		Gu8_RS485_TX_buff[1] = Gu8_RS485_RX_buff[1];     //������	
		Gu8_RS485_TX_buff[2] = 0x02;                    //�쳣��	
    Usart_Send_Array(Gu8_RS485_TX_buff, 3);		
		return;
	}

	Gu8_RS485_TX_buff[0] = Gu8_RS485_RX_buff[0];      //ID
	Gu8_RS485_TX_buff[1] = Gu8_RS485_RX_buff[1];     //������
	Gu8_RS485_TX_buff[2] = reg_num*2;               //�ֽڳ��� = �Ĵ�������x2
	for(i = 0; i < (reg_num*2); i++)
	{
		Gu8_RS485_TX_buff[3 + i*2] = ( Gu16_modbus_outputIO[reg_start_addr + i] / 256 ); 
		Gu8_RS485_TX_buff[4 + i*2] = ( Gu16_modbus_outputIO[reg_start_addr + i] % 256 ); 
	}
	
	crc_temp = get_crc16( Gu8_RS485_TX_buff, reg_num*2+3 );
	
	Gu8_RS485_TX_buff[reg_num*2+3] = (crc_temp>>8) & 0xFF;
	Gu8_RS485_TX_buff[reg_num*2+4] =  crc_temp & 0xFF;
	Usart_Send_Array(Gu8_RS485_TX_buff, (uint8_t)(reg_num*2+5));
	
}

////Modbus������05�������///////////////////////////////////////////////////////////////////////////////////////δ��֤����OK
////�����ּĴ���
static void modbus_05_solve(void)
{
	static uint16_t reg_start_addr = 0;
	static uint16_t crc_temp = 0;
	
	reg_start_addr = ((uint16_t)Gu8_RS485_RX_buff[2]<<8)|Gu8_RS485_RX_buff[3];	
	Gu16_modbus_outputIO[reg_start_addr]=Gu8_RS485_RX_buff[4]<<8;//���ֽ���ǰ                    ////////�޸�Ϊ���ֽ���ǰ�����ֽ��ں�
	Gu16_modbus_outputIO[reg_start_addr]|=((uint16_t)Gu8_RS485_RX_buff[5]);//���ֽ��ں�
	
	Gu8_RS485_TX_buff[0]=Gu8_RS485_RX_buff[0];
	Gu8_RS485_TX_buff[1]=Gu8_RS485_RX_buff[1];
	Gu8_RS485_TX_buff[2]=Gu8_RS485_RX_buff[2];
	Gu8_RS485_TX_buff[3]=Gu8_RS485_RX_buff[3];
	Gu8_RS485_TX_buff[4]=Gu8_RS485_RX_buff[4];
	Gu8_RS485_TX_buff[5]=Gu8_RS485_RX_buff[5];

	crc_temp=get_crc16(Gu8_RS485_TX_buff,6);
	Gu8_RS485_TX_buff[6]=(crc_temp>>8)&0xFF;
	Gu8_RS485_TX_buff[7]=(crc_temp)&0xFF;	
	
	Usart_Send_Array(Gu8_RS485_TX_buff, 8);
}
////Modbus������06�������   //////////////////////////////////////////////////////////////////////////////////����֤����OK
////д�������ּĴ���
static void modbus_06_solve(void)
{
	static uint16_t reg_start_addr = 0;
	static uint16_t crc_temp = 0;
	
	reg_start_addr = ((uint16_t)Gu8_RS485_RX_buff[2]<<8)|Gu8_RS485_RX_buff[3];	
	Gu16_modbus_outputIO[reg_start_addr]=Gu8_RS485_RX_buff[4]<<8;//���ֽ���ǰ                    ////////�޸�Ϊ���ֽ���ǰ�����ֽ��ں�
	Gu16_modbus_outputIO[reg_start_addr]|=((uint16_t)Gu8_RS485_RX_buff[5]);//���ֽ��ں�
	
	Gu8_RS485_TX_buff[0]=Gu8_RS485_RX_buff[0];
	Gu8_RS485_TX_buff[1]=Gu8_RS485_RX_buff[1];
	Gu8_RS485_TX_buff[2]=Gu8_RS485_RX_buff[2];
	Gu8_RS485_TX_buff[3]=Gu8_RS485_RX_buff[3];
	Gu8_RS485_TX_buff[4]=Gu8_RS485_RX_buff[4];
	Gu8_RS485_TX_buff[5]=Gu8_RS485_RX_buff[5];

	crc_temp=get_crc16(Gu8_RS485_TX_buff,6);
	Gu8_RS485_TX_buff[6]=(crc_temp>>8)&0xFF;
	Gu8_RS485_TX_buff[7]=(crc_temp)&0xFF;	
	
	Usart_Send_Array(Gu8_RS485_TX_buff, 8);
}
////Modbus������16���ݴ������ /////////////////////////////////////////////////////////////////////////////////////////////////����֤����OK
////д������ּĴ���
static void modbus_16_solve(void)
{
	static uint16_t reg_start_addr = 0;	
	static uint16_t reg_num = 0;
	static uint16_t crc_temp = 0;	
	static uint8_t i;
	
	reg_start_addr = ((uint16_t)Gu8_RS485_RX_buff[2]<<8)|Gu8_RS485_RX_buff[3];	
	reg_num = ((uint16_t)Gu8_RS485_RX_buff[4]<<8)|Gu8_RS485_RX_buff[5];	

	if((reg_start_addr + reg_num) > REG_LAST_ADDR)  //�Ĵ�����ַ+����������Χ
	{
		Gu8_RS485_TX_buff[0] = Gu8_RS485_RX_buff[0];      //ID
		Gu8_RS485_TX_buff[1] = Gu8_RS485_RX_buff[1];     //������	
		Gu8_RS485_TX_buff[2] = 0x02;                    //�쳣��	
    Usart_Send_Array(Gu8_RS485_TX_buff, 3);		
		return;
	}	
	
	for(i=0;i<reg_num;i++)
	{
		Gu16_modbus_outputIO[reg_start_addr+i]=Gu8_RS485_RX_buff[7+i*2]<<8; //���ֽ���ǰ                 /////// ���ֽ���ǰ�����ֽ��ں�����
		Gu16_modbus_outputIO[reg_start_addr+i]|=((uint16_t)Gu8_RS485_RX_buff[8+i*2]); //���ֽ��ں�
	}

	Gu8_RS485_TX_buff[0]=Gu8_RS485_RX_buff[0];
	Gu8_RS485_TX_buff[1]=Gu8_RS485_RX_buff[1];
	Gu8_RS485_TX_buff[2]=Gu8_RS485_RX_buff[2];
	Gu8_RS485_TX_buff[3]=Gu8_RS485_RX_buff[3];
	Gu8_RS485_TX_buff[4]=Gu8_RS485_RX_buff[4];
	Gu8_RS485_TX_buff[5]=Gu8_RS485_RX_buff[5];

	crc_temp=get_crc16(Gu8_RS485_TX_buff,6);
	Gu8_RS485_TX_buff[6]=(crc_temp>>8)&0xFF;
	Gu8_RS485_TX_buff[7]=(crc_temp)&0xFF;
  Usart_Send_Array(Gu8_RS485_TX_buff, 8);	
			
}

////Modbus������06������� /////////////////////////////////////////////////////////////////////////////////////////////////δ��֤����OK
////
void modbus_06_handle(void)
{
	
}

////Modbus������05������� /////////////////////////////////////////////////////////////////////////////////////////////////δ��֤����OK
////
void modbus_05_handle(uint8_t *buf)
{
	static unsigned int startaddr,code;//number,
	startaddr = buf[2]*256 + buf[3];
	code = buf[4]*256 + buf[5];
//	if ((startaddr) > 100)		//MODBUS_LENGTH----100
//	{
//		return;
//	}
	if(((buf[2]<<8|buf[3])== 0x0001)&&((buf[4]<<8|buf[5]) == 0xFF00))//01 05 00 01 FF 00 DD FA
	{
		LED_ON;
	}
	if(((buf[2]<<8|buf[3])== 0x0001)&&((buf[4]<<8|buf[5]) == 0x0000))//01 05 00 01 00 00 9C 0A
	{
		LED_OFF;
	}
//	modbus_reg[startaddr] = code;
//	Usart_Send_Array(buf,8);
//	Write_EN = 1;
	return;	
}

void modbus_handle(void)
{
	static uint16_t crc_temp1 = 0;//temp��ʱ
	static uint16_t crc_temp2 = 0;	
	
	if(Gu8_RS485_FrameFlag == 0)
	{
		return;
	}	
	Gu8_RS485_FrameFlag = 0; 
 	if(Gu8_RS485_RX_buff[0] !=  Gu8_RS485_Addr) //������ַ
	{
		return;
	} 
	crc_temp1=get_crc16(Gu8_RS485_RX_buff,Gu8_RS485_RX_cnt-2);	
	crc_temp2 =	((uint16_t)Gu8_RS485_RX_buff[Gu8_RS485_RX_cnt-2]<<8) + Gu8_RS485_RX_buff[Gu8_RS485_RX_cnt-1];
	if(crc_temp1 != crc_temp2)
	{
		return;
	}

	switch(Gu8_RS485_RX_buff[1])   //���չ��������
	{
		case 0x01:
			modbus_01_solve();
			break;
		case 0x03:   //����Ҫ��ӻ�����һЩ���ݣ�������Ĵ���
			modbus_03_solve();
			break;
		case 0x05:
			modbus_05_solve();
			modbus_05_handle(Gu8_RS485_TX_buff);
			break;			
		case 0x06:
			modbus_06_solve();
//			modbus_06_handle();
			break;		
		case 0x16:
			modbus_16_solve();
			break;
		
		default :
			break;
	}
}


