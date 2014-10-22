#include <asf.h>
#include <stdio.h>
#include <string.h>
#define F_CPU 32000000UL
#include <util/delay.h>

#include "initialize.h"
#include "nrf24l01_L.h"
#include "nrf24l01_R.h"
#include "Menu.h"
#include "lcd.h"
#include "transmitter.h"

char Buf_Tx_R[Max_Robot][_Buffer_Size] ;
char Buf_Rx_R[Max_Robot][_Buffer_Size];
//char Buf_Tx_Comp[122] = {0xa5,0x5a}; //2+10*12
int tmprid,robotspeed;
int wireless_reset;

char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};
char str[200];
uint8_t count;
uint8_t flg;
float P_temp,I_temp,D_temp,P,I,D,a=0,ki=0,kp=0.15,kd=0.05,M1,M1_temp;//ki=1.34,kp=1,kd=0.02,;
int16_t M3RPM;
int motor_show;

uint16_t pck_timeout[Max_Robot];

int main (void)
{
	En_RC32M();
	PORT_init();
	//LCDInit();
	TimerD0_init();
	PMIC_CTRL |=PMIC_LOLVLEN_bm|PMIC_MEDLVLEN_bm;

	wdt_enable();

	USART_R_init();
	USART_L_init();
	NRF24L01_L_CE_LOW;       //disable transceiver modes
	NRF24L01_R_CE_LOW;
	///////////////////////////////////////////////////////////////////////////////////////////spi se

	spi_xmega_set_baud_div(&NRF24L01_L_SPI,8000000UL,F_CPU);
	spi_enable_master_mode(&NRF24L01_L_SPI);
	spi_enable(&NRF24L01_L_SPI);

	spi_xmega_set_baud_div(&NRF24L01_R_SPI,8000000UL,F_CPU);
	spi_enable_master_mode(&NRF24L01_R_SPI);
	spi_enable(&NRF24L01_R_SPI);
	
	sei();
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	_delay_us(10);
	_delay_ms(100);      //power on reset delay needs 100ms
	NRF24L01_L_Clear_Interrupts();
	NRF24L01_R_Clear_Interrupts();
	NRF24L01_L_Flush_TX();
	NRF24L01_R_Flush_TX();
	NRF24L01_L_Flush_RX();
	NRF24L01_R_Flush_RX();

	NRF24L01_L_CE_LOW;
	NRF24L01_L_Init_milad(_RX_MODE, _CH_L, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_L_WriteReg(W_REGISTER | DYNPD,0x07);//0x07
	NRF24L01_L_WriteReg(W_REGISTER | FEATURE,0x06);//0x06
	NRF24L01_L_CE_HIGH;


	NRF24L01_R_CE_LOW;
	NRF24L01_R_Init_milad(_RX_MODE, _CH_R, _2Mbps, Address, _Address_Width, _Buffer_Size, RF_PWR_MAX);
	NRF24L01_R_WriteReg(W_REGISTER | DYNPD,0x07);
	NRF24L01_R_WriteReg(W_REGISTER | FEATURE,0x06);
	NRF24L01_R_CE_HIGH;

	_delay_us(130);

	for (uint8_t i=0;i<Max_Robot;i++)
	{
		Robot_D_tmp[i].RID=12;
	}
	
	
	while (1)
	{
		//LCDGotoXY(0,0);
		//LCDStringRam("salad");
		
		//usart_putchar(&Wireless_R_USART,START_BYTE0);
		//usart_putchar(&Wireless_R_USART,START_BYTE1);
		//for (uint8_t i=0;i<Max_Robot;i++)
		//for (uint8_t j = 0; j<Max_SendPacket_Lenght; j++)
		//usart_putchar(&Wireless_R_USART,Buf_Rx_R[i][j]);
		//for (uint8_t i=0;i<_Buffer_Size;i++)
		//usart_putchar(&USARTE0,Buf_Rx_R[0][i]);
		if (wireless_reset>=30)
		{
			while(1)
			{
				
			}
		}
		
		for(uint8_t i=0;i<12;i++)
		{
			Buf_Tx_R[i][11] = Menu_Num;
			Buf_Tx_R[i][12] = (int)(kp*100);
			Buf_Tx_R[i][13] = (int)(ki*100);
			Buf_Tx_R[i][14] = (int)(kd*100);
		}


		
		//_delay_ms(20);
		_delay_us(1);
	}
}


ISR(TCD0_OVF_vect)
{   wireless_reset++;
	for (uint8_t i=0;i<Max_Robot;i++)
	{
		pck_timeout[i]++;
		if (pck_timeout[i]>=300)
		{
			if(pck_timeout[i]<=600)
			
			{
				Buf_Tx_R[i][0] = i;
				Buf_Tx_R[i][1] = 0;
				Buf_Tx_R[i][2] = 1;
				Buf_Tx_R[i][3] = 0;
				Buf_Tx_R[i][4] = 1;
				Buf_Tx_R[i][5] = 0;
				Buf_Tx_R[i][6] = 1;
				Buf_Tx_R[i][7] = 0;
				Buf_Tx_R[i][8] = 1;
				Buf_Tx_R[i][9] = 0;
				Buf_Tx_R[i][10] = 0;
			}
			else
			{
				Buf_Tx_R[i][0] = i;
				Buf_Tx_R[i][1] = 1;
				Buf_Tx_R[i][2] = 2;
				Buf_Tx_R[i][3] = 3;
				Buf_Tx_R[i][4] = 4;
				Buf_Tx_R[i][5] = 0;
				Buf_Tx_R[i][6] = 0;
				Buf_Tx_R[i][7] = 0;
				Buf_Tx_R[i][8] = 0;
				Buf_Tx_R[i][9] = 0;
				Buf_Tx_R[i][10]= 0;
			}
		}
	}
	while(Menu_PORT.IN & Menu_Side_Switch_PIN_bm);
	wdt_reset();
}

ISR(PRX_R)
{		
	wdt_reset();
	uint8_t status_R = NRF24L01_R_ReadReg(STATUSe);

	if((status_R & _RX_DR) == _RX_DR)
	{
		LED_White_R_PORT.OUTTGL = LED_White_R_PIN_bm;
		wireless_reset=0;
		do
		{
			//wdt_reset();
			//strcat(Buf_Tx_R[0] ,"123456789abcdefghijklmnopqrstuvw");
			tmprid = ((status_R&0x0e)>>1);
			NRF24L01_R_WriteRegBuf(W_ACK_PAYLOAD + tmprid, Buf_Tx_R[tmprid], _Buffer_Size);
			//1) read payload through SPI,
			NRF24L01_R_Read_RX_Buf(Buf_Rx_R[tmprid], _Buffer_Size);
			//2) clear RX_DR IRQ,
			status_R=NRF24L01_R_WriteReg(W_REGISTER | STATUSe, _RX_DR );
			//3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
			//4) if there are more data in RX FIFO, repeat from step 1).
		} while((NRF24L01_R_ReadReg(FIFO_STATUS)&RX_EMPTY) == 0x00);
	}
	status_R = NRF24L01_R_WriteReg(W_REGISTER|STATUSe,_TX_DS|_MAX_RT);
	if((status_R&_TX_DS) == _TX_DS)
	{
		LED_Green_R_PORT.OUTTGL = LED_Green_R_PIN_bm;
		wireless_reset=0;
	}
	if ((status_R&_MAX_RT) == _MAX_RT)
	{
		//LED_Green_L_PORT.OUTTGL = LED_Green_L_PIN_bm;
		NRF24L01_R_Flush_TX();
	}
	

	if(Menu_PORT.IN & Menu_Side_Select_PIN_bm)
	{
		count = sprintf(str,"%d,%d\r",
		((Buf_Rx_R[0][1+(Menu_Num*2)]<<8)&0x0ff00)|(Buf_Rx_R[0][0+(Menu_Num*2)]&0x00ff),
		((Buf_Rx_R[0][9+(Menu_Num*2)]<<8)&0x0ff00)|(Buf_Rx_R[0][8+(Menu_Num*2)]&0x00ff));
		
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
	}
	else
	{
		//count = sprintf(str,"%d,%d,",((int)(Buf_Rx_R[0][motor_show*2-2]<<8) & 0xff00) | ((int)(Buf_Rx_R[0][motor_show*2-1]) & 0x0ff),((int)(Buf_Rx_R[0][9]<<8) & 0xff00) | ((int)(Buf_Rx_R[0][8]) & 0x0ff));
		
		count = sprintf(str,"%d,%d,",((int)(Buf_Rx_R[0][9]<<8) & 0xff00) | ((int)(Buf_Rx_R[0][8]) & 0x0ff),((int)(Buf_Rx_R[0][11]<<8) & 0xff00) | ((int)(Buf_Rx_R[0][10]) & 0x0ff)),((int)(Buf_Rx_R[0][13]<<8) & 0xff00) | ((int)(Buf_Rx_R[0][12]) & 0x0ff),((int)(Buf_Rx_R[0][15]<<8) & 0xff00) | ((int)(Buf_Rx_R[0][14]) & 0x0ff));
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		
		count = sprintf(str,"%d,%d,%d\r",(int)(kp*100),(int)(ki*100),(int)(kd*100));

		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
	}
	
	


}

ISR(PRX_L)
{
	wdt_reset();
	uint8_t status_L = NRF24L01_L_ReadReg(STATUSe);
	if((status_L & _RX_DR) == _RX_DR)
	{
		LED_White_L_PORT.OUTTGL = LED_White_L_PIN_bm;
		do
		{
			//wdt_reset();
			//Buf_Tx_R[0] = "123456789abcdefghijklmnopqrstuvw";
			tmprid = ((status_L&0x0e)>>1);
			NRF24L01_L_WriteRegBuf(W_ACK_PAYLOAD + tmprid,Buf_Tx_R[tmprid+3], _Buffer_Size);
			//1) read payload through SPI,
			NRF24L01_L_Read_RX_Buf(Buf_Rx_R[tmprid+3], _Buffer_Size);
			//2) clear RX_DR IRQ,
			robotspeed= Buf_Rx_R[1][9]| Buf_Rx_R[1][8]<<8 ;
			status_L=NRF24L01_L_WriteReg(W_REGISTER | STATUSe, _RX_DR );
			//3) read FIFO_STATUS to check if there are more payloads available in RX FIFO,
			//4) if there are more data in RX FIFO, repeat from step 1).
		} while((NRF24L01_L_ReadReg(FIFO_STATUS)&RX_EMPTY) == 0x00);
	}
	status_L = NRF24L01_L_WriteReg(W_REGISTER|STATUSe,_TX_DS|_MAX_RT);
	if((status_L&_TX_DS) == _TX_DS)
	{
		LED_Green_L_PORT.OUTTGL = LED_Green_L_PIN_bm;
	}
	if ((status_L&_MAX_RT) == _MAX_RT)
	{
		//LED_Green_R_PORT.OUTTGL = LED_Green_R_PIN_bm;
		NRF24L01_L_Flush_TX();
	}
	
	
	
	
	//if(Menu_PORT.IN & Menu_Side_Select_PIN_bm)
	//{
	//count = sprintf(str,"%d,%d\r",
	//((Buf_Rx_R[0][1+(Menu_Num*2)]<<8)&0x0ff00)|(Buf_Rx_R[0][0+(Menu_Num*2)]&0x00ff),
	//((Buf_Rx_R[0][11+(Menu_Num*2)]<<8)&0x0ff00)|(Buf_Rx_R[0][10+(Menu_Num*2)]&0x00ff));
	//
	//for (uint8_t i=0;i<count;i++)
	//usart_putchar(&USARTE0,str[i]);
	//}
	//else
	//{
	//count = sprintf(str,"%d,%d,",((int)(Buf_Rx_R[3][7]<<8) & 0xff00) | ((int)(Buf_Rx_R[3][6]) & 0x0ff),((int)(Buf_Rx_R[3][15]<<8) & 0xff00) | ((int)(Buf_Rx_R[3][14]) & 0x0ff));
	//
	//for (uint8_t i=0;i<count;i++)
	//usart_putchar(&USARTE0,str[i]);
	//
	//count = sprintf(str,"%d,%d,%d\r",(int)(kp*100),(int)(ki*100),(int)(kd*100));
	//
	//for (uint8_t i=0;i<count;i++)
	//usart_putchar(&USARTE0,str[i]);
	//}
}

ISR(USART_R_RXC_vect)
{
	GetNewData(USARTC0_DATA);
}

ISR(USART_R_DRE_vect) //Wireless_R_USART
{

}

ISR(USART_L_RXC_vect)
{
	char data;
	data=USARTE0_DATA;


	switch (data)
	{
		case 'p':
		kp=kp+0.01;
		count = sprintf(str,"kp: %d\r",(int)(kp*100));
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		case 'o':
		ki=ki+0.05;
		count = sprintf(str,"ki: %d\r",(int)(ki*100));
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		case 'i':
		kd=kd+0.01;
		count = sprintf(str,"kd: %d\r",(int)(kd*100));
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		case 'l':
		kp=kp-0.01;
		count = sprintf(str,"kp: %d\r",(int)(kp*100));
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		case 'k':
		ki=ki-0.05;
		count = sprintf(str,"ki: %d\r",(int)(ki*100));
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		case 'j':
		kd=kd-0.01;
		count = sprintf(str,"kd: %d\r",(int)(kd*100));
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		
		case 'w':
		M3RPM+=10;
		Buf_Tx_R[0][7] = (char)((M3RPM>>8) & 0x0ff) ;
		Buf_Tx_R[0][8] = (char)(M3RPM & 0x0ff) ;
		count = sprintf(str,"M3RPM: %d\r",M3RPM);
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		
		case 's':
		M3RPM-=10;
		Buf_Tx_R[0][7] = (M3RPM>>8) & 0x0ff ;
		Buf_Tx_R[0][8] = M3RPM & 0x0ff ;
		count = sprintf(str,"M3RPM: %d\r",M3RPM);
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		
		case '1':
		motor_show = 1 ;
		break;
		
		case '2':
		motor_show = 2 ;
		break;
		
		case '3':
		motor_show = 3 ;
		break;
		
		case '4':
		motor_show = 4 ;
		break;
	};

}
ISR(USART_L_DRE_vect) //Wireless_R_USART
{

}
