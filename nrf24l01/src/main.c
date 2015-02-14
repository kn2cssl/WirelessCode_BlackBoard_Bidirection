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
int tmprid,robotspeed;
int wireless_reset;

char Address[_Address_Width] = { 0x11, 0x22, 0x33, 0x44, 0x55};
char str[200];
uint8_t count;
int display_counter = 0;
int16_t M3RPM;
int test_motor;
int test_robot;
int time;

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
	/////////////////////////////////////////////////////////////spi setting
																			
	spi_xmega_set_baud_div(&NRF24L01_L_SPI,8000000UL,F_CPU);				
	spi_enable_master_mode(&NRF24L01_L_SPI);								
	spi_enable(&NRF24L01_L_SPI);											
																			
	spi_xmega_set_baud_div(&NRF24L01_R_SPI,8000000UL,F_CPU);				
	spi_enable_master_mode(&NRF24L01_R_SPI);								
	spi_enable(&NRF24L01_R_SPI);											
																			
	sei();																	
																			
	////////////////////////////////////////////////////////////////////////

	_delay_us(10);
	//power on reset delay needs 100ms
	_delay_ms(100);															
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
				count = sprintf(str,"%d,%d,%d,%d,%d\r",test_robot,
				((int)(Buf_Rx_R[test_robot][0]<<8) & 0xff00) | ((int)(Buf_Rx_R[test_robot][1]) & 0x0ff),
				((int)(Buf_Rx_R[test_robot][2]<<8) & 0xff00) | ((int)(Buf_Rx_R[test_robot][3]) & 0x0ff),
				((int)(Buf_Rx_R[test_robot][4]<<8) & 0xff00) | ((int)(Buf_Rx_R[test_robot][5]) & 0x0ff),
				((int)(Buf_Rx_R[test_robot][6]<<8) & 0xff00) | ((int)(Buf_Rx_R[test_robot][7]) & 0x0ff));
				for (uint8_t i=0;i<count;i++)
				usart_putchar(&USARTE0,str[i]);
		
	}
}


ISR(TCD0_OVF_vect)
{   wireless_reset++;
	time++;
	for (uint8_t i=0;i<Max_Robot;i++)
	{
		//if there is no order from pc for a robot
		//an order of brake and then free wheel will
		//be send to that robot
		pck_timeout[i]++;
		if (pck_timeout[i]>=300)
		{
			if(pck_timeout[i]<=600)
			
			{//brake order
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
			{//freewheel order
				Buf_Tx_R[i][0] = i;
				Buf_Tx_R[i][1] = 1;
				Buf_Tx_R[i][2] = 2;
				Buf_Tx_R[i][3] = 3;
				Buf_Tx_R[i][4] = 4;
				Buf_Tx_R[i][5] = 0;
				Buf_Tx_R[i][6] = time;
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
	LED_White_R_PORT.OUTTGL = LED_White_R_PIN_bm;
	if((status_R & _RX_DR) == _RX_DR)
	{
		LED_White_R_PORT.OUTTGL = LED_White_R_PIN_bm;
		wireless_reset=0;
		do
		{
			tmprid = ((status_R&0x0e)>>1);
			NRF24L01_R_Flush_TX();
			NRF24L01_R_WriteRegBuf(W_ACK_PAYLOAD + 0, Buf_Tx_R[0], _Buffer_Size);
			NRF24L01_R_WriteRegBuf(W_ACK_PAYLOAD + 1, Buf_Tx_R[1], _Buffer_Size);
			NRF24L01_R_WriteRegBuf(W_ACK_PAYLOAD + 2, Buf_Tx_R[2], _Buffer_Size);
			//NRF24L01_R_WriteRegBuf(W_ACK_PAYLOAD + tmprid, Buf_Tx_R[tmprid], _Buffer_Size);
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
			tmprid = ((status_L&0x0e)>>1);
						NRF24L01_L_Flush_TX();
						NRF24L01_L_WriteRegBuf(W_ACK_PAYLOAD + 2, Buf_Tx_R[5], _Buffer_Size);
						NRF24L01_L_WriteRegBuf(W_ACK_PAYLOAD + 0, Buf_Tx_R[3], _Buffer_Size);
						NRF24L01_L_WriteRegBuf(W_ACK_PAYLOAD + 1, Buf_Tx_R[4], _Buffer_Size);
						
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


	switch (data)// used character : {w,s,123456(for robot id),!@#$(for motors id)}
	{
		
		case 'w':
		M3RPM+=10;
		Buf_Tx_R[0][7] = (char)((M3RPM>>8) & 0x0ff) ;
		Buf_Tx_R[0][8] = (char)(M3RPM & 0x0ff) ;
		count = sprintf(str,"M3RPM: %d\r",M3RPM);
		for (uint8_t i=0;i<count;i++)
		usart_putchar(&USARTE0,str[i]);
		break;
		
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		test_robot = data - '0';
		break;
		
		case '!':
		Buf_Tx_R[test_robot][11] = 0;
		break;
		
		case '@':
		Buf_Tx_R[test_robot][11] = 1;
		break;
		
		case '#':
		Buf_Tx_R[test_robot][11] = 2;
		break;
		
		case '$':
		Buf_Tx_R[test_robot][11] = 3;
		break;	
		
	};

}
ISR(USART_L_DRE_vect) //Wireless_R_USART
{

}
