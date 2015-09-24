/*
 * transmitter.h
 *
 * Created: 02/27/2012 05:52:57 ب.ظ
 *  Author: Milad
 
 
 
 
 */ 

#ifndef TRANSMITTER_H_
#define TRANSMITTER_H_
#include <lcd.h>
#include <stdio.h>

#define Header_Lenght 4
#define Data_Lenght 11
#define Max_Robot 12
#define START_BYTE0 0xA5
#define START_BYTE1 0x5A
#define STOP_BYTE 0x80
#define R 0
#define L 1
uint8_t PCK_Num[2]={0,0};
extern char Buf_Tx[2][Max_Robot][_Buffer_Size];
extern uint16_t pck_timeout[2][Max_Robot];
struct PCK_Header
{
	uint8_t NOB;  //Counter for Packet Bytes
	uint8_t SIB;
	uint8_t CHK;
};

union Motor_Speed
{
	char Bytes[2];
	int16_t Speed;
};

struct Robot_Data
{
	uint8_t RID;
	union Motor_Speed M0;
	union Motor_Speed M1;
	union Motor_Speed M2;
	union Motor_Speed M3;
	uint8_t KCK;
	uint8_t CHP;
	uint8_t ASK;
	uint8_t P;
	uint8_t I;
	uint8_t D;
	
};

struct PCK_Send_Header
{
	uint8_t SIB;
	uint8_t CHK;
	uint8_t RID;	
};


struct PCK_Header PCK_H[2];
struct Robot_Data Robot_D[2][Max_Robot], Robot_D_tmp[2][Max_Robot];


// receiving data for 9 robots(SIB = 104 ),why? should be 12 robots !!
// running time : about 560 clk
// time for receiving all packet : 560 * 104 clk = 58240 clk = 1.82 ms
static void GetNewData(uint8_t data,int side)
{	
	if (PCK_Num[side]<Header_Lenght)
	{
		switch(PCK_Num[side])
		{	
		case 0:
			if (data == START_BYTE0) 
			{
				PCK_Num[side]++;// LED_White_R_PORT.OUTTGL = LED_White_R_PIN_bm;
			}
			break;
		case 1:
			if (data == START_BYTE1) 
				PCK_Num[side]++;
			else
			{
				PCK_Num[side] = 0;
			}			
			break;
		case 2:
			PCK_H[side].SIB = data;
			PCK_Num[side]++;
			break;
		case 3:
			PCK_H[side].CHK = data;
			PCK_Num[side]++;
			break;
		}
	}	
	else
	{
		if (PCK_Num[side] < PCK_H[side].SIB-1)
		{
			
			switch((PCK_Num[side]-Header_Lenght) % Data_Lenght) //0x07 is Size of Robot Data
			{
			case 0:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].RID=data;
				PCK_H[side].CHK -= data;
			break;
			case 1:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].M0.Bytes[1]=data;
				PCK_H[side].CHK -= data;
			break;
			case 2:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].M0.Bytes[0]=data;
				PCK_H[side].CHK -= data;
			break;
			case 3:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].M1.Bytes[1]=data;
				PCK_H[side].CHK -= data;
			break;
			case 4:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].M1.Bytes[0]=data;
				PCK_H[side].CHK -= data;
			break;
			case 5:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].M2.Bytes[1]=data;
				PCK_H[side].CHK -= data;
			break;
			case 6:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].M2.Bytes[0]=data;
				PCK_H[side].CHK -= data;
			break;
			case 7:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].M3.Bytes[1]=data;
				PCK_H[side].CHK -= data;
			break;
			case 8:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].M3.Bytes[0]=data;
				PCK_H[side].CHK -= data;
			break;
			case 9:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].KCK=data;
				PCK_H[side].CHK -= data;
			break;
			case 10:
				Robot_D_tmp[side][(PCK_Num[side]-Header_Lenght)/Data_Lenght].CHP=data;
				PCK_H[side].CHK -= data;
			break;		
			}
			PCK_Num[side]++;
		}		
		else
		{ 
			if (PCK_H[side].CHK == 0 && data == STOP_BYTE)
			{	
			
				for (uint8_t i=0;i<Max_Robot;i++)
				{
					
					
						if (Robot_D_tmp[side][i].RID!=12)
						{
							Robot_D[side][i] = Robot_D_tmp[side][i];
							Robot_D_tmp[side][i].RID=12;
							Robot_D_tmp[side][i].M0.Speed=0x7FFF;
							Robot_D_tmp[side][i].M1.Speed=0x7FFF;
							Robot_D_tmp[side][i].M2.Speed=0x7FFF;
							Robot_D_tmp[side][i].M3.Speed=0x7FFF;
							Robot_D_tmp[side][i].KCK=0;
							Robot_D_tmp[side][i].CHP=0;

							
							if (Robot_D[side][i].RID<Max_Robot)
							{
								Buf_Tx[side][Robot_D[side][i].RID][0] = Robot_D[side][i].RID;
								Buf_Tx[side][Robot_D[side][i].RID][1] = Robot_D[side][i].M0.Bytes[1];
								Buf_Tx[side][Robot_D[side][i].RID][2] = Robot_D[side][i].M0.Bytes[0];
								Buf_Tx[side][Robot_D[side][i].RID][3] = Robot_D[side][i].M1.Bytes[1];
								Buf_Tx[side][Robot_D[side][i].RID][4] = Robot_D[side][i].M1.Bytes[0];
								Buf_Tx[side][Robot_D[side][i].RID][5] = Robot_D[side][i].M2.Bytes[1];
								Buf_Tx[side][Robot_D[side][i].RID][6] = Robot_D[side][i].M2.Bytes[0];
								Buf_Tx[side][Robot_D[side][i].RID][7] = Robot_D[side][i].M3.Bytes[1];
								Buf_Tx[side][Robot_D[side][i].RID][8] = Robot_D[side][i].M3.Bytes[0];
								Buf_Tx[side][Robot_D[side][i].RID][9] = Robot_D[side][i].KCK;
								Buf_Tx[side][Robot_D[side][i].RID][10] = Robot_D[side][i].CHP;

								pck_timeout[side][Robot_D[side][i].RID]=0;
							}
						}
				}
			}
			PCK_Num[side] = 0;
		}					
	}
}
#endif /* TRANSMITTER_H_ */




////////
////////SOP	// start of packet (2 bytes)
////////SIB	// size in bytes
////////CHK	// ckeck sum
////////-------------------------------------------
////////RID	// robot id (1 bytes)
////////M1	// motor 1
////////M2	// motor 2
////////M3	// motor 3
////////M4	// motor 4
////////KCK	// kick options
////////CHP	// CHIP & SPIN   #SCCCCCCC
////////-------------------------------------------
////////EOP	// end of packet
////////
////////
////////
////////SOP	// start of packet (2 bytes)
////////CHK	// ckeck sum
////////-------------------------------------------
////////EN1	Batt
////////EN2	Kick
////////EN3
////////EN4
////////-------------------------------------------
////////EOP	// end of packet



