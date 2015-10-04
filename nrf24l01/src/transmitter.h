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

#define HEADER_LENGHT 4
#define STOP_BYTE_SIZE 1
#define Data_Lenght 18 // old length 11
#define Max_Robot 12
#define START_BYTE0 0xA5
#define START_BYTE1 0x5A
#define STOP_BYTE 0x80
#define R 0
#define L 1
#define high 1
#define	low	 0
uint8_t PCK_Num[2]={0,0};
extern char Buf_Tx[2][Max_Robot][_Buffer_Size];
extern uint16_t pck_timeout[2][Max_Robot];
struct PCK_Header
{
	uint8_t SIB;
	uint8_t CHK;
};

union Motor_Speed
{
	char Bytes[2];
	int16_t Speed;
};

typedef union High_Low{
	uint8_t byte[2] ;
	int16_t full ;
} HL;

struct Robot_Data
{
	uint8_t RID ;
	HL Vx_sp ;
	HL Vy_sp ;
	HL Wr_sp ;
	HL Vx ;
	HL Vy ;
	HL Wr ;
	HL alpha ;
	uint8_t KICK ;
	uint8_t CHIP ;
	uint8_t SPIN ;
};

struct PCK_Send_Header
{
	uint8_t SIB;
	uint8_t CHK;
	uint8_t RID;
};


struct PCK_Header PCK_H[2];
struct Robot_Data Robot_D_tmp[2][Max_Robot];


// receiving data for 9 robots(SIB = 104 ),why? should be 12 robots !!
// running time : about 560 clk
// time for receiving all packet : 560 * 104 clk = 58240 clk = 1.82 ms
static void GetNewData(uint8_t data,int side)
{
	if (PCK_Num[side]<HEADER_LENGHT)
	{
		switch(PCK_Num[side])
		{
			case 0:
			if (data == START_BYTE0)
			{
				PCK_Num[side]++;
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
			uint8_t id = (PCK_Num[side]-HEADER_LENGHT) / Data_Lenght ;
			
			switch((PCK_Num[side]-HEADER_LENGHT) % Data_Lenght)
			{
				case 0:
				Robot_D_tmp[side][id].RID=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 1:
				Robot_D_tmp[side][id].Vx_sp.byte[high]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 2:
				Robot_D_tmp[side][id].Vx_sp.byte[low]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 3:
				Robot_D_tmp[side][id].Vy_sp.byte[high]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 4:
				Robot_D_tmp[side][id].Vy_sp.byte[low]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 5:
				Robot_D_tmp[side][id].Wr_sp.byte[high]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 6:
				Robot_D_tmp[side][id].Wr_sp.byte[low]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 7:
				Robot_D_tmp[side][id].Vx.byte[high]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 8:
				Robot_D_tmp[side][id].Vx.byte[low]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 9:
				Robot_D_tmp[side][id].Vy.byte[high]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 10:
				Robot_D_tmp[side][id].Vy.byte[low]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 11:
				Robot_D_tmp[side][id].Wr.byte[high]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 12:
				Robot_D_tmp[side][id].Wr.byte[low]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 13:
				Robot_D_tmp[side][id].alpha.byte[high]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 14:
				Robot_D_tmp[side][id].alpha.byte[low]=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 15:
				Robot_D_tmp[side][id].KICK=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 16:
				Robot_D_tmp[side][id].CHIP=data;
				PCK_H[side].CHK -= data;
				break;
				
				case 17:
				Robot_D_tmp[side][id].SPIN=data;
				PCK_H[side].CHK -= data;
				break;
				
			}
			PCK_Num[side]++;
		}
		else
		{
			if (PCK_H[side].CHK == 0 && data == STOP_BYTE)
			{
				uint8_t number_of_robots = (PCK_H[side].SIB - HEADER_LENGHT - STOP_BYTE_SIZE)/Data_Lenght ;
				for (uint8_t i=0;i<number_of_robots;i++)
				{
					
					if (Robot_D_tmp[side][i].RID != 255)
					{
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 0] = Robot_D_tmp[side][i].RID ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 1] = Robot_D_tmp[side][i].Vx_sp.byte[high] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 2] = Robot_D_tmp[side][i].Vx_sp.byte[low] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 3] = Robot_D_tmp[side][i].Vy_sp.byte[high] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 4] = Robot_D_tmp[side][i].Vy_sp.byte[low] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 5] = Robot_D_tmp[side][i].Wr_sp.byte[high] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 6] = Robot_D_tmp[side][i].Wr_sp.byte[low] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 7] = Robot_D_tmp[side][i].Vx.byte[high] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 8] = Robot_D_tmp[side][i].Vx.byte[low] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][ 9] = Robot_D_tmp[side][i].Vy.byte[high] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][10] = Robot_D_tmp[side][i].Vy.byte[low] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][11] = Robot_D_tmp[side][i].Wr.byte[high] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][12] = Robot_D_tmp[side][i].Wr.byte[low] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][13] = Robot_D_tmp[side][i].alpha.byte[high] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][14] = Robot_D_tmp[side][i].alpha.byte[low] ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][15] = Robot_D_tmp[side][i].KICK ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][16] = Robot_D_tmp[side][i].CHIP ;
						Buf_Tx[side][Robot_D_tmp[side][i].RID][17] = Robot_D_tmp[side][i].SPIN ;

						pck_timeout[side][Robot_D_tmp[side][i].RID]=0;
						
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



