/*
 * Menu.h
 *
 * Created: 3/22/2012 3:35:24 PM
 *  Author: Milad
 */ 


#ifndef MENU_H_
#define MENU_H_

#include <asf.h>
#include "initialize.h"

#define Menu_Disp_OFF 0x00
#define Menu_Disp_ON 0xFF
#define Menu_Disp(_A_) disp = _A_
#define Menu_Cancel (Menu_PORT.IN & Menu_Cancel_PIN_bm) >> Menu_Cancel_PIN_bp
#define Menu_Set (Menu_PORT.IN & Menu_Set_PIN_bm) >> Menu_Set_PIN_bp
#define Menu_Num ((((Menu_PORT.IN & Menu_PIN0_bm) >> Menu_PIN0_bp) << 0) |\
				  (((Menu_PORT.IN & Menu_PIN2_bm) >> Menu_PIN2_bp) << 2) |\
				  (((Menu_PORT.IN & Menu_PIN1_bm) >> Menu_PIN1_bp) << 1) |\
				  (((Menu_PORT.IN & Menu_PIN3_bm) >> Menu_PIN3_bp) << 3))


#define Disp_R(_A_) Segment_PORT.OUT = (SegR[_A_] |(Segment_PORT.OUT & Segment_DP_bm))& disp
#define Disp_L(_A_) Segment_PORT.OUT = (SegL[_A_] |(Segment_PORT.OUT & Segment_DP_bm))& disp
#define Disp_R_DP(_A_) Segment_PORT.OUT = (Segment_PORT.OUT & ~Segment_DP_bm) | (_A_<<Segment_DP_bp)
#define Disp_L_DP(_A_) Segment_PORT.OUT = (Segment_PORT.OUT & ~Segment_DP_bm) | (_A_<<Segment_DP_bp)

#define Menu_Default 17
#define Menu_Clear 16

extern int SegR[18];
extern uint8_t SegL[18];

extern bool setMotor;
extern bool Use_PID;
extern uint16_t M1_SetPoint,M2_SetPoint,M3_SetPoint;
extern uint16_t M1_MaxSpeed,M2_MaxSpeed,M3_MaxSpeed;
extern uint32_t M1_MaxSpeed_delay,M2_MaxSpeed_delay,M3_MaxSpeed_delay;
extern uint16_t M1_RPM,M2_RPM,M3_RPM;//Motors Speed (rpm)

extern bool Menu_Set_flg,Menu_Cancel_flg;

extern uint8_t disp;
extern uint8_t menu_index[3];

extern uint16_t menu_time;
extern uint32_t time_ms;
void menu_table(void);
void Menu_Reset(void);
bool menu_check_sw(uint8_t SW,uint8_t *SW_Flag);
void menu_check_status(void);
void Menu_Display(void);

void Menu_table_11(void);
void Menu_table_12(void);
void Menu_table_13(void);
void Menu_table_14(void);
void Menu_table_15(void);
void Menu_table_16(void);
void Menu_table_1default(void);

void Menu_table_20(void);
void Menu_table_21(void);
void Menu_table_22(void);
void Menu_table_23(void);
void Menu_table_24(void);
void Menu_table_2default(void);

void Menu_table_30(void);
void Menu_table_3default(void);

void Menu_table_AB(void);
void Menu_table_AC(void);
void menu_table_AD(void);
void Menu_table_AE(void);
void Menu_table_Adefault(void);
#endif /* MENU_H_ */