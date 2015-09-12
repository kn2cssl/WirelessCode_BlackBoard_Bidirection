/*
 * initialize.h
 *
 * Created: 3/22/2012 12:50:44 AM
 *  Author: Milad
 */ 


#ifndef INITIALIZE_H_
#define INITIALIZE_H_
#include <asf.h>

#define F_CPU 32000000UL


#define ON 1
#define OFF 0

#define Out_Set(_PORT_,_PIN_bp_,_STATUS_ ) _PORT_.OUT = (_PORT_.OUT & ~(1<<_PIN_bp_)) | (_STATUS_<<_PIN_bp_)
#define LED_White_R_PORT		PORTB
#define LED_Green_R_PORT		PORTB
#define LED_White_L_PORT		PORTB
#define LED_Green_L_PORT		PORTB
#define LED_White_R_PIN_bp		PIN7_bp
#define LED_White_R_PIN_bm		PIN7_bm
#define LED_Green_R_PIN_bp		PIN6_bp
#define LED_Green_R_PIN_bm		PIN6_bm
#define LED_White_L_PIN_bp		PIN5_bp
#define LED_White_L_PIN_bm		PIN5_bm
#define LED_Green_L_PIN_bp		PIN4_bp
#define LED_Green_L_PIN_bm		PIN4_bm
#define LED_White_R(_STATUS_)	Out_Set(LED_White_R_PORT,LED_White_R_PIN_bp,_STATUS_ ) 
#define LED_Green_R(_STATUS_)	Out_Set(LED_Green_R_PORT,LED_Green_R_PIN_bp,_STATUS_ ) 
#define LED_White_L(_STATUS_)	Out_Set(LED_White_L_PORT,LED_White_L_PIN_bp,_STATUS_ ) 
#define LED_Green_L(_STATUS_)   Out_Set(LED_Green_L_PORT,LED_Green_L_PIN_bp,_STATUS_ ) 
 
#define Buzzer_PORT				PORTD
#define Buzzer_PIN_bp			PIN7_bp
#define Buzzer_PIN_bm			PIN7_bm
#define Buzzer(_STATUS_)		Out_Set(Buzzer_PORT,Buzzer_PIN_bp,_STATUS_ )

#define Wireless_R_PORT PORTC
#define Wireless_L_PORT PORTE
#define Wireless_R_USART USARTC0
#define Wireless_L_USART USARTE0
#define Wireless_TX_R_PIN_bm PIN3_bm
#define Wireless_RX_R_PIN_bm PIN2_bm
#define Wireless_TX_L_PIN_bm PIN3_bm
#define Wireless_RX_L_PIN_bm PIN2_bm
#define USART_R_RXC_vect USARTC0_RXC_vect
#define USART_R_DRE_vect USARTC0_DRE_vect
#define USART_L_RXC_vect USARTE0_RXC_vect
#define USART_L_DRE_vect USARTE0_DRE_vect

#define Menu_PIN3_bm PIN1_bm
#define Menu_PIN3_bp PIN1_bp
#define Menu_PIN2_bm PIN4_bm
#define Menu_PIN2_bp PIN4_bp
#define Menu_PIN1_bm PIN3_bm
#define Menu_PIN1_bp PIN3_bp
#define Menu_PIN0_bm PIN2_bm
#define Menu_PIN0_bp PIN2_bp
#define Menu_Set_PIN_bm PIN7_bm
#define Menu_Set_PIN_bp PIN7_bp
#define Menu_Cancel_PIN_bm PIN0_bm
#define Menu_Cancel_PIN_bp PIN0_bp
#define Menu_Side_Switch_PIN_bm PIN6_bm
#define Menu_Side_Switch_PIN_bp PIN6_bp
#define Menu_Side_Select_PIN_bm PIN5_bm
#define Menu_Side_Select_PIN_bp PIN5_bp
#define Menu_PORT PORTA

#define Segment_A_bm PIN4_bm
#define Segment_B_bm PIN6_bm
#define Segment_C_bm PIN5_bm
#define Segment_D_bm PIN2_bm
#define Segment_E_bm PIN0_bm
#define Segment_F_bm PIN1_bm
#define Segment_G_bm PIN3_bm
#define Segment_DP_bm PIN7_bm
#define Segment_DP_bp PIN7_bp

#define Segment_0  Segment_A_bm | Segment_B_bm | Segment_C_bm | Segment_D_bm | Segment_E_bm | Segment_F_bm
#define Segment_1  Segment_B_bm | Segment_C_bm
#define Segment_2  Segment_A_bm | Segment_B_bm | Segment_G_bm | Segment_E_bm | Segment_D_bm 
#define Segment_3  Segment_A_bm | Segment_B_bm | Segment_C_bm | Segment_D_bm | Segment_G_bm 
#define Segment_4  Segment_F_bm | Segment_G_bm | Segment_B_bm | Segment_C_bm
#define Segment_5  Segment_A_bm | Segment_F_bm | Segment_G_bm | Segment_C_bm | Segment_D_bm 
#define Segment_6  Segment_A_bm | Segment_F_bm | Segment_G_bm | Segment_C_bm | Segment_D_bm | Segment_E_bm
#define Segment_7  Segment_F_bm | Segment_A_bm | Segment_B_bm | Segment_C_bm 
#define Segment_8  Segment_A_bm | Segment_B_bm | Segment_C_bm | Segment_D_bm | Segment_E_bm | Segment_F_bm | Segment_G_bm
#define Segment_9  Segment_G_bm | Segment_F_bm | Segment_A_bm | Segment_B_bm | Segment_C_bm | Segment_D_bm
#define Segment_10 Segment_E_bm | Segment_F_bm | Segment_A_bm | Segment_B_bm | Segment_E_bm | Segment_G_bm 
#define Segment_11 Segment_F_bm | Segment_G_bm | Segment_C_bm | Segment_D_bm | Segment_E_bm
#define Segment_12 Segment_A_bm | Segment_F_bm | Segment_E_bm | Segment_D_bm
#define Segment_13 Segment_G_bm | Segment_E_bm | Segment_D_bm | Segment_C_bm | Segment_B_bm
#define Segment_14 Segment_A_bm | Segment_F_bm | Segment_E_bm | Segment_D_bm | Segment_G_bm
#define Segment_15 Segment_A_bm | Segment_F_bm | Segment_G_bm | Segment_E_bm
#define Segment_Dash Segment_G_bm

#define Segment_PORT PORTF
#define Segment_SideR_L_Enable_bm PIN0_bm
#define Segment_SideR_L_Enable_bp PIN0_bp
#define Segment_SideR_R_Enable_bm PIN1_bm
#define Segment_SideR_R_Enable_bp PIN1_bp
#define Segment_SideL_L_Enable_bm PIN3_bm
#define Segment_SideL_L_Enable_bp PIN3_bp
#define Segment_SideL_R_Enable_bm PIN2_bm
#define Segment_SideL_R_Enable_bp PIN2_bp


#define NRF24L01_R_SPI			SPIC
#define NRF24L01_R_PORT			PORTC

#define NRF24L01_R_CE_LINE		PIN1_bm
#define NRF24L01_R_CS_LINE		PIN4_bm
#define NRF24L01_R_MOSI_LINE	PIN5_bm
#define NRF24L01_R_MISO_LINE	PIN6_bm
#define NRF24L01_R_SCK_LINE		PIN7_bm
#define NRF24L01_R_IRQ_LINE		PIN0_bm

#define NRF24L01_L_SPI			SPIE
#define NRF24L01_L_PORT			PORTE

#define NRF24L01_L_CE_LINE		PIN1_bm
#define NRF24L01_L_CS_LINE		PIN4_bm
#define NRF24L01_L_MOSI_LINE	PIN5_bm
#define NRF24L01_L_MISO_LINE	PIN6_bm
#define NRF24L01_L_SCK_LINE		PIN7_bm
#define NRF24L01_L_IRQ_LINE		PIN0_bm

#define PRX_R PORTC_INT0_vect
#define PRX_L PORTE_INT0_vect

void En_RC32M(void);
void PORT_init(void);

void TimerD0_init(void);

void TimerE0_init(void);

void TimerE1_init(void);


#define USART_BUADRATE 115200
void USART_R_init(void);
void USART_L_init(void);

void OUT_Bling(PORT_t *OUT_PORT,uint8_t OUT_PIN_bp,uint8_t Speed,uint32_t *Time_ON,uint32_t time_ms);

#endif /* INITIALIZE_H_ */