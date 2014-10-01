/*
 * Menu.c
 *
 * Created: 3/22/2013 1:39:56 PM
 *  Author: Milad
 */

#include <Menu.h>

int SegR[18] = {Segment_0,Segment_1,Segment_2,Segment_3,Segment_4,Segment_5,Segment_6,Segment_7,Segment_8,Segment_9,
				Segment_10,Segment_11,Segment_12,Segment_13,Segment_14,Segment_15,Segment_Dash};



bool Menu_Set_flg=false,Menu_Cancel_flg=false;
uint8_t menu_index[3]={Menu_Clear,Menu_Clear,0};
uint8_t disp;
uint16_t menu_time = 0;

void Menu_Reset(void)
{
    uint8_t i=0;
    for (i=0;i<16;i++)
    {
        menu_index[1] = i;
        menu_index[0] = Menu_Default;
        menu_table();
    }
    menu_index[0] = Menu_Clear;
    menu_index[1] = Menu_Clear;
    menu_index[2] = 0;
}

bool menu_check_sw(uint8_t SW,uint8_t *SW_Flag)
{
    if(SW && !(*SW_Flag))
    {
        *SW_Flag = true;
        return 1;
    }
    else if(SW == 0)
        *SW_Flag = false;
    return 0;
}

void menu_check_status(void)
{
	switch(menu_index[2])
	{
		case 0://Wait for Index0
		Segment_PORT.OUT = (Segment_PORT.OUT & ~(1<<Segment_DP_bp)) | (((time_ms / 100) & 1) << Segment_DP_bp);
		Segment_PORT.OUTCLR = Segment_DP_bm;
		menu_index[1] = Menu_Num;
		menu_index[0] = Menu_Clear;
		Menu_Display();
		if(menu_check_sw(Menu_Set,&Menu_Set_flg))
		{
			menu_index[2]++;
		}
		if(menu_check_sw(Menu_Cancel,&Menu_Cancel_flg))
		{
			 menu_time=100;
		}
		break;
		case 1://Wait for Index1
		Segment_PORT.OUT = (Segment_PORT.OUT & ~(1<<Segment_DP_bp)) | (((time_ms / 100) & 1) << Segment_DP_bp);
		Segment_PORT.OUTSET = Segment_DP_bm;
		menu_index[0] = Menu_Num;
		Menu_Display();
		if(menu_check_sw(Menu_Set,&Menu_Set_flg))
		{
			menu_index[2]++;
		}
		if(menu_check_sw(Menu_Cancel,&Menu_Cancel_flg))
		{
			menu_index[2]--;
		}
		break;
		case 2:
		Segment_PORT.OUTSET = Segment_DP_bm;
		Segment_PORT.OUTSET = Segment_DP_bm;
		menu_table();
		if(menu_check_sw(Menu_Cancel,&Menu_Cancel_flg))
		{
			menu_index[2]--;
			menu_index[0] = Menu_Default;
			menu_table();
			menu_index[0] = Menu_Clear;
			Menu_Display();
		}

		break;
	}
}

void Menu_Display(void)
{
    Disp_R(menu_index[0]);
    Disp_L(menu_index[1]);
}

void menu_table(void)
{

    switch(menu_index[1])
    {
    case 1:
        switch(menu_index[0])
        {
        case 1:
            Menu_table_11();
            break;
        case 2:
            Menu_table_12();
            break;
        case 3:
            Menu_table_13();
            break;
        case 4:
            Menu_table_14();
            break;
        case 5:
            Menu_table_15();
            break;
        case 6:
            Menu_table_16();
            break;
        case Menu_Default:
            Menu_table_1default();
            break;
        }
        break;
    case 2:
        switch(menu_index[0])
        {
        case 0:
            break;
        case 1:
            Menu_table_21();
            break;
        case 2:
            Menu_table_22();
            break;
        case 3:
            Menu_table_23();
            break;
        case 4:
            Menu_table_24();
            break;
        case Menu_Default:
			Menu_table_2default();
            break;
        }
        break;
	case 0xA:
		switch(menu_index[0])
		{
		case 0xB:
			Menu_table_AB();
			break;
		case 0xC:
			Menu_table_AC();
			break;
		case 0xD:
		menu_table_AD();
		break;
				case 0xE:
				Menu_table_AE();
				break;
		case Menu_Default:
			Menu_table_Adefault();
			break;
			
		}
		break;
	case 0xE:
		switch(menu_index[0])
		{
		case 0xF:
			break;
		}
		break;
	case Menu_Default:
		break;
    }
}

void Menu_table_11(void)
{
}
void Menu_table_12(void)
{
}
void Menu_table_13(void)
{
}
void Menu_table_14(void)
{
}
void Menu_table_15(void)
{
}
void Menu_table_16(void)
{
}
void Menu_table_1default(void)
{
}
void Menu_table_20()
{
}

void Menu_table_21(void)
{
}
void Menu_table_22(void)
{
}
void Menu_table_23(void)
{
}
void Menu_table_24(void)
{
}
void Menu_table_2default()
{
}

int timer_tmp=0;
void Menu_table_30(void)
{
}

void Menu_table_3default(void)
{
}

void Menu_table_AB(void)
{
}
void Menu_table_AC(void)
{
}
void menu_table_AD()
{
}
void Menu_table_AE(void)
{
}
void Menu_table_Adefault(void)
{
}