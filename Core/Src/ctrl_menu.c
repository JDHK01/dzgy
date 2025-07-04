#include <stdio.h>
#include <string.h>
#include "main.h"
#include "common.h"
#include "ctrl_menu.h"
#include "oled_i2c.h"
#include "ir_track.h"
#include "car_control.h"
#include "led_show.h"

// ?? ctrl_menu.c ??????
const menu_define_t g_menu[] = {
    // ???
    {"Car Control", NULL, 0, 0, 0},
    {"Steer Moto",  NULL, 0, 0, 1},
    {"Drive Moto",  NULL, 0, 0, 2},
    {"Avoid Mode",  NULL, 0, 0, 3},
    {"System Menu", NULL, 0, 0, 4},
    {"Car Mission", NULL, 0, 0, 5},  // ??????
    
    // Car Control ???
    {"Car Start",   &CarCtrl_Start,     1, 0, 0},
    {"Car Stop",    &CarCtrl_Stop,      1, 0, 1},
    {"Return",      &Ctrl_Menu_Return,  1, 0, 2},

    // Steer Moto ???
    {"Straight",    &CarCtrl_Straight,  1, 1, 0},
    {"Right",       &CarCtrl_Right,     1, 1, 1},
    {"Left",        &CarCtrl_Left,      1, 1, 2},
    {"Return",      &Ctrl_Menu_Return,  1, 1, 3},
    
    // Drive Moto ???
    {"Stop",        &CarCtrl_Stop,      1, 2, 0},
    {"SpeedUp",     &CarCtrl_SpeedUp,   1, 2, 1},
    {"SpeedDown",   &CarCtrl_SpeedDown, 1, 2, 2},
    {"Forward",     &CarCtrl_Forward,   1, 2, 3},
    {"Backword",    &CarCtrl_Backward,  1, 2, 4},
    {"Return",      &Ctrl_Menu_Return,  1, 2, 5},
    
		/*
    // Avoid Mode ???
    {"Enable",      &CarCtrl_EnableAvoid,  1, 3, 0},
    {"Disable",     &CarCtrl_DisableAvoid, 1, 3, 1},
    {"Demo",        &CarCtrl_AvoidDemo,    1, 3, 2},
    {"Return",      &Ctrl_Menu_Return,     1, 3, 3},
    */
		
    // System Menu ???
    {"UniqueID",    &Show_Unique_ID,       1, 4, 0},
    {"LED Show",    &UserLED_Show_Menu,    1, 4, 1},
    {"Ultra Wave",  &UltrasonicWave_Menu,  1, 4, 2},
    {"Music On",    &Music_Menu_On,        1, 4, 3},
    {"Music Off",   &Music_Menu_Off,       1, 4, 4},
    {"Return",      &Ctrl_Menu_Return,     1, 4, 5},
    
    // Car Mission ??? - ??
    {"Mission 1",   &CarCtrl_Mission1,     1, 5, 0},
    {"Mission 2",   &CarCtrl_Mission2,     1, 5, 1},
    {"Mission 3",   &CarCtrl_Mission3,     1, 5, 2},
    {"Return",      &Ctrl_Menu_Return,     1, 5, 3},
};


uint8_t g_menu_size = sizeof(g_menu)/sizeof(menu_define_t) ;
uint8_t g_cur_level = 0 ;
uint8_t g_cur_parent = 0 ;
uint8_t g_cur_index = 0 ;
uint8_t g_cur_level_item = 0 ;
uint8_t g_menu_index = 0 ;

__IO uint8_t g_btn0_flag = 0 ;
__IO uint8_t g_btn1_flag = 0 ;


void Ctrl_Menu_Init( void )
{
	printf("Start Ctrl Menu...\r\n");
	OLED_init();
	OLED_clear();
	Ctrl_Menu_Item();
	Ctrl_Menu_Show();
}

void Ctrl_Menu_Item( void )
{
	uint8_t i ;
	g_cur_level_item = 0 ;
	for ( i = 0 ; i < g_menu_size  ; i++ )
	{
		if ( g_menu[i].level == g_cur_level &&
         g_menu[i].parent == g_cur_parent	)
			g_cur_level_item++;
	}
}

void Ctrl_Menu_Show( void )
{
	uint8_t i ;
	uint8_t j = 0 ;
	

	for ( i = 0 ; i < g_menu_size  ; i++ )
	{
		if ( g_menu[i].level == g_cur_level &&
         g_menu[i].parent == g_cur_parent	)
		{
			if ( (g_cur_index % MENU_PAGE_SIZE) == 0 )
			{
				if ( g_menu[i].index == g_cur_index )
				{
					OLED_ShowAscii( j,0,g_menu[i].text , 16 , 1  );				
					g_menu_index = i ;
  				j++;
				}				
				else if ( g_menu[i].index > g_cur_index )
				{
					OLED_ShowAscii( j,0,g_menu[i].text , 16 , 0  );				
  				j++;
				}
			}
			else
			{
				if ( g_menu[i].index == g_cur_index )
				{
					OLED_ShowAscii( j,0,g_menu[i].text , 16 , 1  );				
					g_menu_index = i ;
					j++;
				}				
				else if ( g_menu[i].index >= ((g_cur_index/MENU_PAGE_SIZE) * MENU_PAGE_SIZE) )
				{
					OLED_ShowAscii( j,0,g_menu[i].text , 16 , 0  );				
					j++;
				}
			}
			if ( j == MENU_PAGE_SIZE ) break ;
		}
	}
	while ( j < MENU_PAGE_SIZE )
	{
		OLED_ShowAscii( j,0,"" , 16 , 0  );
		j++;
	}
}


void Ctrl_Menu_Return( void )
{
	g_cur_level -- ;
	if ( g_cur_level == 0 ) 
		g_cur_parent = 0 ;
	g_cur_index = g_menu[g_menu_index].parent ;
	Ctrl_Menu_Item();
	Ctrl_Menu_Show();	
}


void Ctrl_Menu_Process( void )
{
	if ( g_btn0_flag == 1 )
	{
		g_btn0_flag = 0 ;
		g_cur_index++;
		if ( g_cur_index == g_cur_level_item ) g_cur_index = 0;
		Ctrl_Menu_Show();
	}
	if ( g_btn1_flag == 1 )
	{
		g_btn1_flag = 0 ;
		if ( g_menu[g_menu_index].func_callback == NULL )
		{
			g_cur_level ++ ;
			g_cur_parent = g_cur_index ;
			g_cur_index = 0 ;
			Ctrl_Menu_Item();
			Ctrl_Menu_Show();
		}
		else
		{
			g_menu[g_menu_index].func_callback();
		}
	}	
}
