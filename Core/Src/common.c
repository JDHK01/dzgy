#include <stdio.h>
#include <string.h>
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"
#include "common.h"
#include "music.h"
#include "led.h"
#include "uart_dma.h"
#include "oled_i2c.h"
#include "motor_drive.h"
#include "speed_encoder.h"
#include "ir_track.h"
#include "UltrasonicWave.h"
#include "car_control.h"
#include "ctrl_menu.h"
#include "led_show.h"


uint8_t g_device_uid[25]={0};

UART_HandleTypeDef *g_user_uart = &huart1;  // printf output
TIM_HandleTypeDef *g_sys_timer = &htim6;

int fputc(int ch,FILE *p) 
{
	HAL_UART_Transmit( g_user_uart, (uint8_t *)&ch, 1, 10 );
	return ch ;   
}

//timer interrupt

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t counter = 0 ;
  if ( htim == &htim5 )  // ultrawave
	{
		if ( g_ultrawave_state == ULTRAWAVE_STATE_TRIG ) // 
		{
			g_ultrawave_state = ULTRAWAVE_STATE_TRIG_OK ;
		}
		else if ( g_ultrawave_state == ULTRAWAVE_STATE_WAIT_ECHO_START ||  // no rising
			        g_ultrawave_state == ULTRAWAVE_STATE_WAIT_ECHO_STOP )   // no falling
		{
			g_ultrawave_state = ULTRAWAVE_STATE_TIMEOUT ;
		}
	}
	else if(htim == &htim6)	// 100Hz 10ms
	{
		counter++;
		
		if ( g_car_ctrl_state == CarCtrl_IDLE ) 
		{
			Speed_Encoder_Update();
			g_car_ctrl_state = CarCtrl_START ;
		}
		
		if ( counter == 10 ) // 10 Hz
		{
			g_ultrawave_state = ULTRAWAVE_STATE_EN ;
			counter = 0 ;
		}
	}
	else if(htim == &htim7)	// led show & music
	{
			if ( g_music_state == MUSIC_IDLE ) 
				g_music_state = MUSIC_START ;
			if ( g_LED_Show_state == LEDSHOW_IDLE ) 
				g_LED_Show_state = LEDSHOW_START ;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	UltrasonicWave_Echo(GPIO_Pin);
	if ( GPIO_Pin == BTN1_Pin )
	{
		g_btn0_flag = 1;
		return ;
	}
	if ( GPIO_Pin == BTN2_Pin )
	{
		g_btn1_flag = 1;
		return ;
	}
	if ( GPIO_Pin == GPIO_PIN_3 )
	{
		g_LED_key_down = 1 ;
	}
}


//////////////////////////////////////////////////////////////////////////////////////////////


static void GetUniqueID(void)
{
    uint8_t uid[12]={0};

    uid[0] = UNI_DEV_SN->SN[11];
    uid[1] = UNI_DEV_SN->SN[10];
    uid[2] = UNI_DEV_SN->SN[9];
    uid[3] = UNI_DEV_SN->SN[8];
    uid[4] = UNI_DEV_SN->SN[7];
    uid[5] = UNI_DEV_SN->SN[6];
    uid[6] = UNI_DEV_SN->SN[5];
    uid[7] = UNI_DEV_SN->SN[4];
    uid[8] = UNI_DEV_SN->SN[3];
    uid[9] = UNI_DEV_SN->SN[2];
    uid[10] = UNI_DEV_SN->SN[1];
    uid[11] = UNI_DEV_SN->SN[0];

    sprintf((char*)g_device_uid, "%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X%02X",
            uid[0],uid[1],uid[2],uid[3],uid[4],uid[5],
            uid[6],uid[7],uid[8],uid[9],uid[10],uid[11]);
}

void Show_Unique_ID( void )
{
	OLED_ShowAscii( 0,0,g_device_uid , 16 , 0  );		
	OLED_ShowAscii( 1,0,g_device_uid+16 , 16 , 0  );		
}

void System_Timer_Start( void )
{
		HAL_TIM_Base_Stop_IT( g_sys_timer );
		__HAL_TIM_SET_PRESCALER(g_sys_timer , 35999 ); // 72000000/36000=2000
		__HAL_TIM_SET_AUTORELOAD( g_sys_timer , 19 ); // 2000/20=100
		__HAL_TIM_SET_COUNTER( g_sys_timer , 0 );
		HAL_TIM_Base_Start_IT( g_sys_timer );
}

void System_Init( void )
{
	StartAllUartDMAReceive();
	UserLED_Init();	
  GetUniqueID();
	Moto_Ctrl_Init();
	Speed_Encoder_Init();
	UltrasonicWave_Init();
	Ctrl_Menu_Init();
	CarCtrl_Init();
	System_Timer_Start();
	
	printf("MCU Init OK!\n");
}

void UserTasks( void)
{
	UltrasonicWave_Process();
	CheckUartRxData();
	CheckUartTxData();
	Ctrl_Menu_Process();
	CarCtrl_Process();
	Music_Process();
	UserLEDShowProcess();
}
