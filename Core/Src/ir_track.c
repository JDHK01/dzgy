
#include <stdio.h>
#include "main.h"
#include "tim.h"
#include "adc.h"
#include "string.h"
#include "common.h"
#include "ir_track.h"
#include "oled_i2c.h"
#include "car_control.h"

ADC_HandleTypeDef * g_ir_track_adc = &hadc1;

__IO uint8_t g_ir_power_on_flag = 0 ;	// 1~5ms
__IO uint8_t g_ir_track_ok_flag = 0 ; // 6ms
__IO uint8_t g_ir_adc_ok_flag = 0 ;   // adc ok

uint8_t g_ir_channel       = 0 ;
track_status_t g_TrackStatus  ;

uint8_t g_calib_white_flag = 0 ;
uint8_t g_calib_black_flag = 0 ;

void IR_Track_Init( void )
{
	IR_Track_Channel_Set(5); // all power off
	memset( (void*)&g_TrackStatus , 0 , sizeof(g_TrackStatus));
	
	// 00100 				line is middle and no error
	g_TrackStatus.track_error[4] = 0;			//00100
	
	// 00010/01000 	line is left or right one sensor position, error is 18mm
	g_TrackStatus.track_error[2] = -18;		//00010
	g_TrackStatus.track_error[8] = 18;		//01000
	
	// 00001/10000 	line is left or right two sensor position, error is 36mm
	g_TrackStatus.track_error[1]  = -36;	//00001
	g_TrackStatus.track_error[16] = 36;	//10000
	
	// 00110/01100 	line is left or right about half sensor position, error is 9mm
	g_TrackStatus.track_error[6]  = -9;	//00110
	g_TrackStatus.track_error[12] = 9;	//01100
	
	// 00011/11000 	line is left or right one and half sensor position, error is 27mm	
	g_TrackStatus.track_error[3]  = -27;	//00011
	g_TrackStatus.track_error[24] = 27;	//11000

	// 00111/11100 	line is left or right one sensor position, error is 18mm	
	g_TrackStatus.track_error[7]  = -18;	//00111
	g_TrackStatus.track_error[28] = 18;	//11100
	
	
	// 11111/00000 	all black or white , start line or no line found , no error , keep straint
	g_TrackStatus.track_error[0]  = 0;		//00000
	g_TrackStatus.track_error[31] = 0;	//11111
	
	#ifdef STM32L476xx
		HAL_ADCEx_Calibration_Start( g_ir_track_adc , ADC_SINGLE_ENDED );
	#els
		HAL_ADCEx_Calibration_Start( g_ir_track_adc );	
	#endif
}

void IR_Track_Channel_Set ( uint8_t ch )
{
	uint8_t data=0 ;
	uint8_t i ;
	switch ( ch )
	{
		case 0 :
			data = 0x01; // 000 00001
			break ;
		case 1 :
			data = 0x22; // 001 00010
			break ;
		case 2 :
			data = 0x44; // 010 00100
			break ;
		case 3 :
			data = 0x68; // 011 01000
			break ;
		case 4 :
			data = 0x90; // 100 10000
			break ;
		case 5 :
			data = 0xE0; // 111 00000
			break ;
		case 6 :
			data = 0xFF; // 111 11111
			break ;
	}
	//HAL_GPIO_WritePin( IR_SCK_GPIO_Port , IR_SCK_Pin , GPIO_PIN_RESET );
  HAL_GPIO_WritePin( IR_LOCK_GPIO_Port , IR_LOCK_Pin , GPIO_PIN_RESET );

	for ( i  = 0 ; i <8 ; i++)
	{
		if ( data & 0x80 )
		{
			//HAL_GPIO_WritePin( IR_DATA_GPIO_Port , IR_DATA_Pin , GPIO_PIN_SET );
		}
		else
		{
			//HAL_GPIO_WritePin( IR_DATA_GPIO_Port , IR_DATA_Pin , GPIO_PIN_RESET );
		}
		//HAL_GPIO_WritePin( IR_SCK_GPIO_Port , IR_SCK_Pin , GPIO_PIN_SET );
		data = data << 1 ;
		//HAL_GPIO_WritePin( IR_SCK_GPIO_Port , IR_SCK_Pin , GPIO_PIN_RESET );
	}
  HAL_GPIO_WritePin( IR_LOCK_GPIO_Port , IR_LOCK_Pin , GPIO_PIN_SET );
	
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		g_ir_adc_ok_flag = 1 ;
}

void IR_Track_ADC_Start( void )
{
	if ( HAL_ADC_Start_DMA( g_ir_track_adc , (uint32_t*)&g_TrackStatus.track_adc[g_ir_channel] , 1 ) != HAL_OK )
	{
		return  ;
	}
}

void IR_Track_NormalCal(void)
{
	g_TrackStatus.adc_value = 0;
	g_TrackStatus.total_adc_value = 0 ;
	g_TrackStatus.full_white = 0;
	g_TrackStatus.full_black= 0;
	for(int i = 0; i < IR_CHANNEL_NUM; i++)
	{
		g_TrackStatus.adc_value <<= 1;
		if(g_TrackStatus.track_adc[i] > g_TrackStatus.track_compare_gate[i] )
		{
			g_TrackStatus.adc_value |= 0x01 ; // black
		}
	}
	if(g_TrackStatus.adc_value == 0)
	{
		g_TrackStatus.full_white = 1;
	}
	else if(g_TrackStatus.adc_value == 31)
	{
		g_TrackStatus.full_black = 1;
	}
}

// 1KHz 0.1ms 100us
void IR_Track_Timer( void )
{
		static uint32_t timer = 0 ;

		timer++;
		if ( timer < 6 ) {   // 1~5 ms power on 
			g_ir_power_on_flag = 1 ;
			return ;
		}
		if ( timer == 6 ) {   // start ctrl
			g_ir_track_ok_flag = 1 ;
			return ;
		}
		if ( timer == 10 ) {  // 10ms 
			timer = 0 ;
			return ;
		}	
}

void IR_Track_Menu_Calib_White( void )
{
	g_calib_white_flag = 1 ;	
}

void IR_Track_Menu_Calib_Black( void )
{
	g_calib_black_flag = 1 ;	
}

void IR_Track_Menu_Start( void )
{
	g_car_track_flag = 1 ;	
}

void IR_Track_Menu_Stop( void )
{
	g_car_track_flag = 0 ;	
}

void IR_Track_Caliberate( uint8_t mode )
{
	uint8_t buf[17] ;
	if ( mode == 0 )
	{
		for(int i = 0; i < IR_CHANNEL_NUM; i++)
		{
			g_TrackStatus.track_compare_white[i] = g_TrackStatus.track_adc[i] ;
			g_TrackStatus.track_compare_gate[i] = (g_TrackStatus.track_compare_white[i] + g_TrackStatus.track_compare_black[i])/2 ;			
		}		
		sprintf( buf , "%4d %4d %4d", 
				g_TrackStatus.track_compare_white[0] ,
				g_TrackStatus.track_compare_white[1] ,
				g_TrackStatus.track_compare_white[2] );
		OLED_ShowAscii( 0,0, buf , 16 , 0  );
		sprintf( buf , "%4d %4d", 
				g_TrackStatus.track_compare_white[3] ,
				g_TrackStatus.track_compare_white[4] );
		OLED_ShowAscii( 1,0, buf , 16 , 0  );
	}
	else
	{
		for(int i = 0; i < IR_CHANNEL_NUM; i++)
		{
			g_TrackStatus.track_compare_black[i] = g_TrackStatus.track_adc[i] ;
			g_TrackStatus.track_compare_gate[i] = (g_TrackStatus.track_compare_white[i] + g_TrackStatus.track_compare_black[i])/2 ;
		}		
		sprintf( buf , "%4d %4d %4d", 
				g_TrackStatus.track_compare_black[0] ,
				g_TrackStatus.track_compare_black[1] ,
				g_TrackStatus.track_compare_black[2] );
		OLED_ShowAscii( 0,0, buf , 16 , 0  );
		sprintf( buf , "%4d %4d", 
				g_TrackStatus.track_compare_black[3] ,
				g_TrackStatus.track_compare_black[4] );
		OLED_ShowAscii( 1,0, buf , 16 , 0  );
	}
}

void IR_Track_ShowGate( void )
{
	uint8_t buf[17] ;
	
	sprintf( buf , "%4d %4d %4d", 
				g_TrackStatus.track_compare_gate[0] ,
				g_TrackStatus.track_compare_gate[1] ,
				g_TrackStatus.track_compare_gate[2] );
	OLED_ShowAscii( 0,0, buf , 16 , 0  );
	sprintf( buf , "%4d %4d", 
				g_TrackStatus.track_compare_gate[3] ,
				g_TrackStatus.track_compare_gate[4] );
	OLED_ShowAscii( 1,0, buf , 16 , 0  );
}

void IR_Track_ShowValue( void )
{
	uint8_t buf[17] ;
	
	sprintf( buf , "%4d %4d %4d", 
				g_TrackStatus.track_adc[0] ,
				g_TrackStatus.track_adc[1] ,
				g_TrackStatus.track_adc[2] );
	OLED_ShowAscii( 0,0, buf , 16 , 0  );
	sprintf( buf , "%4d %4d", 
				g_TrackStatus.track_adc[3] ,
				g_TrackStatus.track_adc[4] );
	OLED_ShowAscii( 1,0, buf , 16 , 0  );
}

void IR_Track_Process( void )
{
	static uint16_t ir_delay_counter = 0 ;
	static uint8_t ir_delay = 0 ;
	if ( g_ir_power_on_flag == 1 )
	{
		g_ir_power_on_flag = 0 ;
		IR_Track_Channel_Set(g_ir_channel); // power on
		ir_delay_counter = 0 ;
		ir_delay = 1 ; 
		return ;
	}
	
	if ( ir_delay == 1 )
	{
		ir_delay_counter++;
		if ( ir_delay_counter == 10 )
		{
			ir_delay_counter = 0 ;
			ir_delay = 0 ;
			IR_Track_ADC_Start();
		}
		return ;		
	}

	if ( g_ir_adc_ok_flag == 1 )
	{
		g_ir_adc_ok_flag = 0 ;
		IR_Track_Channel_Set(5); // power off
		if ( g_ir_channel < (IR_CHANNEL_NUM -1))
		{
			g_ir_channel++;
		}
		else // all channel ok 
		{
			g_ir_channel = 0 ;
		}
		return ;
	}
	
	if ( g_ir_track_ok_flag == 1 )
	{
		g_ir_track_ok_flag = 0 ;
		if ( g_car_track_flag == 1 )
		{
			IR_Track_NormalCal();
			g_car_ctrl_flag = 1 ;
			return ;
		}
		if ( g_calib_white_flag == 1 )
		{
			IR_Track_Caliberate(0); 
			g_calib_white_flag = 0 ;
		}
		if ( g_calib_black_flag == 1 )
		{
			IR_Track_Caliberate(1); 
			g_calib_black_flag = 0 ;
		}
	}
}



