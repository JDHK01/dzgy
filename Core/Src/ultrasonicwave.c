#include <stdio.h>
#include <string.h>
#include "main.h"
#include "tim.h"
#include "UltrasonicWave.h"
#include "oled_i2c.h"

TIM_HandleTypeDef *g_ultra_wave_timer = &htim5;

__IO UltraWave_StateTypeDef g_ultrawave_state = ULTRAWAVE_STATE_IDLE ;
__IO uint8_t  g_ultrawave_pos  = 0 ;

UltraWave_DataTypeDef g_ultrawave_data[ULTRAWAVE_NUM] =
{
	{ TRIG_F_GPIO_Port , TRIG_F_Pin , ECHO_F_GPIO_Port , ECHO_F_Pin , EXTI9_5_IRQn , 0 , 0xffffffff ,  },
	{ TRIG_B_GPIO_Port , TRIG_B_Pin , ECHO_B_GPIO_Port , ECHO_B_Pin , EXTI15_10_IRQn , 0 , 0xffffffff ,  },
};

// interrupt
void UltrasonicWave_Echo(uint16_t GPIO_Pin)
{
	if ( g_ultrawave_state == ULTRAWAVE_STATE_WAIT_ECHO_START )
	{
		if ( g_ultrawave_data[g_ultrawave_pos].Echo_Pin == GPIO_Pin )
		{
			if ( HAL_GPIO_ReadPin(g_ultrawave_data[g_ultrawave_pos].Echo_Port  , g_ultrawave_data[g_ultrawave_pos].Echo_Pin) == GPIO_PIN_SET )
			{
				HAL_TIM_Base_Stop_IT(g_ultra_wave_timer);
				__HAL_TIM_SET_COUNTER( g_ultra_wave_timer , 0 );
				HAL_TIM_Base_Start_IT(g_ultra_wave_timer);
				g_ultrawave_state = ULTRAWAVE_STATE_WAIT_ECHO_STOP ;				
			}
		}
	}
	else if ( g_ultrawave_state == ULTRAWAVE_STATE_WAIT_ECHO_STOP )
	{
		if ( g_ultrawave_data[g_ultrawave_pos].Echo_Pin == GPIO_Pin )
		{
			if ( HAL_GPIO_ReadPin(g_ultrawave_data[g_ultrawave_pos].Echo_Port  , g_ultrawave_data[g_ultrawave_pos].Echo_Pin) == GPIO_PIN_RESET )
			{
				g_ultrawave_data[g_ultrawave_pos].counter = __HAL_TIM_GET_COUNTER( g_ultra_wave_timer );			
				HAL_TIM_Base_Stop_IT(g_ultra_wave_timer);
				g_ultrawave_state = ULTRAWAVE_STATE_CAL_DISTANCE ;
			}
		}
	}
}


void UltrasonicWave_Init( void ) 
{
	g_ultrawave_state = ULTRAWAVE_STATE_IDLE ;
	g_ultrawave_pos = 0 ;

	for (int i = 0 ; i < ULTRAWAVE_NUM ; i++) 
	{
		g_ultrawave_data[i].counter = 0 ;
		g_ultrawave_data[i].distance = 0 ;
		HAL_GPIO_WritePin( g_ultrawave_data[i].Trig_Port  , g_ultrawave_data[i].Trig_Pin , GPIO_PIN_RESET );		
		HAL_NVIC_DisableIRQ( g_ultrawave_data[i].IRQn ); 
	}

	HAL_TIM_Base_Stop_IT(g_ultra_wave_timer);
}

void UltrasonicWave_Trig_Start( void ) 
{

	g_ultrawave_data[g_ultrawave_pos].counter = 0 ;
	g_ultrawave_data[g_ultrawave_pos].distance = 0xffffffff ;
	HAL_GPIO_WritePin( g_ultrawave_data[g_ultrawave_pos].Trig_Port  , g_ultrawave_data[g_ultrawave_pos].Trig_Pin , GPIO_PIN_SET );		
	HAL_NVIC_DisableIRQ( g_ultrawave_data[g_ultrawave_pos].IRQn ); 
		
	HAL_TIM_Base_Stop_IT(g_ultra_wave_timer);
	__HAL_TIM_SET_PRESCALER( g_ultra_wave_timer , 71 );   // 72M / 72 = 1MHz(1 us )
	__HAL_TIM_SET_AUTORELOAD( g_ultra_wave_timer , 19 );   // 1us *20 = 20us
	__HAL_TIM_SET_COUNTER( g_ultra_wave_timer , 0 );
	HAL_TIM_Base_Start_IT(g_ultra_wave_timer);
}

void UltrasonicWave_Trig_Stop( void )  
{
	HAL_GPIO_WritePin( g_ultrawave_data[g_ultrawave_pos].Trig_Port  , g_ultrawave_data[g_ultrawave_pos].Trig_Pin , GPIO_PIN_RESET );		
	HAL_NVIC_EnableIRQ( g_ultrawave_data[g_ultrawave_pos].IRQn ); 

	HAL_TIM_Base_Stop_IT(g_ultra_wave_timer);
	__HAL_TIM_SET_PRESCALER( g_ultra_wave_timer , 71 );   // 72M / 72 = 1MHz(1 us )
	__HAL_TIM_SET_AUTORELOAD( g_ultra_wave_timer , 49999 );   // 1us *50000 = 50ms
	__HAL_TIM_SET_COUNTER( g_ultra_wave_timer , 0 );
	HAL_TIM_Base_Start_IT(g_ultra_wave_timer);
}

void UltrasonicWave_Menu( void ) 
{
	if ( g_ultrawave_state == ULTRAWAVE_STATE_IDLE ) 
		g_ultrawave_state = ULTRAWAVE_STATE_EN ;
}

void UltrasonicWave_ShowDistance( void ) 
{
	uint8_t buf[17];
	
	for (int i = 0 ; i < ULTRAWAVE_NUM ; i++) 
	{
		if ( g_ultrawave_data[i].distance < 100 ) sprintf( buf , "%d: -- mm" , i );
		else sprintf( buf , "%d: %4.1f mm" , i , g_ultrawave_data[i].distance/100.0 );
		OLED_ShowAscii( i,0, buf , 16 ,0 );
	}
}

void UltrasonicWave_Process( void ) 
{
	if ( g_ultrawave_state == ULTRAWAVE_STATE_IDLE )
	{
		return ;
	}
	else if ( g_ultrawave_state == ULTRAWAVE_STATE_EN )
	{
		g_ultrawave_state = ULTRAWAVE_STATE_TRIG ; 
		UltrasonicWave_Trig_Start();
	}
	else if ( g_ultrawave_state == ULTRAWAVE_STATE_TRIG_OK )
	{
		g_ultrawave_state = ULTRAWAVE_STATE_WAIT_ECHO_START ; 
		UltrasonicWave_Trig_Stop( );
	}
	else if ( g_ultrawave_state == ULTRAWAVE_STATE_CAL_DISTANCE )
	{
		g_ultrawave_data[g_ultrawave_pos].distance = g_ultrawave_data[g_ultrawave_pos].counter * 17 ; // 340000 mm / 1000000us / 2 = 0.17 mm/us * 100
		g_ultrawave_state = ULTRAWAVE_STATE_TIMEOUT ; 
	}
	else if ( g_ultrawave_state == ULTRAWAVE_STATE_TIMEOUT )
	{
		if ( g_ultrawave_pos < (ULTRAWAVE_NUM -1) ) 
		{
			g_ultrawave_pos ++ ;
			g_ultrawave_state = ULTRAWAVE_STATE_EN ;
		}
		else
		{
			g_ultrawave_pos = 0 ;
			g_ultrawave_state = ULTRAWAVE_STATE_IDLE ;
			//g_ultrawave_state = ULTRAWAVE_STATE_SHOW ;
		}		
	}	
	else if ( g_ultrawave_state == ULTRAWAVE_STATE_SHOW )
	{
		g_ultrawave_state = ULTRAWAVE_STATE_IDLE ;
		UltrasonicWave_ShowDistance();
	}

}
