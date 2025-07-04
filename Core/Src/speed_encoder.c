#include "speed_encoder.h"
#include "oled_i2c.h"
	
__IO Encoder_StateTypeDef g_speed_encoder_state = ENCODER_STATE_IDLE ; 


__IO Speed_Encoder_t g_speed_encoder[SPEED_ENCODER_NUM] = 
{
	{ &htim3 , 0 , 0.0 }, // LEFT
	{ &htim8 , 0 , 0.0 }, // RIGHT    
};

void Speed_Encoder_Init(void)
{
	for ( int i = 0 ; i < SPEED_ENCODER_NUM ; i++)
	{
		__HAL_TIM_SET_COUNTER( g_speed_encoder[ i ].timer , 0 );
		HAL_TIM_Encoder_Start( g_speed_encoder[ i ].timer , TIM_CHANNEL_ALL );
	}
}

void Speed_Encoder_Update( void )
{
	for ( int i = 0 ; i < SPEED_ENCODER_NUM ; i++)
	{
		g_speed_encoder[i].counter  = __HAL_TIM_GET_COUNTER(g_speed_encoder[ i ].timer);
		__HAL_TIM_SET_COUNTER( g_speed_encoder[ i ].timer , 0 );
	}	
	g_speed_encoder_state = ENCODER_STATE_CAL_SPEED ;
}

void Speed_Calculate( void )
{
	g_speed_encoder[ 0 ].speed = g_speed_encoder[0].counter * SPEED_COEFF / 10000 ;
	g_speed_encoder[ 1 ].speed = g_speed_encoder[1].counter * -SPEED_COEFF / 10000 ;
}

void Speed_Process( void )
{
	if ( g_speed_encoder_state == ENCODER_STATE_CAL_SPEED )
	{
		g_speed_encoder_state = ENCODER_STATE_IDLE ;
		Speed_Calculate();
		Speed_Show();
	}
}



void Speed_Show( void ) 
{
	static uint8_t  index = 0 ;
	static int32_t    speed[SPEED_ENCODER_NUM] = { 0 , 0 } ;
	static int32_t  counter[SPEED_ENCODER_NUM] = { 0 , 0 } ;
	uint8_t 				buf[17];
	
	for (int i = 0 ; i < SPEED_ENCODER_NUM ; i++) 
	{	
		speed[i] += g_speed_encoder[ i ].speed ;
		counter[i] += g_speed_encoder[ i ].counter ;
	}
	
	if ( index < 9 ) 
	{
		index++;
	}
	else
	{
		index = 0 ;
		speed[0] = speed[0] / 10 ;
		speed[1] = speed[1] / 10 ;
		sprintf( buf , "L:%d R:%d" , speed[0] , speed[1] );
		OLED_ShowAscii( 0,0, buf , 16 ,0 );
		
		sprintf( buf , "L:%d R:%d" , counter[0] , counter[1] );
		OLED_ShowAscii( 1,0, buf , 16 ,0 );
		
		speed[0] = 0 ;
		speed[1] = 0 ;		
		counter[0] = 0 ;
		counter[1] = 0 ;
	}
}

