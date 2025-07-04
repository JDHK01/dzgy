#include "main.h"
#include "spi.h"
#include "tim.h"
#include "led_show.h"

#define LED_SIZE			24		

SPI_HandleTypeDef * g_led_show_spi = &hspi1;

__IO LEDSHOW_State_TypeDef  g_LED_Show_state  = LEDSHOW_STOP ;

__IO uint8_t  g_LED_Update_flag  = 0 ;
__IO uint8_t  g_LED_key_down  = 0 ;
TIM_HandleTypeDef *g_ledshow_time = &htim7;

const uint8_t g_ShowData[3][ 48 ]  = {
	{ 
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x66,0x52,0x52,0x52,0x4A,0x6C,0x00,
		0x02,0x06,0x4A,0x70,0x4A,0x06,0x02,0x00,
		0x26,0x42,0x42,0x42,0x42,0x64,0x38,0x00,		
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } ,
	{ 
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x38,0x78,0x66,0x47,0x67,0x6A,0x62,0x42,
		0x46,0x4E,0x4C,0x44,0x54,0x7C,0x38,0x30,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } ,
	{ 
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x40,0x40,0x7E,0x44,0x44,0x00,
	  0x00,0x00,0x40,0x40,0x7E,0x44,0x44,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
		0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 } 	
	};


uint8_t g_LED_Show_RAM[LED_SIZE] = { 
	0x01 , 0x01 , 0x01 , 0x01 ,
	0x08 , 0x08 , 0x08 , 0x08 ,
	0x80 , 0x80 , 0x80 , 0x80 ,
	0x01 , 0x01 , 0x01 , 0x01 ,
	0x08 , 0x08 , 0x08 , 0x08 ,
	0x80 , 0x80 , 0x80 , 0x80 
};

uint8_t g_LED_CAT_RAM[6] = { 
	0xFB , 0xF7 , 0xEF , 
	0xDF , 0xBF , 0x7F  
};

uint8_t g_LED_MAP[2][6] = { 
	{0,10,7,4,1,2},
	{3,6,9,11,8,5}
};

void LEDSHOW( uint8_t mode ,  uint8_t cat_addr )
{
	//static uint8_t cat_addr = 0 ;	// 0~5 6 

	uint8_t ram_addr ;
	uint8_t i ,j , tmp ;
	uint8_t ram_data[12] ;
	uint8_t buf[3] ;

	buf[2] = g_LED_CAT_RAM[cat_addr];
	if (mode == 0 )
	{
		ram_addr = cat_addr * 4 ;		
		for ( i = 0 ; i < 4 ; i++) 
		{	
			tmp = g_LED_Show_RAM[ram_addr + i] ;
			for (j = 0 ; j < 3 ; j++ )
			{
				ram_data[i*3+j ] =  tmp & 0x07 ;
				tmp >>= 3 ;
			}
		}

		for (i = 0 ; i < 2 ; i++ )
		{
			buf[ i ] = 0 ;
			for ( j = 0 ; j < 6 ; j++ )
			{
				buf[ i ] = buf[ i ] >> 1 ;
				if ( ram_data[g_LED_MAP[i][j]] ) 
				{
					buf[ i ] = buf[ i ] | 0x80 ;
				}
			}
		}
	}
	else
	{
		buf[0] = 0 ;
		buf[1] = 0 ;		
	}
	
	HAL_GPIO_WritePin( IR_LOCK_GPIO_Port , IR_LOCK_Pin , GPIO_PIN_RESET );
	HAL_SPI_Transmit(g_led_show_spi, (uint8_t *)&buf[0], 3, 20);
	HAL_GPIO_WritePin( IR_LOCK_GPIO_Port , IR_LOCK_Pin , GPIO_PIN_SET );

//	if ( cat_addr == 5 ) cat_addr = 0 ;
//	else cat_addr++;
}

void LED_Clear( void )
{
	uint8_t i ;
	for( i =0 ; i < LED_SIZE ; i++)
		g_LED_Show_RAM[i] = 0 ;
}

void LED_Loop0( void )
{	
	static uint8_t i = 0 ;
	if ( i == LED_SIZE -1 ) {
		g_LED_Show_RAM[0] = g_LED_Show_RAM[i] ;
		g_LED_Show_RAM[i] = 0 ;
		i = 0;
	}
	else
	{
		g_LED_Show_RAM[i+1] = g_LED_Show_RAM[i] ;
		g_LED_Show_RAM[i] = 0 ;
		i++;
	}
}

void LED_Loop1( uint8_t index )
{	
	uint8_t tmp , i ;
	tmp = g_ShowData[0][index] ;
	for( i = 0 ; i < 8 ;i++)
	{
		if ( tmp & 0x01 ) g_LED_Show_RAM[i] = 0x80  ;
		else g_LED_Show_RAM[i] = 0x00  ;
		tmp = tmp >> 1 ;
	}
	tmp = g_ShowData[1][index] ;
	for( i = 8 ; i < 16 ;i++)
	{
		if ( tmp & 0x01 ) g_LED_Show_RAM[i] = 0x01  ;
		else g_LED_Show_RAM[i] = 0x00  ;
		tmp = tmp >> 1 ;
	}
	tmp = g_ShowData[2][index] ;
	for( i = 16 ; i < 24 ;i++)
	{
		if ( tmp & 0x01 ) g_LED_Show_RAM[i] = 0x09  ;
		else g_LED_Show_RAM[i] = 0x00  ;
		tmp = tmp >> 1 ;
	}
}

void UserLED_Init( void )
{	
	LED_Clear();
	//g_LED_Show_RAM[0] = 0x81 ;
}

void UserLED_Show_Menu( void )
{
	if ( g_LED_Show_state == LEDSHOW_STOP )
	{
		g_LED_Show_state = LEDSHOW_IDLE ;		
		HAL_TIM_Base_Stop_IT(g_ledshow_time);		
		__HAL_TIM_SET_PRESCALER(g_ledshow_time , 3599 );
		__HAL_TIM_SET_AUTORELOAD( g_ledshow_time , 4 );
		__HAL_TIM_SET_COUNTER( g_ledshow_time , 0 );
		HAL_TIM_Base_Start_IT(g_ledshow_time);
	}
	else
	{
		g_LED_Show_state = LEDSHOW_STOP ;
		HAL_TIM_Base_Stop_IT(g_ledshow_time);		
	}		
}

void UserLEDShowProcess( void )
{
	static uint8_t col = 0 ;
	static uint8_t cat = 0 ;
	
	if ( g_LED_key_down == 1 ) 
	{
		g_LED_key_down = 0 ;
		col = 0 ; 
		cat = 0 ;
	}

	if ( g_LED_Show_state == LEDSHOW_START ) 
	{
		g_LED_Show_state = LEDSHOW_IDLE ;			
		if ( col < 40 )
		{
			if (cat == 5  ) 
			{
				cat = 0 ;
				LED_Loop1(col);
				col++;
			}
			else cat++;
			LEDSHOW(0 , cat  );
		}
		else
		{
			LEDSHOW(1 , 0 );
		}
	}
}
