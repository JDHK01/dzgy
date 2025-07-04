#include "main.h"
#include "led.h"

LED_t g_led[] = 
{
	{ LED_FL_GPIO_Port , LED_FL_Pin },
	{ LED_FR_GPIO_Port , LED_FR_Pin },
	{ LED_BL_GPIO_Port , LED_BL_Pin },
	{ LED_BR_GPIO_Port , LED_BR_Pin }
};

__IO uint8_t g_LED_flag = 0;



void UserLEDCtrl( uint8_t pos  , uint8_t mode )
{
	int i = 0 ; 
	uint8_t led_pos = pos ;
	uint8_t led_mode = mode ;
	
	for ( i = 0 ; i < 4 ; i++ )
	{
		if( led_pos & 0x01 )
		{
			switch ( led_mode& 0x03) 
			{
				case 0 :
					HAL_GPIO_WritePin( g_led[i].gpio , g_led[i].pin , GPIO_PIN_SET );
					break ;
				case 1 :
					HAL_GPIO_WritePin( g_led[i].gpio , g_led[i].pin , GPIO_PIN_RESET );
					break ;
				case 2 :
					HAL_GPIO_TogglePin( g_led[i].gpio , g_led[i].pin );
					break ;					
			}
		}		
		else
		{
				HAL_GPIO_WritePin( g_led[i].gpio , g_led[i].pin , GPIO_PIN_SET );
		}
		led_pos >>= 1 ;
		led_mode >>=2 ;
	}
}

void UserLEDProcess( void )
{
	if ( g_LED_flag == 0 )
		return ;
	g_LED_flag = 0 ;
	UserLEDCtrl( 0x0F , 0xAA );
}
