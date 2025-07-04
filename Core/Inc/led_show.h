
#ifndef LED_SHOW_H
#define LED_SHOW_H

#ifdef __cplusplus
extern "C" {
#endif
	
typedef enum
{
  LEDSHOW_STOP        		= 0x00U,    
  LEDSHOW_IDLE        		,    
  LEDSHOW_START     			,    
} LEDSHOW_State_TypeDef;
	
void UserLED_Init( void );
void UserLEDShowProcess( void );
void UserLED_Show_Menu( void );

extern __IO LEDSHOW_State_TypeDef  g_LED_Show_state  ;
extern __IO uint8_t  g_LED_Update_flag ;
extern __IO uint8_t  g_LED_key_down  ;

#ifdef __cplusplus
}
#endif

#endif /* LED_SHOW_H */
