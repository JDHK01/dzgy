
#ifndef LED_H
#define LED_H

#ifdef __cplusplus
extern "C" {
#endif
	
typedef struct {
	GPIO_TypeDef * gpio ;
	uint16_t				 pin ;
} LED_t ;	

void UserLEDProcess( void );

extern __IO uint8_t  g_LED_flag  ;

#ifdef __cplusplus
}
#endif

#endif /* LED_H */
