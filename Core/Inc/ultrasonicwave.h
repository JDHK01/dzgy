#ifndef __UltrasonicWave_H
#define	__UltrasonicWave_H

#define ULTRAWAVE_NUM 	2

typedef enum
{
  ULTRAWAVE_STATE_IDLE             = 0x00U,
  ULTRAWAVE_STATE_EN               ,
  ULTRAWAVE_STATE_TRIG             ,
  ULTRAWAVE_STATE_TRIG_OK          ,
  ULTRAWAVE_STATE_WAIT_ECHO_START  ,
  ULTRAWAVE_STATE_WAIT_ECHO_STOP   ,
  ULTRAWAVE_STATE_CAL_DISTANCE		 ,
  ULTRAWAVE_STATE_TIMEOUT          ,
  ULTRAWAVE_STATE_SHOW             ,
} UltraWave_StateTypeDef;

typedef struct
{
	GPIO_TypeDef * Trig_Port ;
	uint16_t Trig_Pin ;
	GPIO_TypeDef * Echo_Port ;
	uint16_t Echo_Pin ;
	IRQn_Type  IRQn ;
	uint16_t counter ;
	uint32_t distance ;
} UltraWave_DataTypeDef;

void UltrasonicWave_Init( void ) ;
void UltrasonicWave_Trig_Start( void );
void UltrasonicWave_Trig_Stop( void );
void UltrasonicWave_ShowDistance( void ) ;
void UltrasonicWave_Process( void ) ;
void UltrasonicWave_Echo(uint16_t GPIO_Pin);

extern __IO UltraWave_StateTypeDef g_ultrawave_state ;
extern __IO uint8_t  g_ultrawave_pos ;
extern UltraWave_DataTypeDef g_ultrawave_data[ULTRAWAVE_NUM] ;
#endif /* __UltrasonicWave_H */

