#ifndef __IR_TRACK_H
#define __IR_TRACK_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"

#define IR_CHANNEL_NUM 	5
#define ADC_THRESHOLD		3000  // 1/3 full scale
typedef struct
{
  uint32_t 	track_adc[IR_CHANNEL_NUM]  ;   
	uint16_t 	track_compare_white[IR_CHANNEL_NUM]  ;  
	uint16_t 	track_compare_black[IR_CHANNEL_NUM]  ;  
	uint16_t 	track_compare_gate[IR_CHANNEL_NUM]  ;  
	int     	track_error[32];
	uint16_t 	total_adc_value;
	uint8_t 	adc_value;
	uint8_t 	full_black;
	uint8_t 	full_white;
}track_status_t ;

void IR_Track_Init( void );
void IR_Track_ADC_Start( void );
void IR_Track_NormalCal(void);
void IR_Track_Power_On( void );
void IR_Track_Power_Off( void );
void IR_Track_Channel_Set ( uint8_t ch );

void IR_Track_Process( void );
void IR_Track_Timer( void );

void IR_Track_Menu_Calib_White( void );
void IR_Track_Menu_Calib_Black( void );
void IR_Track_ShowGate( void );
void IR_Track_ShowValue( void );
void IR_Track_Menu_Start( void );
void IR_Track_Menu_Stop( void );

extern track_status_t g_TrackStatus;

extern __IO uint8_t g_ir_power_on_flag ;
extern __IO uint8_t g_ir_track_ok_flag ;

#ifdef __cplusplus
}
#endif
#endif /*__IR_TRACK_H */
