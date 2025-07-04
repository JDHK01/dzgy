#ifndef __SPEED_ENCODER_H
#define __SPEED_ENCODER_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "tim.h"
#include "stm32_hal_legacy.h"

#define SPEED_ENCODER_NUM		2
#define SPEED_FREQ			  	(100)
#define PULSE_PER_CIRCLE		(13*4*48)
#define WHEEL_SIZE		  		(65*31416)
//(WHEEL_SIZE*SPEED_FREQ)/PULSE_PER_CIRCLE = 81812
#define SPEED_COEFF 				81812

typedef enum
{
  ENCODER_STATE_IDLE             = 0x00U,
  ENCODER_STATE_CAL_SPEED   		 ,
} Encoder_StateTypeDef;

typedef struct {
	TIM_HandleTypeDef * timer ;
	int16_t counter ;
	int32_t speed ;	
} Speed_Encoder_t ;	

void Speed_Encoder_Init( void );
void Speed_Encoder_Update( void );
void Speed_Calculate( void );
void Speed_Process( void );
void Speed_Show(void);

extern __IO Encoder_StateTypeDef g_speed_encoder_state ;
extern __IO Speed_Encoder_t g_speed_encoder[SPEED_ENCODER_NUM] ;

#endif
