#ifndef __MOTOR_DRIVE_H
#define __MOTOR_DRIVE_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"
#include "tim.h"
#include "stm32_hal_legacy.h"

#define DRIVE_MOTO_NUM		2
#define STEER_MOTO_NUM		2

#define MOTO_PWM_FREQ			50
#define MOTO_PWM_STEP			10000

#define STEER_MOTO_LEFT		(1*MOTO_PWM_STEP*MOTO_PWM_FREQ)/1000
#define STEER_MOTO_RIGHT	(2*MOTO_PWM_STEP*MOTO_PWM_FREQ)/1000
#define STEER_MOTO_CENTER	(STEER_MOTO_RIGHT+STEER_MOTO_LEFT)/2
#define STEER_MOTO_STEP		(STEER_MOTO_RIGHT-STEER_MOTO_LEFT)/180

typedef struct {
	TIM_TypeDef       *Instance;
	TIM_HandleTypeDef * timer ;
	uint32_t				 forword ;
	uint32_t				 backword ;
} Drive_Moto_t ;	

void Moto_Ctrl_Init(void);
void Drive_Moto_Ctrl(int32_t pos , int16_t speed);
void Steer_Moto_Ctrl(int32_t pos , int16_t degree );
void Stop_All_Moto(void);
void TestSteerMotoProcess(void);
extern __IO uint8_t g_servo_moto_test ;
#endif
