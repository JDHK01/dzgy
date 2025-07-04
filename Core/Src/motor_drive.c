#include "motor_drive.h"

__IO uint8_t g_servo_moto_test = 0 ;
	
Drive_Moto_t g_drive_moto[DRIVE_MOTO_NUM] = 
{
//	{ TIM3 , &htim3 , TIM_CHANNEL_4 , TIM_CHANNEL_3 },	// LEFT_FRONT
//	{ TIM8 , &htim8 , TIM_CHANNEL_4 , TIM_CHANNEL_3 }, // RIGHT_FRONT
	{ TIM2 , &htim2 , TIM_CHANNEL_4 , TIM_CHANNEL_3  }, // LEFT_REAR    
		 //TIM2 GPIO Configuration PB10 --> TIM2_CH3  PB11 --> TIM2_CH4    
	{ TIM4 , &htim4 , TIM_CHANNEL_4 , TIM_CHANNEL_3  }, // RIGHT_REAR
		 //TIM4 GPIO Configuration PB8 --> TIM4_CH3  PB9 --> TIM4_CH4    
};

Drive_Moto_t g_steer_moto[STEER_MOTO_NUM] = 
{
	{ TIM4 , &htim4 , TIM_CHANNEL_1 , TIM_CHANNEL_1 }, // RIGHT_FRONT
	//TIM4 GPIO Configuration PB6 --> TIM4_CH1    
	{ TIM2 , &htim2 , TIM_CHANNEL_1 , TIM_CHANNEL_1 }, // LEFT_FRONT
	//TIM2 GPIO Configuration PA15 --> TIM2_CH1    
};

void Drive_Moto_Init( uint8_t index )
{

	TIM_HandleTypeDef* timHandle ;
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	if (index >= DRIVE_MOTO_NUM ) return ;
	
	timHandle = g_drive_moto[ index ].timer;
	
  timHandle->Instance = g_drive_moto[ index ].Instance ;
  timHandle->Init.Prescaler = 72000000/(MOTO_PWM_FREQ*MOTO_PWM_STEP) -1 ; // 400KHz (72MHz / 180 )
  timHandle->Init.CounterMode = TIM_COUNTERMODE_UP;
  timHandle->Init.Period = MOTO_PWM_STEP -1;		// 100Hz ( 400KHz / 4000)
  timHandle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  timHandle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	
	if ( timHandle->Instance == TIM2 )
	{
    __HAL_RCC_TIM2_CLK_ENABLE();		
	}
	if ( timHandle->Instance == TIM3 )
	{
    __HAL_RCC_TIM3_CLK_ENABLE();		
	}
	else if ( timHandle->Instance == TIM4 )
	{
    __HAL_RCC_TIM4_CLK_ENABLE();		
	}
	else if ( timHandle->Instance == TIM8 )
	{
    __HAL_RCC_TIM8_CLK_ENABLE();		
	}

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(timHandle, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(timHandle) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(timHandle, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(timHandle, &sConfigOC, g_drive_moto[ index ].forword) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(timHandle, &sConfigOC, g_drive_moto[ index ].backword) != HAL_OK)
  {
    Error_Handler();
  }

	if ( timHandle->Instance == TIM2 )
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11 ;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_AFIO_REMAP_TIM2_PARTIAL_1(); //TIM2_CH1/ETR/PA15, TIM2_CH2/PB3, TIM2_CH3/PA2,  TIM2_CH4/PA3
	}
	if ( timHandle->Instance == TIM3 )
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_AFIO_REMAP_TIM3_PARTIAL(); //TIM3_CH1/PB4, TIM3_CH2/PB5, TIM3_CH3/PB0, TIM3_CH4/PB1
	}
	else if ( timHandle->Instance == TIM4 )
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_AFIO_REMAP_TIM4_DISABLE(); //TIM4_CH1/PB6,  TIM4_CH2/PB7,  TIM4_CH3/PB8,  TIM4_CH4/PB9
	}
	else if ( timHandle->Instance == TIM8 )
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		//TIM8_CH1/PC6,  TIM8_CH2/PC7,  TIM8_CH3/PC8,  TIM8_CH4/PC9
	}
}

void Steer_Moto_Init( uint8_t index )
{

	TIM_HandleTypeDef* timHandle ;

  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	if (index >= STEER_MOTO_NUM ) return ;
	
	timHandle = g_steer_moto[ index ].timer;
	
  timHandle->Instance = g_steer_moto[ index ].Instance ;

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(timHandle, &sConfigOC, g_steer_moto[ index ].forword) != HAL_OK)
  {
    Error_Handler();
  }

	if ( timHandle->Instance == TIM2 )
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_15 ;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		__HAL_AFIO_REMAP_TIM2_PARTIAL_1(); //TIM2_CH1/ETR/PA15, TIM2_CH2/PB3, TIM2_CH3/PA2, TIM2_CH4/PA3
	}
	if ( timHandle->Instance == TIM3 )
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_4 ;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_AFIO_REMAP_TIM3_PARTIAL(); //TIM3_CH1/PB4, TIM3_CH2/PB5, TIM3_CH3/PB0, TIM3_CH4/PB1
	}
	else if ( timHandle->Instance == TIM4 )
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		__HAL_AFIO_REMAP_TIM4_DISABLE(); //TIM4_CH1/PB6,  TIM4_CH2/PB7,  TIM4_CH3/PB8,  TIM4_CH4/PB9
	}
	else if ( timHandle->Instance == TIM8 )
	{
		__HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
		//TIM8_CH1/PC6,  TIM8_CH2/PC7,  TIM8_CH3/PC8,  TIM8_CH4/PC9
	}
}

void Moto_Ctrl_Init(void)
{
	uint32_t i ;
	for ( i = 0 ; i < DRIVE_MOTO_NUM ; i++)
	{
		Drive_Moto_Init(i) ;

		__HAL_TIM_SET_COUNTER(g_drive_moto[i].timer, 0);
		HAL_TIM_PWM_Stop(g_drive_moto[i].timer , g_drive_moto[i].forword );
		HAL_TIM_PWM_Stop(g_drive_moto[i].timer , g_drive_moto[i].backword );
	}

	for ( i = 0 ; i < STEER_MOTO_NUM ; i++)
	{
		Steer_Moto_Init(i) ;
		
		__HAL_TIM_SET_COMPARE(g_steer_moto[i].timer, g_steer_moto[i].forword  , STEER_MOTO_CENTER ); // 1.5 ms	
		HAL_TIM_PWM_Start(g_steer_moto[i].timer , g_steer_moto[i].forword );
	}
	
}

 void Drive_Moto_Ctrl(int32_t pos , int16_t speed)
{
	if (pos >= DRIVE_MOTO_NUM ) return ;
	HAL_TIM_PWM_Stop(g_drive_moto[pos].timer , g_drive_moto[pos].forword );
	HAL_TIM_PWM_Stop(g_drive_moto[pos].timer , g_drive_moto[pos].backword );
	if(speed < 0)
	{
		__HAL_TIM_SET_COMPARE(g_drive_moto[pos].timer, g_drive_moto[pos].backword  , -speed);//速度参数小于0，反转电机
		HAL_TIM_PWM_Start(g_drive_moto[pos].timer , g_drive_moto[pos].backword );
	}
	else
	{
		__HAL_TIM_SET_COMPARE(g_drive_moto[pos].timer , g_drive_moto[pos].forword , speed);//速度参数大于0，正转电机
		HAL_TIM_PWM_Start(g_drive_moto[pos].timer , g_drive_moto[pos].forword);
	}
}

void Steer_Moto_Ctrl(int32_t pos , int16_t degree )
{
	uint16_t value ;
	if (pos >= STEER_MOTO_NUM ) return ;
	if ( degree < -90 || degree > 90 ) return ;
	value = STEER_MOTO_LEFT + ( degree + 90 ) * STEER_MOTO_STEP ;
	__HAL_TIM_SET_COMPARE(g_steer_moto[pos].timer, g_steer_moto[pos].forword  , value );// 
}

void Stop_All_Moto(void)
{
	uint32_t i ;
	for ( i = 0 ; i < DRIVE_MOTO_NUM ; i++)
	{
		HAL_TIM_PWM_Stop(g_drive_moto[i].timer,g_drive_moto[i].forword);
		HAL_TIM_PWM_Stop(g_drive_moto[i].timer,g_drive_moto[i].backword);
	}
}

void TestSteerMotoProcess( void )
{
	// 更改
	static int16_t degree = 0 ;
	static int8_t dir = 0 ;
	if ( g_servo_moto_test == 1 )
	{
		Steer_Moto_Ctrl(0 , degree );
		Steer_Moto_Ctrl(1 , degree );
		if ( dir == 0 )
			degree+=10;
		else 
			degree-=10;
		
		if (degree > 80 )
		{
			dir = 1;
			degree = 70 ;
		}
		else if ( degree < -80 )
		{
			dir = 0;
			degree = -70 ;
		}
		
		g_servo_moto_test = 0 ;
		
	}
}


