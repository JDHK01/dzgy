/*
#include "main.h"
#include "tim.h"
#include "music.h"

const uint8_t  g_music_bpm = 72 ;
const uint16_t g_music_whole_note = 60*1000 / g_music_bpm * 10 ;

uint8_t g_music_mode = 0; // 0  one time , 1 - repeat

// Freq base on 12Mhz clock
const uint32_t g_music_freq[22] = {
	0,
	45802, 40816, 36364, 34384, 30612, 27273, 24292,
	22945, 20443, 18209, 17241, 15306, 13636, 12170,
	11461, 10213,  9105,  8590,  7653,  6818,  6101 
};

const uint8_t g_music_tone[MUSIC_LENGTH] = {
	5, 
	8,8,8,9,10,12, 10,10,0,10,9, 8,8,6,8,6,8, 5,5,0,0,5, 
 	8,8,8,9,10,12, 13,13,12,12,10,9, 8,8,6,5,8, 8,8,0,0,5, 
	8,8,8,9,10,12, 10,10,0,10,9, 8,8,6,8,6,8, 5,0,0,0,5, 
	8,8,8,9,10,12, 13,13,12,12,10,9, 8,8,6,5,5,9, 8,8,0,0,
	12,12,9,10,10,9, 8,9,10,12,12, 13,15,13,12,10,9,8, 6,10,9,10,9,
	8,9,10,12,12, 8,13,13,15,10,12,10, 9,9,8,9,10,13,12, 12,12,0,0,
	10,12,15,16,17, 8,10,15,13,12,12, 13,12,10,9,8, 6,8,12,10,9,9,
	10,12,15,16,17, 8,10,15,16,15,15, 0,15,13,12,10,9,8, 6,8,12,10,9,9,8,
	8,8,0,0,255	
};
const uint8_t g_music_note[MUSIC_LENGTH] = {
  2, 
	1,2,2,1,2,2, 1,1,1,2,2, 1,2,2,2,2,1, 1,1,1,2,2, 
	1,2,2,1,2,2, 1,2,2,1,2,2, 1,2,2,1,1, 1,1,1,2,2, 
	1,2,2,1,2,2, 1,1,1,2,2, 1,2,2,2,2,1, 1,1,1,2,2, 
	1,1,2,2,2,2, 1,2,2,1,2,2, 1,2,2,2,2,1, 1,1,1,1, 
	1,2,2,1,2,2, 1,2,2,1,1, 1,2,2,2,2,2,2, 1,1,1,2,2,
	1,2,2,1,1, 2,2,2,2,1,2,2, 1,2,2,2,2,2,2, 1,1,1,1,
	1,2,2,1,1, 2,2,2,2,1,1, 1,2,2,1,1, 2,2,2,2,1,1,
	1,2,2,1,1, 2,2,2,2,1,1, 2,2,2,2,2,2,1 ,2,2,2,2,1,2,2,
	1,1,1,1, 0	
};

__IO Music_State_TypeDef  g_music_state = MUSIC_STOP ;

uint8_t  g_music_running = 0 ;

uint16_t g_music_timer_set = 0 ;
uint16_t g_music_index = 0 ;

TIM_HandleTypeDef *g_music_beep = &htim1;
TIM_HandleTypeDef *g_music_time = &htim7;
*/
void Music_Menu_On( void )
{
	//g_music_state = MUSIC_IDLE ;
	//HAL_TIM_Base_Start_IT( g_music_time );
}

void Music_Menu_Off( void )
{
	//g_music_state = MUSIC_STOP ;
	//HAL_TIM_Base_Stop_IT( g_music_time );
}
/*
void MusicProcess( void )
{
	uint32_t counter ;
	
	if ( g_music_state == MUSIC_STOP ) 
	{
		if ( g_music_running == 0 ) return ;
		g_music_running = 0 ;
		HAL_TIM_PWM_Stop( g_music_beep ,TIM_CHANNEL_1 );
		HAL_TIM_Base_Stop_IT( g_music_time );
		return ;
	}
	
	if ( g_music_state == MUSIC_START )
	{
	
		g_music_timer_set = g_music_whole_note / g_music_note[g_music_index] ;
		
		HAL_TIM_PWM_Stop( g_music_beep ,TIM_CHANNEL_1 );
		__HAL_TIM_SET_PRESCALER( g_music_beep , 5);
		__HAL_TIM_SET_AUTORELOAD( g_music_beep , g_music_freq[g_music_tone[g_music_index]]-1 );
		__HAL_TIM_SET_COUNTER( g_music_beep , 0 );
		__HAL_TIM_SET_COMPARE( g_music_beep ,TIM_CHANNEL_1,g_music_freq[g_music_tone[g_music_index]]/2 );
		HAL_TIM_PWM_Start( g_music_beep ,TIM_CHANNEL_1 );
		
		HAL_TIM_Base_Stop_IT( g_music_time );
		__HAL_TIM_SET_PRESCALER(g_music_time , 7199 );
		__HAL_TIM_SET_AUTORELOAD( g_music_time , g_music_timer_set );
		__HAL_TIM_SET_COUNTER( g_music_time , 0 );
		HAL_TIM_Base_Start_IT( g_music_time );
		
		g_music_index++;
		if ( g_music_tone[g_music_index] == 255 ) 
		{
			g_music_index = 0 ;
			if ( g_music_mode == 0 )
			{
				g_music_state = MUSIC_STOP ;
				g_music_running = 0 ;
				HAL_TIM_PWM_Stop( g_music_beep ,TIM_CHANNEL_1 );
		    HAL_TIM_Base_Stop_IT( g_music_time );
				return ;
			}
		}
		g_music_state = MUSIC_IDLE ;
		g_music_running = 1 ;
	}
}
*/
/**
  ******************************************************************************
  * @file    music.c
  * @brief   Music player implementation for STM32F103RCT6
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "music.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define MUSIC_TIM_HANDLE    htim2
#define MUSIC_TIM_CHANNEL   TIM_CHANNEL_1
#define MUSIC_TIM_CLK       72000000  // 72MHz
#define MUSIC_TIM_PRESCALER 71         // 72MHz / 72 = 1MHz

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
MUSIC_State_TypeDef g_music_state = MUSIC_STOP;
uint8_t g_music_volume = MUSIC_DEFAULT_VOLUME;

static const MUSIC_Note_TypeDef *g_current_melody = NULL;
static uint16_t g_melody_length = 0;
static uint16_t g_note_index = 0;
static uint32_t g_note_start_time = 0;
static TIM_HandleTypeDef *g_music_timer = &MUSIC_TIM_HANDLE;

// ????
static const MUSIC_Note_TypeDef startup_melody[] = {
    {NOTE_C5, 200},
    {NOTE_E5, 200},
    {NOTE_G5, 200},
    {NOTE_C6, 400},
    {NOTE_REST, 100},
    {NOTE_G5, 200},
    {NOTE_C6, 600}
};

// ???? - ???
static const MUSIC_Note_TypeDef twinkle_star[] = {
    {NOTE_C5, 500}, {NOTE_C5, 500}, {NOTE_G5, 500}, {NOTE_G5, 500},
    {NOTE_A5, 500}, {NOTE_A5, 500}, {NOTE_G5, 1000},
    {NOTE_F5, 500}, {NOTE_F5, 500}, {NOTE_E5, 500}, {NOTE_E5, 500},
    {NOTE_D5, 500}, {NOTE_D5, 500}, {NOTE_C5, 1000},
    {NOTE_REST, 500}
};

/* Private function prototypes -----------------------------------------------*/
static void Music_PWM_Init(void);
static void Music_PWM_Start(void);
static void Music_PWM_Stop(void);
static void Music_PWM_SetFrequency(uint16_t frequency);
static void Music_PWM_SetDutyCycle(uint8_t duty);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  ?????PWM?? (PA5 - TIM2_CH1)
  * @retval None
  */
static void Music_PWM_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};
    
    /* GPIO?? - PA5 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    
    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    /* TIM2???tim.c????,?????PWM?? */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    
    if (HAL_TIM_PWM_ConfigChannel(g_music_timer, &sConfigOC, MUSIC_TIM_CHANNEL) != HAL_OK)
    {
        Error_Handler();
    }
    
    /* ?????????????? */
    __HAL_TIM_SET_PRESCALER(g_music_timer, MUSIC_TIM_PRESCALER);
    __HAL_TIM_SET_AUTORELOAD(g_music_timer, 999);  // ??1kHz
}

/**
  * @brief  ??PWM??
  * @retval None
  */
static void Music_PWM_Start(void)
{
    HAL_TIM_PWM_Start(g_music_timer, MUSIC_TIM_CHANNEL);
}

/**
  * @brief  ??PWM??
  * @retval None
  */
static void Music_PWM_Stop(void)
{
    HAL_TIM_PWM_Stop(g_music_timer, MUSIC_TIM_CHANNEL);
}

/**
  * @brief  ??PWM??
  * @param  frequency: ?? (Hz)
  * @retval None
  */
static void Music_PWM_SetFrequency(uint16_t frequency)
{
    uint32_t period;
    
    if (frequency == 0)
    {
        Music_PWM_Stop();
        return;
    }
    
    // ??????? (ARR)
    // PWM?? = TIM_CLK / ((PSC + 1) * (ARR + 1))
    // ARR = (TIM_CLK / ((PSC + 1) * frequency)) - 1
    period = (1000000 / frequency) - 1;  // 1MHz / frequency
    
    if (period > 65535) period = 65535;
    
    __HAL_TIM_SET_AUTORELOAD(g_music_timer, period);
    __HAL_TIM_SET_COMPARE(g_music_timer, MUSIC_TIM_CHANNEL, period * g_music_volume / 100 / 2);
}

/**
  * @brief  ??PWM???
  * @param  duty: ??? (0-100)
  * @retval None
  */
static void Music_PWM_SetDutyCycle(uint8_t duty)
{
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(g_music_timer);
    uint32_t pulse = (period * duty) / 100;
    
    __HAL_TIM_SET_COMPARE(g_music_timer, MUSIC_TIM_CHANNEL, pulse);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  ????????
  * @retval None
  */
void Music_Init(void)
{
    Music_PWM_Init();
    g_music_state = MUSIC_STOP;
    g_music_volume = MUSIC_DEFAULT_VOLUME;
    
    // ?????????
    Music_PlayStartupSound();
}

/**
  * @brief  ????
  * @param  melody: ??????
  * @param  length: ????
  * @retval None
  */
void Music_Play(const MUSIC_Note_TypeDef *melody, uint16_t length)
{
    if (melody == NULL || length == 0) return;
    
    g_current_melody = melody;
    g_melody_length = length;
    g_note_index = 0;
    g_note_start_time = HAL_GetTick();
    g_music_state = MUSIC_PLAYING;
    
    // ???????
    Music_SetFrequency(melody[0].frequency);
    if (melody[0].frequency != NOTE_REST)
    {
        Music_PWM_Start();
    }
}

/**
  * @brief  ????
  * @retval None
  */
void Music_Stop(void)
{
    g_music_state = MUSIC_STOP;
    g_current_melody = NULL;
    g_melody_length = 0;
    g_note_index = 0;
    Music_PWM_Stop();
}

/**
  * @brief  ????
  * @retval None
  */
void Music_Pause(void)
{
    if (g_music_state == MUSIC_PLAYING)
    {
        g_music_state = MUSIC_PAUSE;
        Music_PWM_Stop();
    }
}

/**
  * @brief  ????
  * @retval None
  */
void Music_Resume(void)
{
    if (g_music_state == MUSIC_PAUSE)
    {
        g_music_state = MUSIC_PLAYING;
        g_note_start_time = HAL_GetTick();
        
        if (g_current_melody[g_note_index].frequency != NOTE_REST)
        {
            Music_PWM_Start();
        }
    }
}

/**
  * @brief  ????
  * @param  volume: ?? (0-100)
  * @retval None
  */
void Music_SetVolume(uint8_t volume)
{
    if (volume > MUSIC_VOLUME_MAX) volume = MUSIC_VOLUME_MAX;
    
    g_music_volume = volume;
    
    // ??????,?????
    if (g_music_state == MUSIC_PLAYING)
    {
        Music_PWM_SetDutyCycle(volume / 2);  // 50%????????
    }
}

/**
  * @brief  ????
  * @param  frequency: ?? (Hz)
  * @retval None
  */
void Music_SetFrequency(uint16_t frequency)
{
    Music_PWM_SetFrequency(frequency);
}

/**
  * @brief  ???????? (?????????)
  * @retval None
  */
void Music_Process(void)
{
    uint32_t current_time;
    
    if (g_music_state != MUSIC_PLAYING || g_current_melody == NULL) return;
    
    current_time = HAL_GetTick();
    
    // ????????????
    if ((current_time - g_note_start_time) >= g_current_melody[g_note_index].duration)
    {
        g_note_index++;
        
        // ???????????
        if (g_note_index >= g_melody_length)
        {
            Music_Stop();
            return;
        }
        
        // ???????
        g_note_start_time = current_time;
        Music_SetFrequency(g_current_melody[g_note_index].frequency);
        
        if (g_current_melody[g_note_index].frequency == NOTE_REST)
        {
            Music_PWM_Stop();
        }
        else
        {
            Music_PWM_Start();
        }
    }
}

/**
  * @brief  ??????
  * @retval None
  */
void Music_PlayStartupSound(void)
{
    Music_Play(startup_melody, sizeof(startup_melody) / sizeof(MUSIC_Note_TypeDef));
}