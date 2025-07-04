
#ifndef MUSIC_H
#define MUSIC_H

#ifdef __cplusplus
extern "C" {
#endif

#define MUSIC_LENGTH 200
	
typedef enum
{
 // MUSIC_STOP        		= 0x00U,    
  MUSIC_IDLE        		,    
  MUSIC_START     			,    
} Music_State_TypeDef;

//void MusicProcess( void );

//extern __IO Music_State_TypeDef  g_music_state  ;



/**
  ******************************************************************************
  * @file    music.h
  * @brief   Music player header file for STM32F103RCT6
  ******************************************************************************
  */



/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MUSIC_STOP = 0,
    MUSIC_PLAYING,
    MUSIC_PAUSE
} MUSIC_State_TypeDef;

typedef struct {
    uint16_t frequency;  // ?? (Hz)
    uint16_t duration;   // ???? (ms)
} MUSIC_Note_TypeDef;

/* Exported constants --------------------------------------------------------*/
// ?????? (Hz)
#define NOTE_C4     262
#define NOTE_CS4    277
#define NOTE_D4     294
#define NOTE_DS4    311
#define NOTE_E4     330
#define NOTE_F4     349
#define NOTE_FS4    370
#define NOTE_G4     392
#define NOTE_GS4    415
#define NOTE_A4     440
#define NOTE_AS4    466
#define NOTE_B4     494

#define NOTE_C5     523
#define NOTE_CS5    554
#define NOTE_D5     587
#define NOTE_DS5    622
#define NOTE_E5     659
#define NOTE_F5     698
#define NOTE_FS5    740
#define NOTE_G5     784
#define NOTE_GS5    831
#define NOTE_A5     880
#define NOTE_AS5    932
#define NOTE_B5     988

#define NOTE_C6     1047
#define NOTE_REST   0       // ???

/* Exported macro ------------------------------------------------------------*/
#define MUSIC_VOLUME_MAX    100
#define MUSIC_VOLUME_MIN    0
#define MUSIC_DEFAULT_VOLUME 50

/* Exported variables --------------------------------------------------------*/
extern MUSIC_State_TypeDef g_music_state;
extern uint8_t g_music_volume;

/* Exported functions prototypes ---------------------------------------------*/
void Music_Init(void);
void Music_Play(const MUSIC_Note_TypeDef *melody, uint16_t length);
void Music_Stop(void);
void Music_Pause(void);
void Music_Resume(void);
void Music_SetVolume(uint8_t volume);
void Music_SetFrequency(uint16_t frequency);
void Music_Process(void);
void Music_PlayStartupSound(void);



#ifdef __cplusplus
}
#endif

#endif /* MUSIC_H */
