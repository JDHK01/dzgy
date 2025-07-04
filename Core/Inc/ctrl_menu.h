#ifndef __CTRL_MENU_H
#define __CTRL_MENU_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
	 
#define MENU_TEXT_LEN		16
#define MENU_PAGE_SIZE	2	 
	 
typedef struct {
	uint8_t text[MENU_TEXT_LEN+1]  ;  
	void (*func_callback)(void); 
	uint8_t level ;
	uint8_t parent ;
	uint8_t index ;
}menu_define_t ;

typedef struct {
	uint8_t text[MENU_TEXT_LEN+1]  ;  
	struct menu_item_t * parent ; 
	struct menu_item_t * childe ; 
}menu_item_t ;

typedef struct {
	uint8_t cur_level ;
	uint8_t cur_index ;
}menu_t ;

void Ctrl_Menu_Item( void );
void Ctrl_Menu_Init( void );
void Ctrl_Menu_Show( void );
void Ctrl_Menu_Process( void );
void Ctrl_Menu_Return( void );


void Music_Menu_Off( void );
void Music_Menu_On( void );
void UltrasonicWave_Menu( void );

extern __IO uint8_t g_btn0_flag ;
extern __IO uint8_t g_btn1_flag ;

#ifdef __cplusplus
}
#endif
#endif /*__CTRL_MENU_H */
