#ifndef __USR_COMMAND_H
#define __USR_COMMAND_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

#pragma pack(4)

typedef struct
{
	uint8_t cmd[32];
	void (*func_callback)(uint8_t *buf, void *param);
	void *param;
	uint8_t type;
}car_ctrl_cmd_t;

#define CMD_MAP_NUM 10
#define CMD_BUF_LEN	64

#define CONV_INT			0
#define CONV_FLOAT		1
#define CONV_NUM_CHAR	2
#define CONV_U8				3
#define CONV_U16			4

void UserCtrlCmdCallback(uint8_t *buf, void *ptr);
uint32_t UartCtrl_RxDataCallback( uint8_t * buf , uint32_t len );

#endif
