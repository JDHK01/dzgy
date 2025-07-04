#include "user_command.h"
#include "uart_dma.h"

uint8_t g_cmd_buf[CMD_BUF_LEN] = {0};

car_ctrl_cmd_t g_CarCtrlCmd[] = {
	{"ID" , &UserCtrlCmdCallback, 0 , 0}
};

uint8_t UserCmdParamParse(uint8_t *buf, void *ptr)
{
	uint8_t symbol = 0;
	uint8_t len;
	int int_temp0 = 0;
	float float_result = 0.0;
	uint8_t point = 0;
	float weight = 1.0;
	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
	if ( cmd_ptr->param == NULL ) return 0;
	
	len = strlen((const char *)buf);
	switch(cmd_ptr->type)
	{
		case CONV_INT:
		case CONV_U8:
		case CONV_U16:
		{			
			for(int i = 0; i < len; i++)
			{
				if(buf[i] == '-')
				{
					if(i != 0)
					{
						return 0;
					}
					symbol = 1;
				}
				else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
				{
					int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
				}
				else
				{
					return 0;
				}
			}
			switch(cmd_ptr->type)
			{
				case CONV_INT:
					*(int *)cmd_ptr->param = (symbol == 0) ? int_temp0 : (-1 * int_temp0);
					break;
				
				case CONV_U8:
					if(symbol == 1 || int_temp0 > 255)
					{
						return 0;
					}
					*(uint8_t *)cmd_ptr->param = int_temp0;
					break;
				
				case CONV_U16:
					if(symbol == 1 || int_temp0 > 65535)
					{
						return 0;
					}
					*(uint16_t *)cmd_ptr->param = int_temp0;
					break;
					
				default:
					return 0;
					break;
			}			
		}
		break;
		
		case CONV_FLOAT:
		{
			for(int i = 0; i < len; i++)
			{
				if(buf[i] == '-')
				{
					if(i != 0)
					{
						return 0;
					}
					symbol = 1;
				}
				else if(buf[i] == '.')
				{
					if(point != 0)
					{
						return 0;
					}
					point = 1;
					float_result = (float)int_temp0 ;
					int_temp0 = 0 ;
					weight  = 1.0 ;					
				}
				else if( ( buf[i] >= '0' ) && ( buf[i] <= '9' ) )
				{
						int_temp0 = int_temp0 * 10 + ( buf[i] - '0' );
					  weight *= 10 ;
				}
				else
				{
						return 0;
				}
			}
			float_result = (point == 0) ? (float)int_temp0 : float_result + (float)int_temp0 / weight;
			*(float *)cmd_ptr->param = (symbol == 0) ? float_result : (-1.0 * float_result);
		}
		break;
		
		case CONV_NUM_CHAR:
			if(len > 5)
			{
				return 0;
			}
			for(int i = 0; i < len; i++)
			{
				if( ( buf[i] < '0' ) || ( buf[i] > '9' ) )
				{
					return 0;
				}
			}
			memcpy((uint8_t *)cmd_ptr->param, buf, len);
		break;
		
		default:
			return 0;
			break;			
	}
	return 1 ;
}


car_ctrl_cmd_t *CheckUserCmd(uint8_t *buf)
{
	car_ctrl_cmd_t *ptr = NULL;
	int i = 0 ;
	for ( i = 0 ; i < sizeof(g_CarCtrlCmd)/sizeof(car_ctrl_cmd_t) ; i++ )
	{
		if(strncasecmp((const uint8_t *)g_CarCtrlCmd[i].cmd, (const char *)buf, strlen((const char *)g_CarCtrlCmd[i].cmd)) == 0)
		{
			return &g_CarCtrlCmd[i];
		}
	}
	return ptr;
}

uint8_t UserCmdParse(uint8_t * buf, uint32_t len)
{
	uint8_t field_num = 0 ;
	uint8_t cmd_buf[16];
	uint8_t cmd_index = 0;
	car_ctrl_cmd_t *cmd_ptr = NULL;
	
	for(int i = 0; i < len; i++)
	{
		if(buf[i] == '|' || i == len - 1)
		{
			if(cmd_index == 0 && i < len - 1)
			{
				return 0;
			}
			else
			{
				if(buf[i] == '|')
				{
					cmd_buf[cmd_index] = 0;
				}
				else
				{
					cmd_buf[cmd_index++] = buf[i];
					cmd_buf[cmd_index] = 0;
				}
				if(field_num == 0)
				{
					cmd_ptr = CheckUserCmd(cmd_buf);
					if(cmd_ptr == NULL)
					{
						return 0;
					}
					if(cmd_ptr->param == NULL )
					{
						cmd_ptr->func_callback(cmd_buf, cmd_ptr);
						return 1;
					}
					field_num++;
				}
				else if(field_num == 1)
				{
					if(cmd_ptr == NULL)
					{
						return 0;
					}
					if(cmd_ptr->func_callback)
					{
						cmd_ptr->func_callback(cmd_buf, cmd_ptr);
						return 1;
					}
				}
				cmd_index = 0;				
			}
		}
		else
		{
			if(cmd_index < 15)
			{
				cmd_buf[cmd_index++] = buf[i];
			}
			else
			{
				return 0;
			}
		}
	}	
	return 0;
}

uint32_t UartCtrl_RxDataCallback( uint8_t * buf , uint32_t len )
{
	static uint8_t tag_flag = 0;
	static uint8_t index;
	for(int i = 0; i < len; i++)
	{
		if( buf[i] == '@' )
		{
			tag_flag = 1;
			index = 0;
		}
		else if( buf[i] == '&' )
		{
			if(tag_flag == 1 && index > 0)
			{
				g_cmd_buf[index] = 0;
				UserCmdParse(g_cmd_buf, index);
				return 1;
			}
		}
		else
		{
			if(tag_flag == 1)
			{
				if(index < 63)
				{
					g_cmd_buf[index++] = buf[i];
				}
			}
		}
	}
	return 1;
}

void UserCtrlCmdCallback(uint8_t *buf, void *ptr)
{
  uint8_t tmp[16] ;
	
	car_ctrl_cmd_t *cmd_ptr = (car_ctrl_cmd_t *)ptr;
	if (strcasecmp( buf , "ID" ) == 0 )
	{
		printf("%s\r\n" ,g_device_uid);
		return ;
	}
}
