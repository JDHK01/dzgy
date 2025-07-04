#include <string.h>
#include <stdio.h>
#include "usart.h"
#include "uart_dma.h"

// uart1 <==> Debug
// uart3 <==> IO

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

uart_buf_t g_uart_buf[ MAX_UART_USED_NUM ] ;


uint32_t Uart1_RxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len );
uint32_t Uart2_RxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len );
uint32_t Uart3_RxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len );

uint32_t Uart1_TxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len );
uint32_t Uart2_TxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len );
uint32_t Uart3_TxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len );

/*
uart_index, buf_index, huart, hdma_rx, hdma_tx, rx_cb, tx_cb, target_index, target_type

uart_index start with 1 and same as system uart index , uart_index = 0 , uart not used
buf_index start with 0 and is the index of g_uart_buf
target_index is system index, all start with 1. 0 is invalid

g_uart_port_map index = uart_index - 1
*/
const uart_map_t g_uart_port_map[ MAX_UART_PORT_NUM ] = { 
	{1,0,&huart1,&hdma_usart1_rx,&hdma_usart1_tx, &Uart1_RxDataCallback , &Uart1_TxDataCallback , 0 , TARGET_CPU } ,  
	{0,0,0,0,0,0,0,0, TARGET_CPU} ,
	{3,1,&huart3,&hdma_usart3_rx,&hdma_usart3_tx, &Uart3_RxDataCallback , &Uart3_TxDataCallback , 0 , TARGET_CPU } ,
	{0,0,0,0,0,0,0,0, TARGET_CPU} ,
	{0,0,0,0,0,0,0,0, TARGET_CPU} ,
	{0,0,0,0,0,0,0,0, TARGET_CPU} ,
	{0,0,0,0,0,0,0,0, TARGET_CPU} ,
	{0,0,0,0,0,0,0,0, TARGET_CPU} ,
};

///////////////////////////////////////////////////////////////////////
//
// UART initialize
//
///////////////////////////////////////////////////////////////////////
uint32_t Uart_Buf_Init( uint32_t uart_index )
{
	uint32_t buf_index ;
	uint32_t map_index ;
	if ( uart_index == 0 || uart_index > MAX_UART_PORT_NUM ) return 0 ;
	map_index = uart_index - 1;	
	
	if ( map_index >= MAX_UART_PORT_NUM ) return 0 ;
	if ( g_uart_port_map[ map_index ].uart_index == 0 ) return 0 ; // not active 
	buf_index = g_uart_port_map[ map_index ].buf_index ;
	
	HAL_UART_Init( g_uart_port_map[ map_index ].huart ) ;

	g_uart_buf[ buf_index ].rx_buf_head = 0 ;
	g_uart_buf[ buf_index ].rx_buf_tail = 0 ;
	g_uart_buf[ buf_index ].rx_buf_full = 0 ;
	
	g_uart_buf[ buf_index ].tx_buf_head = 0 ;
	g_uart_buf[ buf_index ].tx_buf_tail = 0 ;
	g_uart_buf[ buf_index ].tx_buf_full = 0 ;
	g_uart_buf[ buf_index ].tx_busy = 0 ;
	
	g_uart_buf[ buf_index ].rx_size     = 0 ;
	g_uart_buf[ buf_index ].tx_size     = 0 ;
	
	g_uart_buf[ buf_index ].huart_map   = &g_uart_port_map[map_index] ;
	
	__HAL_UART_ENABLE_IT( g_uart_port_map[ map_index ].huart , UART_IT_IDLE);    //使能空闲终中断
	if ( g_uart_port_map[ map_index ].hdma_rx != NULL )
	{
		HAL_UART_Receive_DMA( g_uart_port_map[ map_index ].huart , 
													( uint8_t *)g_uart_buf[ buf_index ].rx_buf[ 0 ] ,
													UART_RX_BUF_SIZE );
	}
	else
	{
		HAL_UART_Receive_IT( g_uart_port_map[ map_index ].huart , 
													( uint8_t *)g_uart_buf[ buf_index ].rx_buf[ 0 ] ,
													UART_RX_BUF_SIZE );
	}
	return 1 ;
}

uint32_t StartUartByIndex( uint32_t uart_index )
{
	uint32_t map_index ;
	if ( uart_index == 0 || uart_index > MAX_UART_PORT_NUM ) return 0 ;
	map_index = uart_index - 1;	
	if ( g_uart_port_map[map_index].uart_index != uart_index ) return 0 ;

	return Uart_Buf_Init( uart_index ); 
}

uint32_t StopUartByIndex( uint32_t uart_index )
{
	uint32_t map_index ;
	if ( uart_index == 0 || uart_index > MAX_UART_PORT_NUM ) return 0 ;
		map_index = uart_index - 1;
	if ( g_uart_port_map[map_index].uart_index != uart_index ) return 0 ;

	if (g_uart_port_map[ map_index ].hdma_rx != NULL )
	{
	   	HAL_UART_DMAStop( g_uart_port_map[ map_index ].huart );
	}
	else
	{
			HAL_UART_EndRxTransfer_IT( g_uart_port_map[ map_index ].huart );			
	}
 	__HAL_UART_DISABLE_IT( g_uart_port_map[ map_index ].huart , UART_IT_IDLE);    //使能空闲终中断
	HAL_UART_DeInit( g_uart_port_map[ map_index ].huart ) ;
	return 1 ;
}

void StartAllUartDMAReceive( void )
{
	uint32_t uart_index ;

	for ( uart_index = 1 ; uart_index <= MAX_UART_PORT_NUM ; uart_index++ )
	{
		StartUartByIndex( uart_index);
	}
}
	
void StopAllUart( void )
{
	uint32_t uart_index ;
	for ( uart_index = 1 ; uart_index <= MAX_UART_PORT_NUM ; uart_index++ )
	{
		StopUartByIndex( uart_index );
	}
}

///////////////////////////////////////////////////////////////////////
//
// UART system function
//
///////////////////////////////////////////////////////////////////////

void HAL_UART_EndRxTransfer_IT(UART_HandleTypeDef *huart)
{
  /* Disable RXNE, PE and ERR (Frame error, noise error, overrun error) interrupts */
  CLEAR_BIT(huart->Instance->CR1, (USART_CR1_RXNEIE | USART_CR1_PEIE));
  CLEAR_BIT(huart->Instance->CR3, USART_CR3_EIE);

  /* At end of Rx process, restore huart->RxState to Ready */
  huart->RxState = HAL_UART_STATE_READY;
}

void HAL_UART_DMAStopRx(UART_HandleTypeDef *huart)
{
  uint32_t dmarequest = 0x00U;
  /* The Lock is not implemented on this API to allow the user application
     to call the HAL UART API under callbacks HAL_UART_TxCpltCallback() / HAL_UART_RxCpltCallback():
     when calling HAL_DMA_Abort() API the DMA TX/RX Transfer complete interrupt is generated
     and the correspond call back is executed HAL_UART_TxCpltCallback() / HAL_UART_RxCpltCallback()
     */

  /* Stop UART DMA Rx request if ongoing */
  dmarequest = HAL_IS_BIT_SET(huart->Instance->CR3, USART_CR3_DMAR);
  if ((huart->RxState == HAL_UART_STATE_BUSY_RX) && dmarequest)
  {
    CLEAR_BIT(huart->Instance->CR3, USART_CR3_DMAR);

    /* Abort the UART DMA Rx channel */
    if (huart->hdmarx != NULL)
    {
      HAL_DMA_Abort(huart->hdmarx);
    }
    HAL_UART_EndRxTransfer_IT(huart);
  }

}


///////////////////////////////////////////////////////////////////////
//
// UART system interrupt callback
//
///////////////////////////////////////////////////////////////////////

void Uart_RxIDLE_Handler( int32_t uart_index )
{
	int32_t map_index , buf_index;
	int32_t	temp , len ;
	
	map_index = uart_index - 1;
	
	if ( g_uart_port_map[ map_index ].uart_index != uart_index ) return ; // not active
	buf_index = g_uart_port_map[ map_index ].buf_index ;
	
	if( __HAL_UART_GET_FLAG( g_uart_port_map[ map_index ].huart , UART_FLAG_IDLE) != RESET )
	{
		__HAL_UART_CLEAR_IDLEFLAG( g_uart_port_map[ map_index ].huart );
		__HAL_UART_CLEAR_PEFLAG( g_uart_port_map[ map_index ].huart );

//		temp = g_uart_port_map[ map_index ].huart->Instance->SR;  
//		temp = g_uart_port_map[ map_index ].huart->Instance->DR;
		if ( g_uart_port_map[ map_index ].hdma_rx != NULL )
		{
			HAL_UART_DMAStopRx( g_uart_port_map[ map_index ].huart );
			temp  = __HAL_DMA_GET_COUNTER( g_uart_port_map[ map_index ].hdma_rx ) ;
			len = UART_RX_BUF_SIZE - temp ;
			if ( len > 0 )
			{
				g_uart_buf[ buf_index ].rx_buf_size[ g_uart_buf[ buf_index ].rx_buf_tail ] =  len;   
				g_uart_buf[ buf_index ].rx_size += len; 
				if ( g_uart_buf[ buf_index ].rx_buf_tail == ( MAX_UART_BUF_NUM - 1 ) )
					g_uart_buf[ buf_index ].rx_buf_tail = 0 ;
				else
					g_uart_buf[ buf_index ].rx_buf_tail ++ ;
			}

			HAL_UART_Receive_DMA( g_uart_port_map[ map_index ].huart , 
														( uint8_t *)g_uart_buf[ buf_index ].rx_buf[ g_uart_buf[ buf_index ].rx_buf_tail ] ,
														UART_RX_BUF_SIZE );
		}
		else
		{
      HAL_UART_EndRxTransfer_IT( g_uart_port_map[ map_index ].huart );			
			temp = g_uart_port_map[ map_index ].huart->RxXferCount ;
			len = UART_RX_BUF_SIZE - temp ;
			if ( len > 0 )
			{
				g_uart_buf[ buf_index ].rx_buf_size[ g_uart_buf[ buf_index ].rx_buf_tail ] =  len ;
				if ( g_uart_buf[ buf_index ].rx_buf_tail == ( MAX_UART_BUF_NUM - 1 ) )
					g_uart_buf[ buf_index ].rx_buf_tail = 0 ;
				else
					g_uart_buf[ buf_index ].rx_buf_tail ++ ;
			}

			HAL_UART_Receive_IT( g_uart_port_map[ map_index ].huart , 
														( uint8_t *)g_uart_buf[ buf_index ].rx_buf[ g_uart_buf[ buf_index ].rx_buf_tail ] ,
														UART_RX_BUF_SIZE );
			
		}
			
		if ( len >0 && g_uart_buf[ buf_index ].rx_buf_tail == g_uart_buf[ buf_index ].rx_buf_head ) 
		{
			g_uart_buf[ buf_index ].rx_buf_full = 1 ;
		}
	}	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	
//	if(huart->Instance == USART1) {	
//		i = 0 ;
//	} else if(huart->Instance == USART2) {	
//		i = 1 ;
//	} else if(huart->Instance == USART3) {	
//		i = 2 ;
//	} else if(huart->Instance == UART4) {	
//		i = 3 ;
//	} else if(huart->Instance == UART5) {	
//		i = 4 ;
//	} else  
//	{	
//		return ;
//	}
//  sprintf( ( char *)err , "uart %d : error code:%X\r\n",i+1 ,huart->ErrorCode ); 
//  SysLog( err ) ;			
//  SendUartDataToUART( g_host_monitor_uart_map ,  err );

}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t uart_index , map_index, buf_index ;

	if(huart->Instance == USART1) {	
		uart_index = 1 ;
	} else if(huart->Instance == USART2) {	
		uart_index = 2 ;
	} else if(huart->Instance == USART3) {	
		uart_index = 3 ;
	} else
	{	
		return ;
	}
	map_index = uart_index -1;
	
	if ( g_uart_port_map[ map_index ].uart_index != uart_index ) return ;
	buf_index = g_uart_port_map[ map_index ].buf_index ;

	g_uart_buf[ buf_index ].rx_buf_size[ g_uart_buf[ buf_index ].rx_buf_tail ] =  UART_RX_BUF_SIZE ;                           
	g_uart_buf[ buf_index ].rx_size += UART_RX_BUF_SIZE; 
	if ( g_uart_buf[ buf_index ].rx_buf_tail == ( MAX_UART_BUF_NUM - 1 ) )
		g_uart_buf[ buf_index ].rx_buf_tail = 0 ;
	else
		g_uart_buf[ buf_index ].rx_buf_tail ++ ;
	
	if ( g_uart_port_map[ map_index ].hdma_rx != NULL )
	{
		HAL_UART_Receive_DMA( g_uart_port_map[ map_index ].huart , 
													( uint8_t *)g_uart_buf[ buf_index ].rx_buf[ g_uart_buf[ buf_index ].rx_buf_tail ] ,
													UART_RX_BUF_SIZE );
	}
	else
	{
		HAL_UART_Receive_IT( g_uart_port_map[ map_index ].huart , 
													( uint8_t *)g_uart_buf[ buf_index ].rx_buf[ g_uart_buf[ buf_index ].rx_buf_tail ] ,
													UART_RX_BUF_SIZE );
	}

	if ( g_uart_buf[ buf_index ].rx_buf_tail == g_uart_buf[ buf_index ].rx_buf_head ) 
	{
		g_uart_buf[ buf_index ].rx_buf_full = 1 ;
	}
	
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uint32_t uart_index , map_index, buf_index ;

	if(huart->Instance == USART1) {	
		uart_index = 1 ;
	} else if(huart->Instance == USART2) {	
		uart_index = 2 ;
	} else if(huart->Instance == USART3) {	
		uart_index = 3 ;
	} else 
	{	
		return ;
	}

	map_index = uart_index -1;	
	if ( g_uart_port_map[ map_index ].uart_index != uart_index ) return ;
	buf_index = g_uart_port_map[ map_index ].buf_index ;

	if ( g_uart_buf[ buf_index ].tx_buf_head == ( MAX_UART_BUF_NUM - 1 ) )
		g_uart_buf[ buf_index ].tx_buf_head = 0 ;
	else
		g_uart_buf[ buf_index ].tx_buf_head ++ ;
	g_uart_buf[ buf_index ].tx_buf_full = 0 ;
  g_uart_buf[ buf_index ].tx_busy = 0 ;
}

///////////////////////////////////////////////////////////////////////
//
// UART user function
//
///////////////////////////////////////////////////////////////////////
UART_HandleTypeDef* Uart_Index_To_Handel(uint32_t uart_index)
{
	uint32_t map_index ;
	if ( uart_index > MAX_UART_PORT_NUM || uart_index == 0 ) return NULL ;
	map_index = uart_index -1;	
	if ( g_uart_port_map[ map_index ].uart_index != uart_index ) return NULL ;
	return g_uart_port_map[ map_index ].huart ;
}

uint32_t UartTxData( uint32_t uart_index , uint8_t * buf , uint32_t len )
{
  uint32_t map_index, buf_index ;

	if ( uart_index > MAX_UART_PORT_NUM || uart_index == 0 ) return 0 ;
	map_index = uart_index -1;	
	if ( g_uart_port_map[ map_index ].uart_index != uart_index ) return 0 ;
	buf_index = g_uart_port_map[ map_index ].buf_index ;

	g_uart_buf[ buf_index ].tx_size += len;
	HAL_UART_Transmit(g_uart_port_map[ map_index ].huart , buf , len , HAL_MAX_DELAY );
	return 1 ;
}


uint32_t UartTxDataQueue( uint32_t uart_index , uint8_t * buf , uint32_t len )
{
  uint32_t map_index, buf_index ;

	if ( len > UART_TX_BUF_SIZE ) return 0 ;
	if ( uart_index > MAX_UART_PORT_NUM || uart_index == 0 ) return 0 ;
	map_index = uart_index -1;	
	if ( g_uart_port_map[ map_index ].uart_index != uart_index ) return 0 ;
	buf_index = g_uart_port_map[ map_index ].buf_index ;

	if ( g_uart_buf[ buf_index ].tx_buf_full == 1 ) return 0 ;

	memcpy( g_uart_buf[ buf_index ].tx_buf[ g_uart_buf[ buf_index ].tx_buf_tail ] , buf , len );	
	g_uart_buf[ buf_index ].tx_buf_size[ g_uart_buf[ buf_index ].tx_buf_tail ] =  len ;              
	
	if ( g_uart_buf[ buf_index ].tx_buf_tail == ( MAX_UART_BUF_NUM - 1 ) )
		g_uart_buf[ buf_index ].tx_buf_tail = 0 ;
	else
		g_uart_buf[ buf_index ].tx_buf_tail ++ ;
	
	if ( g_uart_buf[ buf_index ].tx_buf_tail == g_uart_buf[ buf_index ].tx_buf_head ) 
	{
		g_uart_buf[ buf_index ].tx_buf_full = 1 ;
	}
	return 1 ;
}

///////////////////////////////////////////////////////////////////////
//
// UART user callback function
//
///////////////////////////////////////////////////////////////////////
uint32_t UartCtrl_RxDataCallback( uint8_t * buf , uint32_t len );

uint32_t TxRxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len )
{
	uint32_t ret = 0 ;
	if ( target_type == TARGET_CPU )
	{
		ret = UartCtrl_RxDataCallback( buf , len );	
		
	}
	
	return ret ;
}

uint32_t Uart1_RxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len )
{
	return TxRxDataCallback(target_type ,target_index, buf ,len );
}

uint32_t Uart2_RxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len )
{
	return TxRxDataCallback(target_type ,target_index, buf ,len );
}

uint32_t Uart3_RxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len )
{
	return TxRxDataCallback(target_type ,target_index, buf ,len );
}


uint32_t Uart1_TxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len )
{
	uint32_t ret = 0 ;
	return ret ;
}

uint32_t Uart2_TxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len )
{
	uint32_t ret = 0 ;
	return ret ;
}

uint32_t Uart3_TxDataCallback( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len )
{
	uint32_t ret = 0 ;
	return ret ;
}
///////////////////////////////////////////////////////////////////////
//
// UART user process function
//
///////////////////////////////////////////////////////////////////////

void CheckUartRxData( void )
{
  uint8_t * rx_buf ;
	int32_t len ;
	const uart_map_t *	huart_map ;
	int32_t 			buf_index ;
	
 	for ( buf_index = 0 ; buf_index < MAX_UART_USED_NUM ; buf_index++ )
	{
//		if (  g_uart_buf[ buf_index ].rx_buf_full == 1 )
//		{
//			print_debug("Uart Rx overflow!\n");
//		}
		if ( g_uart_buf[ buf_index ].rx_buf_full == 1 || 
				g_uart_buf[ buf_index ].rx_buf_head != g_uart_buf[ buf_index ].rx_buf_tail  )
		{				
			huart_map = g_uart_buf[ buf_index ].huart_map ;
			len = g_uart_buf[ buf_index ].rx_buf_size[ g_uart_buf[ buf_index ].rx_buf_head ] ;			
			rx_buf = g_uart_buf[ buf_index ].rx_buf[ g_uart_buf[ buf_index ].rx_buf_head ] ;
			if ( huart_map->RxCallback( huart_map->target_type , huart_map->target_index ,rx_buf , len ) == 0 ) return ;
			if ( g_uart_buf[ buf_index ].rx_buf_head == ( MAX_UART_BUF_NUM - 1 ) )
				g_uart_buf[ buf_index ].rx_buf_head = 0 ;
			else
				g_uart_buf[ buf_index ].rx_buf_head ++ ;
			g_uart_buf[ buf_index ].rx_buf_full = 0 ;			
		}			
	}
}

void CheckUartTxData( void )
{
  uint8_t * 		tx_buf ;
	int32_t 			len ;
	const uart_map_t *	huart_map ;
	int32_t 			buf_index ;
	
 	for ( buf_index = 0 ; buf_index < MAX_UART_USED_NUM ; buf_index++ )
	{
		if ( g_uart_buf[ buf_index ].tx_busy == 1 ) continue ;
//		if ( g_uart_buf[ buf_index ].tx_buf_full == 1 )
//		{
//			print_debug("Uart Tx overflow!\n");
//		}
		if ( g_uart_buf[ buf_index ].tx_buf_full == 1 || 
				g_uart_buf[ buf_index ].tx_buf_head != g_uart_buf[ buf_index ].tx_buf_tail  )
		{				
			huart_map = g_uart_buf[ buf_index ].huart_map ;
			len = g_uart_buf[ buf_index ].tx_buf_size[ g_uart_buf[ buf_index ].tx_buf_head ] ;
			tx_buf = g_uart_buf[ buf_index ].tx_buf[ g_uart_buf[ buf_index ].tx_buf_head ] ;
			g_uart_buf[ buf_index ].tx_size += len;
			HAL_UART_Transmit_DMA( huart_map->huart , tx_buf , len );
			g_uart_buf[ buf_index ].tx_busy = 1 ;			
		}			
	}

}

