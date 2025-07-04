#ifndef __uart_dma_H
#define __uart_dma_H
#ifdef __cplusplus
 extern "C" {
#endif
#include "main.h"
#include "common.h"
	 
#define UART_RX_BUF_SIZE    64
#define UART_TX_BUF_SIZE    64
#define MAX_UART_BUF_NUM    4	 
	 
#define MAX_UART_PORT_NUM   8	 
#define MAX_UART_USED_NUM   2	 
	 
#pragma pack(4)
	 
typedef struct 	
{
	uint32_t  						uart_index ;
	uint32_t  						buf_index  ;	
	UART_HandleTypeDef 	* huart ;
	DMA_HandleTypeDef 	* hdma_rx ;
	DMA_HandleTypeDef 	* hdma_tx ;
	uint32_t (* RxCallback)( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len ); 
	uint32_t (* TxCallback)( Switch_Target_TypeDef target_type , uint32_t target_index , uint8_t * buf , uint32_t len );
	uint32_t  target_index ;
	Switch_Target_TypeDef target_type ;	
}uart_map_t;

typedef struct 	
{
	uint8_t   			rx_buf[ MAX_UART_BUF_NUM ][ UART_RX_BUF_SIZE ] ;
	uint32_t  			rx_buf_size[ MAX_UART_BUF_NUM ] ;
	__IO uint32_t  	rx_buf_head ;
	__IO uint32_t  	rx_buf_tail ;
	__IO uint32_t  	rx_buf_full ;	
	
	uint8_t   			tx_buf[ MAX_UART_BUF_NUM ][ UART_TX_BUF_SIZE ] ;
	uint32_t  			tx_buf_size[ MAX_UART_BUF_NUM ] ;
	__IO uint32_t  	tx_buf_head ;
	__IO uint32_t  	tx_buf_tail ;
	__IO uint32_t  	tx_buf_full ;	
	__IO uint32_t		tx_busy     ;	

	uint32_t  			rx_size ;
	uint32_t  			tx_size ;	
	const uart_map_t *  	huart_map ;
}uart_buf_t;


void Uart_RxIDLE_Handler( int32_t uart_index );
void HAL_UART_EndRxTransfer_IT(UART_HandleTypeDef *huart);

uint32_t Uart_Buf_Init( uint32_t uart_index );
uint32_t StartUartByIndex( uint32_t uart_index );
uint32_t StopUartByIndex( uint32_t uart_index );
UART_HandleTypeDef* Uart_Index_To_Handel(uint32_t uart_index);
void StartAllUartDMAReceive( void );
void StopAllUart( void );

uint32_t UartTxData( uint32_t uart_index , uint8_t * buf , uint32_t len );
uint32_t UartTxDataQueue( uint32_t uart_index , uint8_t * buf , uint32_t len );

void CheckUartRxData( void );
void CheckUartTxData( void );

#ifdef __cplusplus
}
#endif
#endif /*__uart_dma_H */
