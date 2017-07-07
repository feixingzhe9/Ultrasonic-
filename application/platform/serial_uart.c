/* 
*  Author: Adam Huang
*  Date:2016/6/16
*/
#include "serial_uart.h"
#include "platform.h"
#include "stm32f1xx_powerboard.h"
#include "RingBufferUtils.h"
#include "stringUtils.h"
#include "protocol.h"
#include "Debug.h"
#include "mico.h"

#define serial_log(M, ...) custom_log("Serial", M, ##__VA_ARGS__)
#define serial_log_trace() custom_log_trace("Serial")

extern const platform_uart_t            platform_uart_peripherals[];
extern platform_uart_driver_t           platform_uart_drivers[];

#define COMM_BUFFER_SIZE   255

static const platform_uart_config_t comm_uart_config =
{
  .baud_rate    = STDIO_UART_BAUDRATE,
  .data_width   = DATA_WIDTH_8BIT,
  .parity       = NO_PARITY,
  .stop_bits    = STOP_BITS_1,
  .flow_control = FLOW_CONTROL_DISABLED,
  .flags        = 0,
};

#ifdef COMM_DMA_USE_INT
static volatile ring_buffer_t           comm_rx_buffer;
static volatile uint8_t                 comm_rx_data[COMM_BUFFER_SIZE];
#else
static void clear_err_bit(UART_HandleTypeDef *huart);
#endif

int uart_ser_open( uart_serial_t *uart_serial, uint32_t baudrate )
{
  int result = 0;
#ifdef COMM_DMA_USE_INT
  (void)uart_serial;
  (void)baudrate;
  ring_buffer_init  ( (ring_buffer_t*)&comm_rx_buffer, (uint8_t *)&comm_rx_buffer, COMM_BUFFER_SIZE );
  MicoUartInitialize( COMM_UART, &comm_uart_config, (ring_buffer_t*)&comm_rx_buffer );
#else
  MicoUartInitialize( COMM_UART, &comm_uart_config, NULL );
  uart_serial->uartHandle = platform_uart_drivers[COMM_UART].uart_handle;
  uart_serial->UARTx = platform_uart_drivers[COMM_UART].peripheral->port;
  CLEAR_BIT( platform_uart_drivers[COMM_UART].uart_handle->Instance->CR1, UART_MODE_RX );
#endif
  return result;
}

int uart_ser_write( uart_serial_t *uart_serial, const void *buffer, int length)
{
    int result = 0;
#ifdef COMM_DMA_USE_INT    
    while (HAL_UART_GetState(uart_serial->uartHandle) != HAL_UART_STATE_READY)
    {
      uart_serial->uartHandle->State = HAL_UART_STATE_READY;
    }
    if (HAL_UART_Transmit(uart_serial->uartHandle, (uint8_t *)buffer, length,1) != HAL_OK)
    {
        result = -1;
    }
#else
    result = MicoUartSend( COMM_UART, (uint8_t *)buffer, length );
#endif
#ifdef PROTOCOL_DEBUG
    if( OPEN_SHOW_LOG == serial->rx_info->showLogFlag )
    {
      char *debugString;
      debugString = DataToHexStringWithSpaces((const uint8_t *)buffer,length);
      serial_log("txBuf: %s", debugString);
      free(debugString);
    }
#endif
    return result;
}

int uart_ser_read( uart_serial_t *uart_serial, void *buffer, int bufsize)
{
    int result = 0;
#ifndef COMM_DMA_USE_INT      
    while (HAL_UART_GetState(uart_serial->uartHandle) != HAL_UART_STATE_READY)
    {
      uart_serial->uartHandle->State = HAL_UART_STATE_READY;
    }
    if ( HAL_UART_Receive(uart_serial->uartHandle, (uint8_t *)buffer, bufsize,1) != HAL_OK )
    {
        result = -1;
    }
#else
    result = MicoUartRecv( COMM_UART, (uint8_t *)buffer, bufsize, 0 );
#endif
    return result;
}

#ifdef COMM_DMA_USE_INT
void startDmaRecive( UART_HandleTypeDef *uartHandle, uint8_t *rxbuf)
{
    uint32_t *tmp=0;
    __HAL_DMA_DISABLE(uartHandle->hdmarx);
    uartHandle->hdmarx->Instance->CNDTR = UART_RX_BUFFER_LENGTH;
    uartHandle->hdmarx->Instance->CPAR = (uint32_t)&uartHandle->Instance->DR;
    tmp = (uint32_t*)&rxbuf;
    uartHandle->hdmarx->Instance->CMAR = *(uint32_t*)tmp;
    __HAL_DMA_ENABLE(uartHandle->hdmarx);
    SET_BIT(uartHandle->Instance->CR3, USART_CR3_DMAR);
}

void stopDmaRecive( UART_HandleTypeDef *uartHandle )
{
   __HAL_DMA_DISABLE(uartHandle->hdmarx);
}

uint32_t receviedDmaDataLength( UART_HandleTypeDef *uartHandle )
{
  return (uint32_t)(UART_RX_BUFFER_LENGTH - uartHandle->hdmarx->Instance->CNDTR);
}

#else

OSStatus startDmaRecive( mico_uart_t uart, uint8_t *rxbuf )
{
    OSStatus err = kNoErr;
    uint32_t *tmp=0;
    DMA_HandleTypeDef*         dma_handle;
    
    if ( uart >= MICO_UART_NONE )
      return kUnsupportedErr;

    require_action_quiet( platform_uart_drivers[uart].rx_dma_handle, exit, err = kParamErr );
    require_action_quiet( platform_uart_drivers[uart].uart_handle, exit, err = kParamErr );
    
    dma_handle = platform_uart_drivers[uart].rx_dma_handle;
//    __HAL_DMA_DISABLE_IT( dma_handle, DMA_IT_TC | DMA_IT_HT | DMA_IT_TE );
    __HAL_DMA_DISABLE( dma_handle ); 
    dma_handle->Instance->CNDTR = UART_RX_BUFFER_LENGTH;
    dma_handle->Instance->CPAR = (uint32_t)&platform_uart_drivers[uart].uart_handle->Instance->DR;
    tmp = (uint32_t*)&rxbuf;
    dma_handle->Instance->CMAR = *(uint32_t *)tmp;
    __HAL_DMA_ENABLE( dma_handle );
    clear_err_bit( platform_uart_drivers[uart].uart_handle );
    SET_BIT( platform_uart_drivers[uart].uart_handle->Instance->CR3, USART_CR3_DMAR );
    SET_BIT( platform_uart_drivers[uart].uart_handle->Instance->CR1, UART_MODE_RX );
    

exit:
    return err;
}

OSStatus stopDmaRecive( mico_uart_t uart )
{
    OSStatus err = kNoErr;
    DMA_HandleTypeDef*         dma_handle;

    clear_err_bit( platform_uart_drivers[uart].uart_handle );
    require_action_quiet( platform_uart_drivers[uart].rx_dma_handle, exit, err = kParamErr );
    dma_handle = platform_uart_drivers[uart].rx_dma_handle;
    
    CLEAR_BIT( platform_uart_drivers[uart].uart_handle->Instance->CR1, UART_MODE_RX );
    __HAL_DMA_DISABLE( dma_handle );
    
exit:
    return err;
}

uint32_t receviedDmaDataLength( mico_uart_t uart )
{
    if( platform_uart_drivers[uart].rx_dma_handle == NULL )
      return 0;
    
    return (uint32_t)(UART_RX_BUFFER_LENGTH - platform_uart_drivers[uart].rx_dma_handle->Instance->CNDTR);
}

static void clear_err_bit(UART_HandleTypeDef *huart)
{
  uint32_t tmp_flag = 0, tmp_it_source = 0;

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);  
  /* UART parity error interrupt occurred ------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
    __HAL_UART_CLEAR_PEFLAG(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_PE;
  }
  
  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
  /* UART frame error interrupt occurred -------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
    __HAL_UART_CLEAR_FEFLAG(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_FE;
  }
  
  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
  /* UART noise error interrupt occurred -------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
    __HAL_UART_CLEAR_NEFLAG(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_NE;
  }
  
  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
  /* UART Over-Run interrupt occurred ----------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
    __HAL_UART_CLEAR_OREFLAG(huart);
    
    huart->ErrorCode |= HAL_UART_ERROR_ORE;
  }
#if 0    
  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  { 
    UART_Receive_IT(huart);
  }

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
  /* UART in mode Transmitter ------------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
    UART_Transmit_IT(huart);
  }

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TC);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC);
  /* UART in mode Transmitter end --------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
    UART_EndTransmit_IT(huart);
  } 
#endif
  if(huart->ErrorCode != HAL_UART_ERROR_NONE)
  {
    /* Set the UART state ready to be able to start again the process */
    huart->State = HAL_UART_STATE_READY;
    
    HAL_UART_ErrorCallback(huart);
  }  
}

#endif
/*********************END OF FILE**************/
