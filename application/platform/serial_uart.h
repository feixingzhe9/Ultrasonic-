/* 
*  Author: Adam Huang
*  Date:2016/6/16
*/
#ifndef __SERIAL_UART_H
#define __SERIAL_UART_H
#include <stdint.h>
#include "stm32f1xx_hal.h"
#include "Common.h"
#include "platform_peripheral.h"

//#define COMM_DMA_USE_INT

#define USART_COM                           USART3

typedef struct _uart_serial_t {
  UART_HandleTypeDef           *uartHandle;
  USART_TypeDef                *UARTx;
} uart_serial_t;

extern uart_serial_t    uart_serial;

int uart_ser_open( uart_serial_t *serial, uint32_t baudrate);
int uart_ser_write( uart_serial_t *serial, const void *buffer, int length);
int uart_ser_read( uart_serial_t *serial, void *buffer, int bufsize);
OSStatus PlatformUartRecv(uint8_t *inRecvBuf, uint32_t inBufLen, uint32_t inTimeOut);
#ifdef COMM_DMA_USE_INT
void startDmaRecive( UART_HandleTypeDef *uartHandle, uint8_t *rxbuf);
void stopDmaRecive( UART_HandleTypeDef *uartHandle );
uint32_t receviedDmaDataLength( UART_HandleTypeDef *uartHandle );
#else
OSStatus startDmaRecive( mico_uart_t uart, uint8_t *rxbuf );
OSStatus stopDmaRecive( mico_uart_t uart );
uint32_t receviedDmaDataLength( mico_uart_t uart );

#endif
#endif
