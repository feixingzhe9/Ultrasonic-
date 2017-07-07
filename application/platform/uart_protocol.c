/* 
*  Author: Adam Huang
*  Date:2016/12/13
*/
#include "protocol.h"
#include "app_platform.h"
#include <stdlib.h>
//#include "serial_leds.h"
#include "voltage_detect.h"
#include "stringUtils.h"
#include "upgrade_flash.h"
#include "tps611xx_bl.h"

#define protocol_log(M, ...) custom_log("Protocol", M, ##__VA_ARGS__)
#define protocol_log_trace() custom_log_trace("Protocol")


static  uint8_t    rxBuf[UART_RX_BUFFER_LENGTH];
static  uint8_t    txBuf[UART_TX_BUFFER_LENGTH];

recBuf_t                rxInfoInRam;

serial_t                serialDataInRam = {
    .rx_buf = {
        .pBuffer = rxBuf,
        .offset = rxBuf + 3, //header,length,ctype
        .bufferSize = UART_RX_BUFFER_LENGTH,
    },
    .tx_buf = {
        .pBuffer = txBuf,
        .offset = txBuf + 2, //header,length
        .bufferSize = UART_TX_BUFFER_LENGTH,
    },
};

uart_serial_t           uartSerialInRam;

serial_t * const serial = &serialDataInRam;


OSStatus Protocol_Init( void )//( serial_t *serial )
{
  OSStatus err = kNoErr;
  
  
  if( serial->isSerialInitialized )
  {    
//    if( STATE_POWER_ON != (boardStatus->sysStatus & STATE_RUN_BITS) )
    {
      goto exit;
    }
    MicoUartFinalize( COMM_UART );
    serial->isSerialInitialized = 0;

  }
  
  serial->rx_info = &rxInfoInRam;
  require_action( serial->rx_info , exit, err = kGeneralErr );
  serial->uart_serial = &uartSerialInRam;
  require_action( serial->uart_serial , exit, err = kGeneralErr );

  serial->rx_info->pData = 0;
  serial->rx_info->rxBuffer = rxBuf;
  serial->rx_info->rxCount = 0;


  err = uart_ser_open( serial->uart_serial, 115200 );  
  require_action( err == kNoErr, exit,  protocol_log("open uart err") ); 
  
  require_action( serial->rx_buf.pBuffer , exit, err = kGeneralErr );  
  memset( (uint8_t *)serial->rx_buf.pBuffer, 0x00, serial->rx_buf.bufferSize );
#ifdef COMM_DMA_USE_INT
  startDmaRecive( serial->uart_serial->uartHandle, (uint8_t *)serial->rx_buf.pBuffer );
#else
  startDmaRecive( COMM_UART, (uint8_t *)serial->rx_buf.pBuffer );
#endif
  serial->isStartDmaReceive = 1;  
  serial->isSerialInitialized = 1;
  protocol_log( "protocol initialize success" );
  
exit:
  return err;
}

        



#if 0
static OSStatus uart_frame_send( serial_t *serial, const uint8_t *pData, uint32_t size )
{
  OSStatus err = kNoErr;
  uint8_t  checkSum;
  ram_buff_t *tx_buff = NULL;
  
  require_action( serial , exit, err = kGeneralErr );
  require_action( serial->uart_serial , exit, err = kGeneralErr );  
  
  tx_buff = &serial->tx_buf;
  require_action( tx_buff->pBuffer , exit, err = kGeneralErr );  
  require_action( size <= tx_buff->bufferSize - 4, exit, err = kParamErr );
  
  *(uint8_t *)tx_buff->pBuffer = FRAME_HEADER;
  *(uint8_t *)(tx_buff->pBuffer + 1) = size + 4;//header,length,footer,checksum
  
  checkSum = 0;
  tx_buff->pData = (uint8_t *)tx_buff->pBuffer;
  if( pData != tx_buff->offset )
  {
    memmove( (void *)tx_buff->offset, (void *)pData, size );
  }
  for( uint8_t i = 0; i < size + 2; i++ )
  {
    checkSum += *tx_buff->pData ++;
  }
  *tx_buff->pData ++ = checkSum;  
  *tx_buff->pData ++ = FRAME_FOOTER;
  
  err = uart_ser_write( serial->uart_serial, \
    (uint8_t *)tx_buff->pBuffer, tx_buff->pData - tx_buff->pBuffer );
  
  memset( (uint8_t *)tx_buff->pBuffer, 0x00, tx_buff->bufferSize );
  
exit:
  return err;
}
#endif



static int is_receive_right_frame( void  );
static int is_receive_right_frame( void  )
{ 
  if( serial->rx_info->rxCount > 0 && *serial->rx_buf.pBuffer == FRAME_HEADER )
  {
    if( *(serial->rx_buf.pBuffer + 1) != 0)
    {
      if( *(serial->rx_buf.pBuffer + *(serial->rx_buf.pBuffer + 1) - 1 ) == FRAME_FOOTER )
      {
        return 0;
      }
    }
  }
  return -1;
}

void protocol_period( void )
{
  uint8_t checksum;
  uint8_t detectType;
  
  if( !serial->isSerialInitialized )
  {
    Protocol_Init();
  }
#ifdef COMM_DMA_USE_INT
  serial->rx_info->rxCount = receviedDmaDataLength( serial->uart_serial->uartHandle );
#else
  serial->rx_info->rxCount = receviedDmaDataLength( COMM_UART );
#endif
  
  if( !is_receive_right_frame() )//serial->rx_info->rxCount > 0 && serial->rx_buf.buffer)//&& rx_buf.rxBuffer[rxBuf[1]-1] == FRAME_FOOTER )//( rx_buf.rxCount > 0 && rx_buf.rxBuffer[rx_buf.rxCount-1] == 0xA5)
  {        
#ifdef PROTOCOL_DEBUG
    if( OPEN_SHOW_LOG == serial->rx_info->showLogFlag )
    {
      char *debugString;
      debugString = DataToHexStringWithSpaces(\
        (const uint8_t *)serial->rx_info->rxBuffer, serial->rx_info->rxCount );
      protocol_log( "rxBuf: %s", debugString );
      free( debugString );
    }
#endif
    serial->rx_info->pData = 0;
    if( serial->rx_info->startTime == 0 )
    {
      protocol_log( "start communicating" );
    }
    serial->rx_info->startTime = os_get_time();
  }
  else
  {
    if( ( serial->rx_info->startTime != 0 ) && \
      ((os_get_time() - serial->rx_info->startTime) >= \
        COMMUNICATION_TIMEOUT_TIME_MS/SYSTICK_PERIOD) )
    {
      serial->rx_info->startTime = 0;
      protocol_log( "communicate timeout" );
   
#ifdef COMM_DMA_USE_INT
      stopDmaRecive( serial->uart_serial->uartHandle );
#else
      stopDmaRecive( COMM_UART );
#endif
      serial->isStartDmaReceive = 0;
//      serial->rx_info->startTime = os_get_time();
      Protocol_Init();
    }
    goto exit;
  }
#ifdef COMM_DMA_USE_INT
  stopDmaRecive( serial->uart_serial->uartHandle );
#else
  stopDmaRecive( COMM_UART );
#endif
  serial->isStartDmaReceive = 0;
  serial->rx_buf.receiveStartTime = 0;
  
  checksum = 0;
  
  serial->rx_buf.pData = (uint8_t *)serial->rx_buf.pBuffer;
  
  checksum += *serial->rx_buf.pData;
  
  serial->rx_info->bufferHeader.detectLength = *++serial->rx_buf.pData; 
  if( *(serial->rx_buf.pData + serial->rx_info->bufferHeader.detectLength - 2) != FRAME_FOOTER )
  {
    protocol_log("not known cmd");
    goto exit;
  }
  checksum += *serial->rx_buf.pData;
  
  serial->rx_info->bufferHeader.detectType = *++serial->rx_buf.pData;
  checksum += *serial->rx_buf.pData;
    
  for( uint8_t i = 0; i < (serial->rx_info->bufferHeader.detectLength - 5); i++ )
  {
    checksum += *++serial->rx_buf.pData;
  }
  require_action_quiet( checksum == *++serial->rx_buf.pData, exit, \
    protocol_log("check sum not match") );

  detectType = serial->rx_info->bufferHeader.detectType;
  
  switch( detectType )
  {
  default:
    break;
  }
#if 1
  if( serial->rx_info->rxCount )
  {
    memset( (uint8_t *)serial->rx_buf.pBuffer, 0x00, serial->rx_buf.bufferSize );  
  }
#ifdef COMM_DMA_USE_INT
  startDmaRecive( serial->uart_serial->uartHandle, (uint8_t *)serial->rx_buf.pBuffer );
#else
  startDmaRecive( COMM_UART, (uint8_t *)serial->rx_buf.pBuffer );
#endif 
  serial->isStartDmaReceive = 1;
#endif
exit:
  if( serial->rx_info->rxCount == 0xFF )
  {
    memset( (uint8_t *)serial->rx_buf.pBuffer, 0x00, serial->rx_buf.bufferSize );
#ifdef COMM_DMA_USE_INT
    startDmaRecive( serial->uart_serial->uartHandle, (uint8_t *)serial->rx_buf.pBuffer );
    stopDmaRecive( serial->uart_serial->uartHandle );
#else
    startDmaRecive( COMM_UART, (uint8_t *)serial->rx_buf.pBuffer );
    stopDmaRecive( COMM_UART );
#endif
    serial->isStartDmaReceive = 0;     
  }
  if( !serial->isStartDmaReceive )
  {
    serial->rx_info->rxCount = 0;
    memset( (uint8_t *)serial->rx_buf.pBuffer, 0x00, serial->rx_buf.bufferSize );   
#ifdef COMM_DMA_USE_INT
    startDmaRecive( serial->uart_serial->uartHandle, (uint8_t *)serial->rx_buf.pBuffer );
#else
    startDmaRecive( COMM_UART, (uint8_t *)serial->rx_buf.pBuffer );
#endif
    serial->isStartDmaReceive = 1;
  }
  return;
}