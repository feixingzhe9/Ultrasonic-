/* 
*  Author: Adam Huang
*  Date:2016/6/4
*/
#include "protocol.h"
#include "app_platform.h"
#include <stdlib.h>
#include "serial_uart.h"
#include "serial_leds.h"
#include "voltage_detect.h"
#include "stringUtils.h"
//#include "nbosDriverFlash.h"
#include "platform_peripheral.h"
#include "upgrade_flash.h"

#define protocol_log(M, ...) custom_log("Protocol", M, ##__VA_ARGS__)
#define protocol_log_trace() custom_log_trace("Protocol")

static  uint8_t    rxBuf[UART_RX_BUFFER_LENGTH];
static  uint8_t    txBuf[UART_TX_BUFFER_LENGTH];

//ring_buffer_t rx_buffer;
recBuf_t                rx_buf;
uart_serial_t           uart_serial;

#ifndef COMM_DMA_USE_INT
UART_HandleTypeDef      ComUartHandle;
#endif

void Protocol_Init( void )
{
  rx_buf.pData = 0;
  rx_buf.rxBuffer = rxBuf;
  rx_buf.rxCount = 0;

#ifndef COMM_DMA_USE_INT
  uart_serial.uartHandle = &ComUartHandle;
  uart_serial.UARTx = USART_COM;
#endif
  uart_ser_open( &uart_serial, 115200 );
#ifndef COMM_DMA_USE_INT
  startDmaRecive( &ComUartHandle, rx_buf.rxBuffer );
#else
  startDmaRecive( COMM_UART, rx_buf.rxBuffer );
#endif
  
}

void serialLedsFrameProcess( void *buf )
{
  uint8_t       ligthMode;
  uint16_t      lightEffect;
  recSerialLedsFrame_t *recSerialLedsFrame;
    
  recSerialLedsFrame = (recSerialLedsFrame_t *)buf;
  ligthMode = recSerialLedsFrame->lightMode;
  lightEffect = (recSerialLedsFrame->lightEffectH << 8) | recSerialLedsFrame->lightEffectL;
  
  if( STATE_IS_POWER_OFF != (boardStatus->sysStatus & STATE_RUN_BITS) )
  {
    setSerialLedsEffect((lightsMode_t)ligthMode, lightEffect);
  }

  ackSerialLedsFrameProcess(recSerialLedsFrame->type);
}

void ackSerialLedsFrameProcess( uint8_t type )
{
  uint8_t  length = sizeof(ackSerialLedsFrame_t);
  uint8_t  checkSum;
  ackSerialLedsFrame_t *ackSerialLedsFrame;
  
  ackSerialLedsFrame = (ackSerialLedsFrame_t *)txBuf;
  
  ackSerialLedsFrame->header = FRAME_HEADER;
  ackSerialLedsFrame->length = length;
  ackSerialLedsFrame->type = type;
  ackSerialLedsFrame->curLightMode = serial_leds->modeType;
  ackSerialLedsFrame->curLightEffectH = (serial_leds->effectType & 0xff00) >> 8;
  ackSerialLedsFrame->curLightEffectL = (serial_leds->effectType & 0x00ff);
  ackSerialLedsFrame->sysStatus = boardStatus->sysStatus;
  ackSerialLedsFrame->vBat = boardStatus->vBatLevel;
  uint32_t states = getEachModuleStates();
 // states = ReadBig32(states);
  memcpy( ackSerialLedsFrame->moduleStatus, &states, 4 );
  memcpy( ackSerialLedsFrame->faultBit, voltageConvertData->faultBitTemp, 4);

  ackSerialLedsFrame->footer = FRAME_FOOTER;
  checkSum = 0;
  for(uint8_t i = 0; i < length-2; i++ )
  {
    checkSum += txBuf[i];
  }
  ackSerialLedsFrame->checksum = checkSum;
  uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(ackSerialLedsFrame_t));
}
  
void recPowerControlFrameProcess( void *buf )
{
  recPowerControlFrame_t *recPowerControlFrame;
  
  recPowerControlFrame = (recPowerControlFrame_t *)buf;
  
  if( recPowerControlFrame->controlBit & CONTROL_SYSTEM_SHUTDOWN )
  {
    PowerOffDevices();
    boardStatus->rebootFlag = REBOOT_YES;
  }
  ackPowerControlFrameProcess(buf);
}

void ackPowerControlFrameProcess( void *buf )
{
   recPowerControlFrame_t *recPowerControlFrame;
   recPowerControlFrame = (recPowerControlFrame_t *)txBuf;
   
   memcpy(recPowerControlFrame, buf, sizeof(recPowerControlFrame_t));
   
   uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(recPowerControlFrame_t));
}

void recVersionInfoFrameProcess( void *buf )
{
  recVersionInfoFrame_t *recVersionInfoFrame;
  recVersionInfoFrame = (recVersionInfoFrame_t *)buf;
  
  if( recVersionInfoFrame->readCommand == READ_FW_VERSION_COMMAND )
  {
    ackVersionInfoFrameProcess();
  }
}

void ackVersionInfoFrameProcess( void )
{
  uint8_t length = sizeof(ackVersionInfoFrame_t);
  uint8_t checkSum;
  int8_t swVersion[11] = SW_VERSION;
  ackVersionInfoFrame_t   *ackVersionInfoFrame;
  ackVersionInfoFrame = (ackVersionInfoFrame_t *)txBuf;
  
  ackVersionInfoFrame->header = FRAME_HEADER;
  ackVersionInfoFrame->type = FRAME_TYPE_VERSION_INFO;
  ackVersionInfoFrame->length = length;
  ackVersionInfoFrame->hw1 = 0x01;
  ackVersionInfoFrame->hw2 = 0x02;
  memcpy(ackVersionInfoFrame->sw,(void const*)swVersion,11);
  ackVersionInfoFrame->footer = FRAME_FOOTER;
  checkSum = 0;
  for(uint8_t i = 0; i < length-2; i++ )
  {
    checkSum += txBuf[i];
  }
  ackVersionInfoFrame->checksum = checkSum;
  uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(ackVersionInfoFrame_t));
}

void recLightsConfiguration( void *buf )
{
  recLightsConfigFrame_t *recLightsConfigFrame;
  recLightsConfigFrame = (recLightsConfigFrame_t *)buf;
  
  (void)recLightsConfigFrame;
  /*not finnished*/
  ackLightsConfiguration();
}

void ackLightsConfiguration( void )
{
  uint8_t length = sizeof(ackLightsConfigFrame_t);
  uint8_t checkSum;

  ackLightsConfigFrame_t   *ackLightsConfigFrame;
  ackLightsConfigFrame = (ackLightsConfigFrame_t *)txBuf;
  
  ackLightsConfigFrame->header = FRAME_HEADER;
  ackLightsConfigFrame->type = FRAME_TYPE_LIGHTS_CONF;
  ackLightsConfigFrame->length = length;
 
  /* not finished */
  ackLightsConfigFrame->footer = FRAME_FOOTER;
  checkSum = 0;
  for(uint8_t i = 0; i < length-2; i++ )
  {
    checkSum += txBuf[i];
  }
  ackLightsConfigFrame->checksum = checkSum;
  uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(ackLightsConfigFrame_t));
}

extern const mico_logic_partition_t mico_partitions[];
void recPrepareUpgrade( void *buf )
{
  uint32_t              firmwareSize;
  recPrepareUpgradeFrame_t *recPrepareUpgradeFrame;
  
  recPrepareUpgradeFrame = (recPrepareUpgradeFrame_t *)buf;
  
  firmwareSize = ReadBig32(recPrepareUpgradeFrame->firmwareSize);
  if( firmwareSize > mico_partitions[MICO_PARTITION_OTA_TEMP].partition_length )
  {
    ackPrepareUpgrade( MCU_STORAGE_NOT_ENOUGH );
    protocol_log( "not enough storage" );
    return;
  }
  protocol_log( "firmwareSize is:%d", firmwareSize );
  if( !upgradePrepareFlash( recPrepareUpgradeFrame->md5, firmwareSize ) )
  {
    ackPrepareUpgrade( MCU_PREPARE_OK );
    protocol_log( "mcu prepare ok" );
  }
  else
  {
    ackPrepareUpgrade( MCU_RETRY_LATER );
    protocol_log( "mcu retry later" );
  }
}

void ackPrepareUpgrade( uint8_t ackState )
{
  uint8_t length = sizeof(ackPrepareUpgradeFrame_t);
  uint8_t checkSum;
  
  ackPrepareUpgradeFrame_t *ackPrepareUpgradeFrame;
  ackPrepareUpgradeFrame = (ackPrepareUpgradeFrame_t *)txBuf;
  
  ackPrepareUpgradeFrame->header = FRAME_HEADER;
  ackPrepareUpgradeFrame->type = FRAME_TYPE_FW_UPGRADE;
  ackPrepareUpgradeFrame->length = length;
  ackPrepareUpgradeFrame->upgradeType = UPGRADE_PREPARE;
  ackPrepareUpgradeFrame->ackState = ackState;//need to check;
  ackPrepareUpgradeFrame->footer = FRAME_FOOTER;
  checkSum = 0;
  for(uint8_t i = 0; i < length-2; i++ )
  {
    checkSum += txBuf[i];
  }
  ackPrepareUpgradeFrame->checksum = checkSum;
  uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(ackPrepareUpgradeFrame_t));
}

void recFirmwarePackageFrame( void *buf )
{
  recFirmwarePackageFrame_t ramRecFirmwarePackageFrame;
  recFirmwarePackageFrame_t *recFirmwarePackageFrame = &ramRecFirmwarePackageFrame;
  
  if( recFirmwarePackageFrame )
  {
    memcpy(recFirmwarePackageFrame, (uint8_t *)buf, 4);//only the first 4 bytes mapping
    recFirmwarePackageFrame->pPackageData = (uint8_t *)buf + 4;
  }
  else
  {
    protocol_log("err: memory not enough");
    ackFirmwarePackageFrame( 0x01 );
    goto exit;
  }
  if( upgradeWriteFlashData((uint32_t *)recFirmwarePackageFrame->pPackageData, recFirmwarePackageFrame->length - 6) == kNoErr )
  {
    ackFirmwarePackageFrame( 0x00 );
  }
  else
  {
    ackFirmwarePackageFrame( 0x01 );
  }
exit:
  return;
}

void ackFirmwarePackageFrame( uint8_t ackState )
{
  uint8_t length = sizeof(ackFirmwarePackageFrame_t);
  uint8_t checkSum;
  
  ackFirmwarePackageFrame_t *ackFirmwarePackageFrame;
  ackFirmwarePackageFrame = (ackFirmwarePackageFrame_t *)txBuf;
  
  ackFirmwarePackageFrame->header = FRAME_HEADER;
  ackFirmwarePackageFrame->type = FRAME_TYPE_FW_UPGRADE;
  ackFirmwarePackageFrame->length = length;
  ackFirmwarePackageFrame->upgradeType = UPGRADE_GOING;
  ackFirmwarePackageFrame->ackState = ackState;// 0x00:OK 0x01:Error
  ackFirmwarePackageFrame->footer = FRAME_FOOTER;
  checkSum = 0;
  for(uint8_t i = 0; i < length-2; i++ )
  {
    checkSum += txBuf[i];
  }
  ackFirmwarePackageFrame->checksum = checkSum;
  uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(ackFirmwarePackageFrame_t));
}

void recUpgradeFinishCheckFrame( void *buf )
{
  recUpgradeFinishCheckFrame_t *recUpgradeFinishCheckFrame;
  
  recUpgradeFinishCheckFrame = (recUpgradeFinishCheckFrame_t *)buf;

  (void)recUpgradeFinishCheckFrame;

  if( !upgradeCheckFlash() )
  {
    ackUpgradeFinishCheckFrame( 0x00 );
    protocol_log("MD5 success,sent right ack");
  }
  else
  {
    ackUpgradeFinishCheckFrame( 0x01 );
    protocol_log("MD5 err,sent err ack");
  }
}

void ackUpgradeFinishCheckFrame( uint8_t ackState )
{
  uint8_t length = sizeof(ackUpgradeFinishCheckFrame_t);
  uint8_t checkSum;
  
  ackUpgradeFinishCheckFrame_t *ackUpgradeFinishCheckFrame;
  ackUpgradeFinishCheckFrame = (ackUpgradeFinishCheckFrame_t *)txBuf;
  
  ackUpgradeFinishCheckFrame->header = FRAME_HEADER;
  ackUpgradeFinishCheckFrame->type = FRAME_TYPE_FW_UPGRADE;
  ackUpgradeFinishCheckFrame->length = length;
  ackUpgradeFinishCheckFrame->upgradeType = UPGRADE_FINISH;
  ackUpgradeFinishCheckFrame->ackState = ackState;// 0x00:OK 0x01:Error
  ackUpgradeFinishCheckFrame->footer = FRAME_FOOTER;
  checkSum = 0;
  for(uint8_t i = 0; i < length-2; i++ )
  {
    checkSum += txBuf[i];
  }
  ackUpgradeFinishCheckFrame->checksum = checkSum;
  uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(ackUpgradeFinishCheckFrame_t));
}

void FirmwareUpgradeProcess( void *buf )
{
  uint8_t updateType;
  
  updateType = *((uint8_t *)buf + 3);
  switch( updateType )
  {
  case UPGRADE_PREPARE:
    recPrepareUpgrade( buf );
    break;
  case UPGRADE_GOING:
    recFirmwarePackageFrame( buf );
    break;
  case UPGRADE_FINISH:
    recUpgradeFinishCheckFrame( buf );
    break;
  default:
    break;
  }
  return;
}

void recTestCurrentCmdFrame( void *buf )
{
  recTestCurrentCmdFrame_t *recTestCurrentCmdFrame;
  recTestCurrentCmdFrame = (recTestCurrentCmdFrame_t *)buf;
  
  switch( recTestCurrentCmdFrame->sendRate )
  {
  case SEND_RATE_SINGLE:
  case SEND_RATE_1HZ:
  case SEND_RATE_2HZ:
  case SEND_RATE_5HZ:
  case SEND_RATE_10HZ:
  case SEND_RATE_50HZ:
  case SEND_RATE_100HZ:
  case SEND_RATE_0_5HZ:
  case SEND_RATE_0_2HZ:
  case SEND_RATE_0_1HZ:
    voltageDebug.uploadRate = recTestCurrentCmdFrame->sendRate;
    break;
  default:
//    ackTestCurrentCmdFrame( 0x01 );
    return;
  }

//  ackTestCurrentCmdFrame( 0x00 );

  voltageDebug.uploadFlagTime = os_get_time();
  switch( recTestCurrentCmdFrame->command )
  {
  case 0x00:
    voltageDebug.isNeedUpload = NO;
    break;
  case 0x01:
    voltageDebug.isNeedUpload = YES;
    break;
  default:
    break;
  }
  return;
}

void ackTestCurrentCmdFrame( uint8_t ackState )
{
  uint8_t length = sizeof(ackTestCurrentCmdFrame_t);
  uint8_t checkSum;
  
  ackTestCurrentCmdFrame_t *ackTestCurrentCmdFrame;
  ackTestCurrentCmdFrame = (ackTestCurrentCmdFrame_t *)txBuf;
  
  ackTestCurrentCmdFrame->header = FRAME_HEADER;
  ackTestCurrentCmdFrame->type = FRAME_TYPE_TEST_CURRENT;
  ackTestCurrentCmdFrame->length = length;
  ackTestCurrentCmdFrame->cmdType = 0x01;
  ackTestCurrentCmdFrame->ackCode = ackState;// 0x00:OK 0x01:Error
  ackTestCurrentCmdFrame->footer = FRAME_FOOTER;
  checkSum = 0;
  for(uint8_t i = 0; i < length-2; i++ )
  {
    checkSum += txBuf[i];
  }
  ackTestCurrentCmdFrame->checksum = checkSum;
  uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(ackTestCurrentCmdFrame_t)); 
}

void uploadCurrentInformation( voltageData_t *voltageInfo )
{
  uint8_t length = sizeof(uploadCurrentInfoFrame_t);
  uint8_t checkSum;
  
  if( voltageInfo == NULL )
  {
    return;
  }
  uploadCurrentInfoFrame_t *uploadCurrentInfoFrame;
  uploadCurrentInfoFrame = (uploadCurrentInfoFrame_t *)txBuf;
  
  uploadCurrentInfoFrame->header = FRAME_HEADER;
  uploadCurrentInfoFrame->type = FRAME_TYPE_TEST_CURRENT;
  uploadCurrentInfoFrame->length = length;
  uploadCurrentInfoFrame->cmdType = 0x00;

  memcpy(&uploadCurrentInfoFrame->currentInfo, voltageInfo, sizeof(uploadCurrentInfoFrame_t));  
  uploadCurrentInfoFrame->sendRate = voltageDebug.uploadRate;
  memcpy( uploadCurrentInfoFrame->faultBit, voltageConvertData->faultBitTemp, 4 );
  uploadCurrentInfoFrame->footer = FRAME_FOOTER;
  checkSum = 0;
  for( uint8_t i = 0; i < length-2; i++ )
  {
    checkSum += txBuf[i];
  }
  uploadCurrentInfoFrame->checksum = checkSum;
  uart_ser_write(&uart_serial, (uint8_t *)txBuf, sizeof(uploadCurrentInfoFrame_t)); 
}

void recReadErrorDataFrame( void *buf )
{
  recReadErrorDataFrame_t *recReadErrorDataFrame;
  
  recReadErrorDataFrame = (recReadErrorDataFrame_t *)buf;
  (void)recReadErrorDataFrame;
}

void ackReadErrorDataFrame( void )
{
  
}

uint32_t sendRateToTime(uint8_t sendRate)
{
  if(!IS_SEND_RATE_DATA(sendRate))
  {
    return 0;
  }
  switch( sendRate )
  {
  case SEND_RATE_1HZ:
    return 1000/SYSTICK_PERIOD;
  case SEND_RATE_2HZ:
    return 500/SYSTICK_PERIOD;
  case SEND_RATE_5HZ:
    return 200/SYSTICK_PERIOD;
  case SEND_RATE_10HZ:
    return 100/SYSTICK_PERIOD;
  case SEND_RATE_50HZ:
    return 20/SYSTICK_PERIOD;
  case SEND_RATE_100HZ:
    return 10/SYSTICK_PERIOD;
  case SEND_RATE_0_5HZ:
    return 2000/SYSTICK_PERIOD;
  case SEND_RATE_0_2HZ:
    return 5000/SYSTICK_PERIOD;
  case SEND_RATE_0_1HZ:
    return 10000/SYSTICK_PERIOD;
  default:
    return 0;
  }
}

typedef struct {
  uint8_t           detectHeader;
  uint8_t           detectLength;
  uint8_t           detectType;
} bufferHeader_t;

__IO bufferHeader_t      bufferHeader;

void protocol_period( void )
{
  uint8_t checksum;
  uint8_t detectType;
#ifndef COMM_DMA_USE_INT  
  rx_buf.rxCount = receviedDmaDataLength( &ComUartHandle );
  if( rx_buf.rxCount > 0 && rx_buf.rxBuffer[rxBuf[1]-1] == FRAME_FOOTER )//( rx_buf.rxCount > 0 && rx_buf.rxBuffer[rx_buf.rxCount-1] == 0xA5)
  {        
#ifdef PROTOCOL_DEBUG
    if( OPEN_SHOW_LOG == rx_buf.showLogFlag )
    {
      char *debugString;
      debugString = DataToHexStringWithSpaces((const uint8_t *)&rx_buf.rxBuffer[0],rx_buf.rxCount);
      protocol_log("rxBuf: %s", debugString);
      free(debugString);
    }
#endif
    rx_buf.rxCount = 0;
    rx_buf.pData = 0;
    if( rx_buf.startTime == 0 )
    {
      if( STATE_POWER_ON == (boardStatus->sysStatus & STATE_RUN_BITS) )
      {
        setCurLedsMode(LIGHTS_MODE_IDLE);
      }
      protocol_log("start communicating");
    }
    rx_buf.startTime = os_get_time();
  }
  else
  {
    if( (rx_buf.startTime != 0) && ((os_get_time() - rx_buf.startTime) >= COMMUNICATION_TIMEOUT_TIME_MS/SYSTICK_PERIOD) )
    {
      rx_buf.startTime = 0;
      protocol_log("communicate timeout");
      if( STATE_POWER_ON == (boardStatus->sysStatus & STATE_RUN_BITS) )
      {
        setCurLedsMode(LIGHTS_MODE_COM_FAULT);
      }
      Protocol_Init();    
    }
    return;
  }
  stopDmaRecive( &ComUartHandle );
  bufferHeader.detectHeader = rxBuf[0];
  if( bufferHeader.detectHeader == FRAME_HEADER )
  {    
    bufferHeader.detectLength = rxBuf[1];
    bufferHeader.detectType = rxBuf[2];
    if( rxBuf[bufferHeader.detectLength - 1] != FRAME_FOOTER )
    {
      protocol_log("not known cmd");
      return;
    }
    checksum = 0;
    for( uint8_t i = 0; i <= (bufferHeader.detectLength - 3); i++ )
    {
      checksum += rxBuf[i];
    }
    if( checksum == rxBuf[bufferHeader.detectLength - 2] )
    {    
      detectType = bufferHeader.detectType;
      switch( detectType )
      {
      case FRAME_TYPE_NOMAL:
        serialLedsFrameProcess( rxBuf );
        break;
      case FRAME_TYPE_CONFIG:
        serialLedsFrameProcess( rxBuf );
        break;
      case FRAME_TYPE_POWER_CONTROL:
        recPowerControlFrameProcess( rxBuf );
        break;
      case FRAME_TYPE_VERSION_INFO:
        recVersionInfoFrameProcess( rxBuf );
        break;
      case FRAME_TYPE_FW_UPGRADE:
        FirmwareUpgradeProcess( rxBuf );
        break;
      case FRAME_TYPE_LIGHTS_CONF:
        recLightsConfiguration( rxBuf );
        break;
      case FRAME_TYPE_TEST_CURRENT:
        recTestCurrentCmdFrame( rxBuf );
        break;
      default:
        break;
      }
    } 
    else
    {
      protocol_log("check sum not match");
    }
  }
  if( !rx_buf.rxCount )
  {
    memset(rxBuf, 0x00 , UART_RX_BUFFER_LENGTH);    
  }
  startDmaRecive(&ComUartHandle,rx_buf.rxBuffer);
#else //#ifndef COMM_DMA_USE_INT 
  
  rx_buf.rxCount = receviedDmaDataLength( COMM_UART );

  if( rx_buf.rxCount > 0 && rx_buf.rxBuffer[rxBuf[1]-1] == FRAME_FOOTER )//( rx_buf.rxCount > 0 && rx_buf.rxBuffer[rx_buf.rxCount-1] == 0xA5)
  {        
#ifdef PROTOCOL_DEBUG
    if( OPEN_SHOW_LOG == rx_buf.showLogFlag )
    {
      char *debugString;
      debugString = DataToHexStringWithSpaces((const uint8_t *)&rx_buf.rxBuffer[0],rx_buf.rxCount);
      protocol_log("rxBuf: %s", debugString);
      free(debugString);
    }
#endif
    rx_buf.rxCount = 0;
    rx_buf.pData = 0;
    if( rx_buf.startTime == 0 )
    {
      if( STATE_POWER_ON == (boardStatus->sysStatus & STATE_RUN_BITS) )
      {
        setCurLedsMode(LIGHTS_MODE_IDLE);
      }
      protocol_log("start communicating");
    }
    rx_buf.startTime = os_get_time();
  }
  else
  {
    if( (rx_buf.startTime != 0) && ((os_get_time() - rx_buf.startTime) >= COMMUNICATION_TIMEOUT_TIME_MS/SYSTICK_PERIOD) )
    {
      rx_buf.startTime = 0;
      protocol_log("communicate timeout");
      if( STATE_POWER_ON == (boardStatus->sysStatus & STATE_RUN_BITS) )
      {
        setCurLedsMode(LIGHTS_MODE_COM_FAULT);
      } 
      //startDmaRecive( COMM_UART, rx_buf.rxBuffer );
      stopDmaRecive( COMM_UART );
      Protocol_Init();
      return;
    }
    if( (rx_buf.rxBuffer[1] != 0 && rx_buf.rxCount > rx_buf.rxBuffer[1]) || rx_buf.rxCount >= UART_RX_BUFFER_LENGTH - 2 )
    {
      stopDmaRecive( COMM_UART );
      rx_buf.rxCount = 0;     
      goto exit;
    }
    return;
  }
  require_action_quiet( stopDmaRecive( COMM_UART ) == kNoErr, exit, protocol_log("Stop dma rec err") );  
  bufferHeader.detectHeader = rxBuf[0];
  if( bufferHeader.detectHeader == FRAME_HEADER )
  {    
    bufferHeader.detectLength = rxBuf[1];
    bufferHeader.detectType = rxBuf[2];
    if( rxBuf[bufferHeader.detectLength - 1] != FRAME_FOOTER )
    {
      protocol_log("not known cmd");
      return;
    }
    checksum = 0;
    for( uint8_t i = 0; i <= (bufferHeader.detectLength - 3); i++ )
    {
      checksum += rxBuf[i];
    }
    if( checksum == rxBuf[bufferHeader.detectLength - 2] )
    {    
      detectType = bufferHeader.detectType;
      switch( detectType )
      {
      case FRAME_TYPE_NOMAL:
        serialLedsFrameProcess( rxBuf );
        break;
      case FRAME_TYPE_CONFIG:
        serialLedsFrameProcess( rxBuf );
        break;
      case FRAME_TYPE_POWER_CONTROL:
        recPowerControlFrameProcess( rxBuf );
        break;
      case FRAME_TYPE_VERSION_INFO:
        recVersionInfoFrameProcess( rxBuf );
        break;
      case FRAME_TYPE_FW_UPGRADE:
        FirmwareUpgradeProcess( rxBuf );
        break;
      case FRAME_TYPE_LIGHTS_CONF:
        recLightsConfiguration( rxBuf );
        break;
      case FRAME_TYPE_TEST_CURRENT:
        recTestCurrentCmdFrame( rxBuf );
        break;
      default:
        break;
      }
    } 
    else
    {
      protocol_log("check sum not match");
    }
  }
exit:
  if( !rx_buf.rxCount )
  {
    memset( rxBuf, 0x00 , UART_RX_BUFFER_LENGTH );
    startDmaRecive( COMM_UART, rx_buf.rxBuffer );
  }
#endif
}


