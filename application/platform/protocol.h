/* 
*  Author: Adam Huang
*  Date:2016/12/13
*/
#ifndef __PROTOCOL_H
#define __PROTOCOL_H
#include <stdint.h>
#include "serial_uart.h"
#include "RingBufferUtils.h"
#include "voltage_detect.h"

#define PROTOCOL_DEBUG

#define UART_RX_BUFFER_LENGTH            255
#define UART_TX_BUFFER_LENGTH            90

#define FRAME_HEADER                    0x5A
#define FRAME_FOOTER                    0xA5



#define FRAME_TYPE_FW_UPGRADE           0x0F

#define CONTROL_SYSTEM_SHUTDOWN         0x01

#define READ_FW_VERSION_COMMAND         0x00
#define READ_PROTOCOL_VERSION           0x01

#define HW_VERSION                      "10"
#define SW_VERSION                      "C001M07B103"
#define PROTOCOL_VERSION                "20170505R0101"

typedef struct _serial_frame_t {
  uint8_t               header;
  uint8_t               length;
} serial_frame_t;

typedef struct {
  uint8_t           detectHeader;
  uint8_t           detectLength;
  uint8_t           detectType;
} bufferHeader_t;

typedef struct recBuf_t {
  uint8_t               showLogFlag;
#define         CLOSE_SHOW_LOG  0x00
#define         OPEN_SHOW_LOG   0x01
  uint8_t               pData;
  uint8_t               *rxBuffer;
  bufferHeader_t        bufferHeader;
  uint8_t               rxCount;
  uint32_t              startTime;
#define         COMMUNICATION_TIMEOUT_TIME_MS           6000
  
} recBuf_t;

typedef struct _ram_buff_t {
  const uint8_t         *pBuffer;
  const uint8_t         *offset;
  uint8_t               *pData;
  const uint32_t        bufferSize;
  uint32_t              receiveStartTime;
} ram_buff_t;

typedef struct _serial_t {
  volatile uint8_t              isSerialInitialized;
  volatile uint8_t              isStartDmaReceive;
  recBuf_t                      *rx_info;
  uart_serial_t                 *uart_serial;
  ram_buff_t                    rx_buf;
  ram_buff_t                    tx_buf;
} serial_t;


/***************** begin of upgrade defines ********************/

#define UPGRADE_FRAME_TYPE_PREPARE      0x00
#define UPGRADE_FRAME_TYPE_GOING        0x01
#define UPGRADE_FRAME_TYPE_FINISH       0x02

#define MCU_PREPARE_OK                  0x00
#define MCU_STORAGE_NOT_ENOUGH          0x01
#define MCU_RETRY_LATER                 0x02

#define PACKGE_PROCESS_OK               0x00
#define PACKGE_PROCESS_FAIL             0x01

#define FIRMWARE_CHECKSUM_RIGHT         0x00
#define FIRMWARE_CHECKSUM_ERR           0x01

typedef struct _recPrepareUpgradeFrame_t {
  uint8_t               upgradeFrameType;
  uint8_t               md5[16];
  uint8_t               firmwareSize[4];
} recPrepareUpgradeFrame_t;

typedef struct _recFirmwarePackageFrame_t {
  uint8_t               upgradeFrameType;
  uint8_t               *pPackageData;
  uint32_t              packageDataLength;
} recFirmwarePackageFrame_t;

typedef struct _recUpgradeFinishCheckFrame_t {
  uint8_t               upgradeFrameType;
  uint8_t               finishFlag;
} recUpgradeFinishCheckFrame_t;

typedef struct _upgradeGeneralAckFrame_t {
  uint8_t               ctype;
  uint8_t               upgradeFrameType;
  uint8_t               ackState;
} upgradeGeneralAckFrame_t;

typedef struct _recTestCurrentCmdFrame_t {
  uint8_t               cmdType;
  uint8_t               command;
  uint8_t               sendRate;
#define                         SEND_RATE_SINGLE        ((uint8_t)0x00)
#define                         SEND_RATE_1HZ           ((uint8_t)0x01)
#define                         SEND_RATE_2HZ           ((uint8_t)0x02)
#define                         SEND_RATE_5HZ           ((uint8_t)0x03)
#define                         SEND_RATE_10HZ          ((uint8_t)0x04)
#define                         SEND_RATE_50HZ          ((uint8_t)0x05)
#define                         SEND_RATE_100HZ         ((uint8_t)0x06)
#define                         SEND_RATE_0_5HZ         ((uint8_t)0x07)
#define                         SEND_RATE_0_2HZ         ((uint8_t)0x08)
#define                         SEND_RATE_0_1HZ         ((uint8_t)0x09) 
} recTestCurrentCmdFrame_t;

#define         IS_SEND_RATE_DATA(x) (((x) == SEND_RATE_SINGLE) ||\
                                      ((x) == SEND_RATE_1HZ) ||\
                                      ((x) == SEND_RATE_2HZ) ||\
                                      ((x) == SEND_RATE_5HZ) || \
                                      ((x) == SEND_RATE_10HZ) ||\
                                      ((x) == SEND_RATE_50HZ) ||\
                                      ((x) == SEND_RATE_100HZ) ||\
                                      ((x) == SEND_RATE_0_5HZ) ||\
                                      ((x) == SEND_RATE_0_2HZ) ||\
                                      ((x) == SEND_RATE_0_1HZ) )

typedef struct _uploadCurrentInfoFrame_t {
  uint8_t               ctype;
  uint8_t               cmdType;
  voltageData_t         currentInfo;  
  uint8_t               faultBit[4];
  uint8_t               sendRate;
  uint8_t               reserve[7];
} uploadCurrentInfoFrame_t;

typedef struct _errInfoFrame_t {
  uint8_t               ctype;
  uint8_t               errChannel;
  uint16_t              errData;  
} errInfoFrame_t;

/***************** end of upgrade defines *********************/
extern serial_t * const serial;

void protocol_period( void );
OSStatus Protocol_Init( void );
OSStatus uploadCurrentInformation( serial_t *serial, voltageData_t *voltageInfo );
uint32_t sendRateToTime(uint8_t sendRate);
//OSStatus Protocol_Init( serial_t *serial );

#endif
