/* 9+
*  Author: Adam Huang
*  Date:2016/6/4
*/
#ifndef __PROTOCOL_H
#define __PROTOCOL_H
#include <stdint.h>
#include "RingBufferUtils.h"
#include "voltage_detect.h"

#define PROTOCOL_DEBUG
//uncomment below line to use new protocol
#ifndef PROTOCOL_NEW
#define PROTOCOL_NEW
#endif

#define FRAME_HEADER                    0x5A
#define FRAME_FOOTER                    0xA5

#define FRAME_TYPE_NOMAL                0x01
#define FRAME_TYPE_CONFIG               0x02
#define FRAME_TYPE_POWER_CONTROL        0x03

#ifndef PROTOCOL_NEW
#define FRAME_TYPE_VERSION_INFO         0x04
#else
#define FRAME_TYPE_VERSION_INFO         0x0E
#endif

#define FRAME_TYPE_LIGHTS_CONF          0x06
#define FRAME_TYPE_TEST_CURRENT         0x0A
#define FRAME_TYPE_FW_UPGRADE           0x0F

#define CONTROL_SYSTEM_SHUTDOWN         0x01
#define READ_FW_VERSION_COMMAND         0x00

#define SW_VERSION                      "C001M07C001"

#define UART_RX_BUFFER_LENGTH            255
#define UART_TX_BUFFER_LENGTH            100

typedef struct _recSerialLedsFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               lightMode;
  uint8_t               lightEffectH;
  uint8_t               lightEffectL;
  uint8_t               checksum;
  uint8_t               footer;
} recSerialLedsFrame_t;

typedef struct _currentInfo_t {
  uint8_t               totalCurDet;
  uint8_t               motorCurDet;
  uint8_t               chassisCurDet;
  uint8_t               camera5VCurDet;
  uint8_t               antiBarrierCurDet;
  uint8_t               dlpCurDet;
  uint8_t               serialLedsCurDet;
  uint8_t               padCurDet;
  uint8_t               x86CurDet;
  uint8_t               audioCurDet;
  uint8_t               camera12VCurDet;
  uint8_t               printerCurDet;
} currentInfo_t;

typedef struct _ackSerialLedsFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               curLightMode;
  uint8_t               curLightEffectH;
  uint8_t               curLightEffectL;
  uint8_t               sysStatus;
  uint8_t               vBat;
  uint8_t               moduleStatus[4];
  uint8_t               faultBit[4];
  uint8_t               checksum;
  uint8_t               footer;
} ackSerialLedsFrame_t;

typedef struct _recPowerControlFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               controlBit;
  uint8_t               checksum;
  uint8_t               footer;
} recPowerControlFrame_t;

typedef struct _ackPowerControlFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               controlBit;
  uint8_t               checksum;
  uint8_t               footer;
} ackPowerControlFrame_t;

typedef struct _recVersionInfoFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               readCommand;
  uint8_t               checksum;
  uint8_t               footer;              
} recVersionInfoFrame_t;

typedef struct _ackVersionInfoFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               hw1;
  uint8_t               hw2;
  uint8_t               sw[11];  
  uint8_t               checksum;
  uint8_t               footer;              
} ackVersionInfoFrame_t;

typedef struct _recLightsConfigFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               danceSN;
  uint8_t               lightsTotal;
  uint8_t               curLightsSN;
  uint8_t               lightsEffect;
  uint8_t               effectDuration;
  uint8_t               checksum;
  uint8_t               footer;     
} recLightsConfigFrame_t;

typedef struct _ackLightsConfigFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               danceSN;
  uint8_t               lightsTotal;
  uint8_t               curLightsSN;
  uint8_t               recStatus;
  uint8_t               checksum;
  uint8_t               footer;     
} ackLightsConfigFrame_t;

#define UPGRADE_PREPARE                 0x00
#define UPGRADE_GOING                   0x01
#define UPGRADE_FINISH                  0x02

#define MCU_PREPARE_OK                  0x00
#define MCU_STORAGE_NOT_ENOUGH          0x01
#define MCU_RETRY_LATER                 0x02

typedef struct _recPrepareUpgradeFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               upgradeType;
  uint8_t               md5[16];
  uint8_t               firmwareSize[4];
  uint8_t               checksum;
  uint8_t               footer;
} recPrepareUpgradeFrame_t;

typedef struct _ackPrepareUpgradeFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               upgradeType;
  uint8_t               ackState;
  uint8_t               checksum;
  uint8_t               footer;
} ackPrepareUpgradeFrame_t;

typedef struct _recFirmwarePackageFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               upgradeType;
  uint8_t               *pPackageData;
  uint8_t               checksum;
  uint8_t               footer;
} recFirmwarePackageFrame_t;

typedef struct _ackFirmwarePackageFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               upgradeType;
  uint8_t               ackState;
  uint8_t               checksum;
  uint8_t               footer;
} ackFirmwarePackageFrame_t;

typedef struct _recUpgradeFinishCheckFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               upgradeType;
  uint8_t               finishFlag;
  uint8_t               checksum;
  uint8_t               footer;
} recUpgradeFinishCheckFrame_t;

typedef struct _ackUpgradeFinishCheckFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               upgradeType;
  uint8_t               ackState;
  uint8_t               checksum;
  uint8_t               footer;
} ackUpgradeFinishCheckFrame_t;

typedef struct _recTestCurrentCmdFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
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
  uint8_t               checksum;
  uint8_t               footer;
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

typedef struct _ackTestCurrentCmdFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               cmdType;
  uint8_t               ackCode;
  uint8_t               checksum;
  uint8_t               footer;
} ackTestCurrentCmdFrame_t;

typedef struct _uploadCurrentInfoFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               cmdType;
  voltageData_t         currentInfo;  
  uint8_t               faultBit[4];
  uint8_t               sendRate;
  uint8_t               reserve;
  uint8_t               checksum;
  uint8_t               footer;
} uploadCurrentInfoFrame_t;

typedef struct _recReadErrorDataFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               cmdType;
  uint8_t               checksum;
  uint8_t               footer;  
} recReadErrorDataFrame_t;

typedef struct _ackReadErrorDataFrame_t {
  uint8_t               header;
  uint8_t               length;
  uint8_t               type;
  uint8_t               errTime[4];
  uint8_t               batVoltage[2];
  uint8_t               totalCur[2];
  uint8_t               errChannel;
  uint8_t               errCur[2];
  uint8_t               errTimes;
  uint8_t               curTime[4];
  uint8_t               checksum;
  uint8_t               footer;
} ackReadErrorDataFrame_t;

typedef struct _serialResponse_t {
  uint8_t               notFinish;
} serialResponse_t;
typedef void ( * serialRespnse_Fn)( serialResponse_t *response );

typedef struct recBuf_t {
  uint8_t               showLogFlag;
#define         CLOSE_SHOW_LOG  0x00
#define         OPEN_SHOW_LOG   0x01
  uint8_t               pData;
  uint8_t               *rxBuffer;
  volatile uint8_t      rxCount;
  uint32_t              startTime;
#define         COMMUNICATION_TIMEOUT_TIME_MS           2000
  
} recBuf_t;

extern recBuf_t rx_buf;
//extern ring_buffer_t rx_buffer;

void Protocol_Init( void );

void serialLedsFrameProcess( void *buf );
void ackSerialLedsFrameProcess( uint8_t type );
void recPowerControlFrameProcess( void *buf );
void ackPowerControlFrameProcess( void *buf );
void recVersionInfoFrameProcess( void *buf );
void ackVersionInfoFrameProcess( void );
void recLightsConfiguration( void *buf );
void ackLightsConfiguration( void );
void FirmwareUpgradeProcess( void *buf );
void recPrepareUpgrade( void *buf );
void ackPrepareUpgrade( uint8_t ackState );
void recFirmwarePackageFrame( void *buf );
void ackFirmwarePackageFrame( uint8_t ackState );
void recUpgradeFinishCheckFrame( void *buf );
void ackUpgradeFinishCheckFrame( uint8_t ackState );
void recTestCurrentCmdFrame( void *buf );
void ackTestCurrentCmdFrame( uint8_t ackState );
void uploadCurrentInformation( voltageData_t *voltageInfo );
void recReadErrorDataFrame( void *buf );
void ackReadErrorDataFrame( void );
uint32_t sendRateToTime(uint8_t sendRate);

void protocol_period( void );

#endif
