/* 
*  Author: Adam Huang
*  Date:2016/6/4
*/
#ifndef __APP_PLATFORM_H
#define __APP_PLATFORM_H

#include <stdint.h>
#include "stm32f1xx_powerboard.h"
#include "Common.h"
#include "Debug.h"

#define OWN_PAD

#define os_get_time                   HAL_GetTick

#define SYSTICK_PERIOD                1      //1ms

#define SWITCH_DEBOUNCE_TIME          1000/SYSTICK_PERIOD   //1s
#ifdef OWN_PAD
#define DLP_POWER_ON_DELAY_TIME       1000/SYSTICK_PERIOD   //1s
#define DLP_POWER_OFF_DELAY_TIME      1000/SYSTICK_PERIOD   //1s
#define DLP_POWER_ON_HOLD_TIME        5000/SYSTICK_PERIOD   //5s
#define DLP_POWER_OFF_HOLD_TIME       5000/SYSTICK_PERIOD   //5s
#define DLP_POWER_ON_PROCESSING_TIME  30000/SYSTICK_PERIOD   //30s
#define DLP_POWER_OFF_PROCESSING_TIME 20000/SYSTICK_PERIOD   //20s
#else
#define DLP_POWER_ON_DELAY_TIME       1000/SYSTICK_PERIOD   //1s
#define DLP_POWER_OFF_DELAY_TIME      1000/SYSTICK_PERIOD   //1s
#define DLP_POWER_ON_HOLD_TIME        5000/SYSTICK_PERIOD   //5s
#define DLP_POWER_OFF_HOLD_TIME       1000/SYSTICK_PERIOD   //1s
#define DLP_POWER_ON_PROCESSING_TIME  15000/SYSTICK_PERIOD   //15s
#define DLP_POWER_OFF_PROCESSING_TIME 10000/SYSTICK_PERIOD   //10s
#endif

#ifdef OWN_PAD
#define PAD_POWER_ON_DELAY_TIME       1000/SYSTICK_PERIOD   //1s
#define PAD_POWER_OFF_DELAY_TIME      1000/SYSTICK_PERIOD   //1s
#define PAD_POWER_ON_HOLD_TIME        4000/SYSTICK_PERIOD   //4s
#define PAD_POWER_OFF_HOLD_TIME       5000/SYSTICK_PERIOD   //5s
#define PAD_POWER_ON_PROCESSING_TIME  30000/SYSTICK_PERIOD   //30s
#define PAD_POWER_OFF_PROCESSING_TIME 20000/SYSTICK_PERIOD   //20s
#else
#define PAD_POWER_ON_DELAY_TIME       1000/SYSTICK_PERIOD   //1s
#define PAD_POWER_OFF_DELAY_TIME      1000/SYSTICK_PERIOD   //1s
#define PAD_POWER_ON_HOLD_TIME        1000/SYSTICK_PERIOD   //1s
#define PAD_POWER_OFF_HOLD_TIME       6000/SYSTICK_PERIOD   //6s
#define PAD_POWER_ON_PROCESSING_TIME  15000/SYSTICK_PERIOD   //15s
#define PAD_POWER_OFF_PROCESSING_TIME 20000/SYSTICK_PERIOD   //20s
#endif

#define X86_POWER_ON_DELAY_TIME       1000/SYSTICK_PERIOD   //1s
#define X86_POWER_OFF_DELAY_TIME      1000/SYSTICK_PERIOD   //1s
#define X86_POWER_ON_HOLD_TIME        2000/SYSTICK_PERIOD   //2s
#define X86_POWER_OFF_HOLD_TIME       6000/SYSTICK_PERIOD   //6s
#define X86_POWER_ON_PROCESSING_TIME  15000/SYSTICK_PERIOD   //15s
#define X86_POWER_OFF_PROCESSING_TIME 15000/SYSTICK_PERIOD   //15s

#define NV_POWER_ON_DELAY_TIME                 10000/SYSTICK_PERIOD   //5s
#define NV_POWER_OFF_DELAY_TIME                1000/SYSTICK_PERIOD   //1s
#if 0
#define NV_POWER_ON_HOLD_TMP1_TIME              70/SYSTICK_PERIOD   //50ms
#define NV_POWER_ON_RELASE_TMP_TIME             200/SYSTICK_PERIOD   //200ms
#define NV_POWER_ON_HOLD_TMP2_TIME              70/SYSTICK_PERIOD   //50ms
#else
#define NV_POWER_ON_HOLD_TIME                   70/SYSTICK_PERIOD   //50ms
#endif
#define NV_POWER_ON_PROCESSING_TMP_TIME         40000/SYSTICK_PERIOD   //40s
#define NV_POWER_OFF_HOLD_TMP_TIME              7500/SYSTICK_PERIOD   //7.5s
#define NV_POWER_OFF_PROCESSING_TIME            15000/SYSTICK_PERIOD   //15s

#define BOARD_REBOOT_WAITING_TIME               5000/SYSTICK_PERIOD  //5s

#define YES   1
#define NO    (!YES)

#define POWER_ON_PROCEESING_TIME      45000/SYSTICK_PERIOD   //45s
#define POWER_OFF_PROCEESING_TIME     30000/SYSTICK_PERIOD   //30s

typedef  uint32_t (*getSwitchState_Fn)(Switch_TypeDef Switch);
typedef  void (*setPowerOnoff_Fn)(PowerEnable_TypeDef PowerEn, PowerOnOff_TypeDef OnOff);
typedef  void (*setControlSigal_Fn)(PowerControl_TypeDef PowerCon, ControlSignal_TypeDef isHold);

typedef struct _switch_t {
  uint8_t       isSwitchOver;
  uint8_t       switchOnOff;
#define                 SWITCH_OFF    0x01
#define                 SWITCH_ON     0x02 
  uint8_t       preIOState;
  uint32_t      debounceTime;
  uint32_t      startTime;
  getSwitchState_Fn  getSwitchState;      
} switch_t;

typedef struct _boardStatus_t {
  uint8_t            isPowerOnFinish;
  uint8_t            isPowerOffFinish;
  uint16_t           sysStatus;
#define                 STATE_RUN_BITS     0x0F
#define                 STATE_POWER_OFF    0x00
#define                 STATE_IS_POWER_ON  0x01
#define                 STATE_POWER_ON     0x02
#define                 STATE_IS_POWER_OFF 0x03
//#define                 STATE_ERROR        0x04

#define                 STATE_IS_CHARGING       0x10
#define                 STATE_IS_LOW_POWER      0x20
#define                 STATE_IS_AUTO_UPLOAD    0x40
#define                 STATE_IS_CHARGER_IN     0x80
#define                 SYSTEM_IS_SLEEP         0x00 //set 0x00 to no use
  uint8_t            devicesOnOff;
#define                 DEVICES_OFF    0x00
#define                 DEVICES_ON     0x01 
  setPowerOnoff_Fn   setPowerOnoff;  
  uint16_t           vBatLevel;  
  uint16_t           vChargerLevel;
  uint8_t            rebootFlag;
#define                 REBOOT_NO      0x00
#define                 REBOOT_YES     0x01
  uint8_t            errChannel;
  uint16_t           errValue;
  uint32_t           startTime;
#ifdef MIKE_TEST
  uint32_t           charger_times;
#endif
  uint8_t            irled_duty;
  uint8_t            isUpgrading;
} boardStatus_t;

typedef struct _controlSignal_t { 
  uint8_t      period;
  uint8_t      isDeviceProcessOver;
  uint8_t      deviceOnOff;
#define                 DEVICE_POWER_OFF   0x00
#define                 DEVICE_POWER_ON    0x01
  uint16_t     powerOnDelayTime;
  uint16_t     powerOffDelayTime;
  uint16_t     powerOnHoldTime;
  uint16_t     powerOffHoldTime;
  setControlSigal_Fn setControlSigal;
  uint32_t     startTime;
#define                SIGNAL_DELAY       0x00
#define                SIGNAL_HOLD        0x01
#define                SIGNAL_REALSE      0x02
#define                SIGNAL_HOLD_TMP1   0x03
#define                SIGNAL_REALSE_TMP  0x04
#define                SIGNAL_HOLD_TMP2   0x05
} controlSignal_t;

extern switch_t *switch_user;
extern boardStatus_t *boardStatus;

extern void PowerOffDevices( void );
extern void PowerOnDevices( void );
extern OSStatus Platform_Init( void );
extern uint32_t getEachModuleStates( void );
extern void setModulePowerSignalOnOff( uint8_t module, uint8_t onoff );
extern void Platform_Tick( void );
extern void bsp_Init( void );
extern void SystemClock_Config( void );
extern void key_switch_interrupt_cb( void );
#endif  /* __PLATFORM_H */

