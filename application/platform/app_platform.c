/* 
*  Author: Adam Huang
*  Date:2016/6/4
*/
#include "app_platform.h"
#include <stdlib.h>
#include "board_init.h"
//#include "serial_leds.h"
#include "protocol.h"
#include "upgrade_flash.h"

//#define JOY_TEST

#define platform_log(M, ...) custom_log("Platform", M, ##__VA_ARGS__)
#define platform_log_trace() custom_log_trace("Platform")

static switch_t ramSwitch_user;
switch_t *switch_user = &ramSwitch_user;

static boardStatus_t ramBoardStatus;
boardStatus_t *boardStatus = &ramBoardStatus;

static controlSignal_t ramDLP_ControlSignal, ramPAD_ControlSignal, ramX86_ControlSignal;
static controlSignal_t ramNV_ControlSignal;

controlSignal_t *DLP_ControlSignal = &ramDLP_ControlSignal;
controlSignal_t *PAD_ControlSignal = &ramPAD_ControlSignal;
controlSignal_t *X86_ControlSignal = &ramX86_ControlSignal;
controlSignal_t *NV_ControlSignal = &ramNV_ControlSignal;

OSStatus Platform_Init( void )
{ 

  OSStatus err = kNoErr;

  if( err != kNoErr )
  {
    platform_log("platform init error: no enough memory!");
  }
  else
  {
    platform_log("platform init success!");
  }
  return err;
}

void bsp_Init( void )
{
  board_gpios_init();
}

#define SWITCH_STATE_ON         ((uint32_t)0x00000001)
#define DLP_STATE_ON            ((uint32_t)0x00000002)
#define X86_STATE_ON            ((uint32_t)0x00000004)
#define PAD_STATE_ON            ((uint32_t)0x00000008)
#define DC_5V_STATE_ON          ((uint32_t)0x00000010)
#define DC_12V_STATE_ON         ((uint32_t)0x00000020)
#define DC_24V_STATE_ON         ((uint32_t)0x00000040)
#define MOTOR_STATE_ON          ((uint32_t)0x00000080)
#define SENSOR_STATE_ON         ((uint32_t)0x00000100)
#define LEDS_STATE_ON           ((uint32_t)0x00000200)
#define RES_5V_STATE_ON         ((uint32_t)0x00000400)
#define ROUTER_STATE_ON         ((uint32_t)0x00000800)
#define PA_2_1_STATE_ON         ((uint32_t)0x00001000)
#define NV_STATE_ON             ((uint32_t)0x00002000)
#define PRINTER_STATE_ON        ((uint32_t)0x00004000)
#define DYP_STATE_ON            ((uint32_t)0x00008000)
#define RES_12V_STATE_ON        ((uint32_t)0x00010000)
#define RES_24V_STATE_ON        ((uint32_t)0x00020000)
#define BAT_NV_STATE_ON         ((uint32_t)0x00040000)
#define AIUI_STATE_ON           ((uint32_t)0x00080000)


/*********************END OF FILE**************/
