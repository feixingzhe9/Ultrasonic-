/* 
*  Author: Adam Huang 
*  Date:2016/6/4
*/
#ifndef __STM32F1XX_POWERBOARD_H
#define __STM32F1XX_POWERBOARD_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f1xx_hal.h"

    
typedef enum
{
  POWER_OFF = 0,
  POWER_ON = 1
} PowerOnOff_TypeDef;

typedef enum {
  CONTROL_RELEASE = 0,
  CONTROL_HOLD = 1
} ControlSignal_TypeDef;

typedef enum 
{
  LED_SYS = 0,
  
} Led_TypeDef;

typedef enum 
{  
  SWITCH_USER = 0,
} Switch_TypeDef;

typedef enum 
{  
  SWITCH_MODE_GPIO = 0,
  SWITCH_MODE_EXTI = 1
} SwitchMode_TypeDef; 

typedef enum
{
  POWER_SWITCH          = 0x00000001,
  POWER_DLP             = 0x00000002,
  POWER_X86             = 0x00000004,
  POWER_PAD             = 0x00000008,
  POWER_5V_EN           = 0x00000010,
  POWER_12V_EN          = 0x00000020,
  POWER_24V_EN          = 0x00000040,
  POWER_MOTOR           = 0x00000080,
  POWER_SENSOR          = 0x00000100,
  POWER_LEDS            = 0x00000200,
  POWER_5V_RES          = 0x00000400,
  POWER_12V_ROUTOR      = 0x00000800,
  POWER_2_1_PA          = 0x00001000,
  POWER_NV              = 0x00002000,
  POWER_PRINTER         = 0x00004000,
  POWER_DYP             = 0x00008000,
  POWER_12V_RES         = 0x00010000,
  POWER_24V_RES         = 0x00020000,
  POWER_BAT_NV          = 0x00040000,
  POWER_AIUI            = 0x00080000,
  POWER_5V_ROUTOR       = 0x00100000,
  POWER_ALL             = 0x001FFFFF,
} PowerEnable_TypeDef;

typedef enum
{
  POWER_CON_DLP = 0x01,
  POWER_CON_PAD = 0x02,
  POWER_CON_X86 = 0x04,
  POWER_CON_NV  = 0x08,
  POWER_CON_RES = 0x10,
  POWER_CON_ALL = 0x1F
} PowerControl_TypeDef;

uint32_t        BSP_GetVersion(void);

void            Serial_Leds_Init( void );

void            BSP_SW_Init(Switch_TypeDef Switch, SwitchMode_TypeDef SwitchMode);
uint32_t        BSP_SW_GetState(Switch_TypeDef Switch);

void     BSP_GPIO_Init( void );
void     BSP_Power_OnOff(PowerEnable_TypeDef PowerEn, PowerOnOff_TypeDef OnOff);
uint32_t getModulePowerState(PowerEnable_TypeDef PowerEn);
void     BSP_Control_Sigal(PowerControl_TypeDef PowerCon, ControlSignal_TypeDef isHold);

extern void halEnterSleepMode( void );

extern void halWakeupFormSLeep( void );

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1XX_POWERBOARD_H */

