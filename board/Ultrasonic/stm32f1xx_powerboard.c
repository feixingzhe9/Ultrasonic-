/* 
*  Author: Adam Huang 
*  Date:2016/6/4
*/

#include "stm32f1xx_powerboard.h"
#include "Mico.h"
 
uint32_t BSP_SW_GetState(Switch_TypeDef Switch)
{
  (void)Switch;    
  return MicoGpioInputGet( MICO_GPIO_PWRKEY );
}

void BSP_Power_OnOff(PowerEnable_TypeDef PowerEn, PowerOnOff_TypeDef OnOff)
{
  if( POWER_ON == OnOff )
  {
    if( PowerEn & POWER_DLP )
    {
      MicoGpioOutputLow( MICO_GPIO_DLP_EN );
    }
    if( PowerEn & POWER_X86 )
    {
      MicoGpioOutputLow( MICO_GPIO_X86_EN );
    }
    if( PowerEn & POWER_PAD )
    {
      MicoGpioOutputLow( MICO_GPIO_PAD_EN );
    }
    if( PowerEn & POWER_5V_EN )
    {
      MicoGpioOutputLow( MICO_GPIO_5V_EN );
    }
    if( PowerEn & POWER_12V_EN )
    {
      MicoGpioOutputLow( MICO_GPIO_12V_EN );
    }
    if( PowerEn & POWER_24V_EN )
    {
      MicoGpioOutputLow( MICO_GPIO_24V_EN );
    }
    if( PowerEn & POWER_MOTOR )
    {
      MicoGpioOutputLow( MICO_GPIO_MOTOR_EN );
    }
    if( PowerEn & POWER_SENSOR )
    {
      MicoGpioOutputLow( MICO_GPIO_SENSOR_EN );
    }
    if( PowerEn & POWER_LEDS )
    {
      MicoGpioOutputLow( MICO_GPIO_LEDS_EN );
    }
    if( PowerEn & POWER_5V_RES )
    {
      MicoGpioOutputLow( MICO_GPIO_5V_RES_EN );
    }
    if( PowerEn & POWER_12V_ROUTOR )
    {
      MicoGpioOutputLow( MICO_GPIO_12V_ROUTER_EN );
    }
    if( PowerEn & POWER_2_1_PA )
    {
      MicoGpioOutputLow( MICO_GPIO_2_1_PA_EN );
    }
    if( PowerEn & POWER_NV )
    {
      MicoGpioOutputLow( MICO_GPIO_NV_EN );
    }
    if( PowerEn & POWER_PRINTER )
    {
      MicoGpioOutputLow( MICO_GPIO_PRINTER_EN );
    }
    if( PowerEn & POWER_DYP )
    {
      MicoGpioOutputLow( MICO_GPIO_DYP_EN );
    }
    if( PowerEn & POWER_12V_RES )
    {
      MicoGpioOutputLow( MICO_GPIO_12V_RES_EN );
    }
    if( PowerEn & POWER_24V_RES )
    {
      MicoGpioOutputLow( MICO_GPIO_24V_RES_EN );
    }
    if( PowerEn & POWER_BAT_NV )
    {
      MicoGpioOutputLow( MICO_GPIO_BAT_NV_EN );
    }
    if( PowerEn & POWER_AIUI )
    {
      MicoGpioOutputLow( MICO_GPIO_AIUI_EN );
    }
    if( PowerEn & POWER_5V_ROUTOR )
    {
      MicoGpioOutputLow( MICO_GPIO_AIUI_EN );
    }
  }
  else if( POWER_OFF == OnOff )
  {
    if( PowerEn & POWER_DLP )
    {
      MicoGpioOutputHigh( MICO_GPIO_DLP_EN );
    }
    if( PowerEn & POWER_X86 )
    {
      MicoGpioOutputHigh( MICO_GPIO_X86_EN );
    }
    if( PowerEn & POWER_PAD )
    {
      MicoGpioOutputHigh( MICO_GPIO_PAD_EN );
    }
    if( PowerEn & POWER_5V_EN )
    {
      MicoGpioOutputHigh( MICO_GPIO_5V_EN );
    }
    if( PowerEn & POWER_12V_EN )
    {
      MicoGpioOutputHigh( MICO_GPIO_12V_EN );
    }
    if( PowerEn & POWER_24V_EN )
    {
      MicoGpioOutputHigh( MICO_GPIO_24V_EN );
    }
    if( PowerEn & POWER_MOTOR )
    {
      MicoGpioOutputHigh( MICO_GPIO_MOTOR_EN );
    }
    if( PowerEn & POWER_SENSOR )
    {
      MicoGpioOutputHigh( MICO_GPIO_SENSOR_EN );
    }
    if( PowerEn & POWER_LEDS )
    {
      MicoGpioOutputHigh( MICO_GPIO_LEDS_EN );
    }
    if( PowerEn & POWER_5V_RES )
    {
      MicoGpioOutputHigh( MICO_GPIO_5V_RES_EN );
    }
    if( PowerEn & POWER_12V_ROUTOR )
    {
      MicoGpioOutputHigh( MICO_GPIO_12V_ROUTER_EN );
    }
    if( PowerEn & POWER_2_1_PA )
    {
      MicoGpioOutputHigh( MICO_GPIO_2_1_PA_EN );
    }
    if( PowerEn & POWER_NV )
    {
      MicoGpioOutputHigh( MICO_GPIO_NV_EN );
    }
    if( PowerEn & POWER_PRINTER )
    {
      MicoGpioOutputHigh( MICO_GPIO_PRINTER_EN );
    }
    if( PowerEn & POWER_DYP )
    {
      MicoGpioOutputHigh( MICO_GPIO_DYP_EN );
    }
    if( PowerEn & POWER_12V_RES )
    {
      MicoGpioOutputHigh( MICO_GPIO_12V_RES_EN );
    }
    if( PowerEn & POWER_24V_RES )
    {
      MicoGpioOutputHigh( MICO_GPIO_24V_RES_EN );
    }
    if( PowerEn & POWER_BAT_NV )
    {
      MicoGpioOutputHigh( MICO_GPIO_BAT_NV_EN );
    }
    if( PowerEn & POWER_AIUI )
    {
      MicoGpioOutputHigh( MICO_GPIO_AIUI_EN );
    }
  }
}

uint32_t getModulePowerState( PowerEnable_TypeDef PowerEn )
{
    uint32_t pinState;
    
    pinState = (uint32_t)0;
#if 0    
    if( PowerEn & POWER_DLP )
    {
      if( !MicoGpioInputGet( MICO_GPIO_DLP_EN ) )
      {
        pinState |= POWER_DLP;
      }
    }
    if( PowerEn & POWER_X86 )
    {
      if( !MicoGpioInputGet( MICO_GPIO_X86_EN ) )
      {
        pinState |= POWER_X86;
      }
    }
    if( PowerEn & POWER_PAD )
    {
      if( !MicoGpioInputGet( MICO_GPIO_PAD_EN ) )
      {
        pinState |= POWER_PAD;
      }
    }
    if( PowerEn & POWER_NV )
    {
      if( !MicoGpioInputGet( MICO_GPIO_NV_EN ) )
      {
        pinState |= POWER_NV;
      }
    }
#endif
    if( PowerEn & POWER_5V_EN )
    {
      if( !MicoGpioInputGet( MICO_GPIO_5V_EN ) )
      {
        pinState |= POWER_5V_EN;
      }
    }
    if( PowerEn & POWER_12V_EN )
    {
      if( !MicoGpioInputGet( MICO_GPIO_12V_EN ) )
      {
        pinState |= POWER_12V_EN;
      }
    }
    if( PowerEn & POWER_24V_EN )
    {
      if( !MicoGpioInputGet( MICO_GPIO_24V_EN ) )
      {
        pinState |= POWER_24V_EN;
      }
    }
    if( PowerEn & POWER_MOTOR )
    {
      if( !MicoGpioInputGet( MICO_GPIO_MOTOR_EN ) )
      {
        pinState |= POWER_MOTOR;
      }
    }
    if( PowerEn & POWER_SENSOR )
    {
      if( !MicoGpioInputGet( MICO_GPIO_SENSOR_EN ) )
      {
        pinState |= POWER_SENSOR;
      }
    }
    if( PowerEn & POWER_LEDS )
    {
      if( !MicoGpioInputGet( MICO_GPIO_LEDS_EN ) )
      {
        pinState |= POWER_LEDS;
      }
    }
    if( PowerEn & POWER_5V_RES )
    {
      if( !MicoGpioInputGet( MICO_GPIO_5V_RES_EN ) )
      {
        pinState |= POWER_5V_RES;
      }
    }
    if( PowerEn & POWER_12V_ROUTOR )
    {
      if( !MicoGpioInputGet( MICO_GPIO_12V_ROUTER_EN ) )
      {
        pinState |= POWER_12V_ROUTOR;
      }
    }
    if( PowerEn & POWER_2_1_PA )
    {
      if( !MicoGpioInputGet( MICO_GPIO_2_1_PA_EN ) )
      {
        pinState |= POWER_2_1_PA;
      }
    }
    if( PowerEn & POWER_PRINTER )
    {
      if( !MicoGpioInputGet( MICO_GPIO_PRINTER_EN ) )
      {
        pinState |= POWER_PRINTER;
      }
    }
    if( PowerEn & POWER_DYP )
    {
      if( !MicoGpioInputGet( MICO_GPIO_DYP_EN ) )
      {
        pinState |= POWER_DYP;
      }
    }
    if( PowerEn & POWER_12V_RES )
    {
      if( !MicoGpioInputGet( MICO_GPIO_12V_RES_EN ) )
      {
        pinState |= POWER_12V_RES;
      }
    }
    if( PowerEn & POWER_24V_RES )
    {
      if( !MicoGpioInputGet( MICO_GPIO_24V_RES_EN ) )
      {
        pinState |= POWER_24V_RES;
      }
    }
    if( PowerEn & POWER_BAT_NV )
    {
      if( !MicoGpioInputGet( MICO_GPIO_BAT_NV_EN ) )
      {
        pinState |= POWER_BAT_NV;
      }
    }
    if( PowerEn & POWER_AIUI )
    {
      if( !MicoGpioInputGet( MICO_GPIO_AIUI_EN ) )
      {
        pinState |= POWER_AIUI;
      }
    }
    if( PowerEn & POWER_5V_ROUTOR )
    {
      if( !MicoGpioInputGet( MICO_GPIO_5V_ROUTER_EN ) )
      {
        pinState |= POWER_5V_ROUTOR;
      }
    }
    return pinState;
}

void BSP_Control_Sigal(PowerControl_TypeDef PowerCon, ControlSignal_TypeDef isHold)
{
  if( CONTROL_RELEASE == isHold )
  {
#if 1
    switch( PowerCon )
    {
    case POWER_CON_NV:
      MicoGpioOutputLow( MICO_GPIO_PWR_NV ); 
      break;
    case POWER_CON_DLP:
      MicoGpioOutputLow( MICO_GPIO_PWR_DLP ); 
      break;
    case POWER_CON_PAD:
      MicoGpioOutputLow( MICO_GPIO_PWR_PAD ); 
      break;
    case POWER_CON_X86:
      MicoGpioOutputLow( MICO_GPIO_PWR_X86 ); 
      break;
    case POWER_CON_RES:
      MicoGpioOutputLow( MICO_GPIO_PWR_RES );
      break;
    default:
      break;
    }
#else
    if( PowerCon & POWER_CON_NV )
    {
      MicoGpioOutputLow( MICO_GPIO_PWR_NV ); 
    }
    if( PowerCon & POWER_CON_DLP )
    {
      MicoGpioOutputLow( MICO_GPIO_PWR_DLP ); 
    }
    if( PowerCon & POWER_CON_PAD )
    {
      MicoGpioOutputLow( MICO_GPIO_PWR_PAD ); 
    }
    if( PowerCon & POWER_CON_X86 )
    {
      MicoGpioOutputLow( MICO_GPIO_PWR_X86 ); 
    }
    if( PowerCon & POWER_CON_RES )
    {
      MicoGpioOutputLow( MICO_GPIO_PWR_RES ); 
    }
#endif
  }
  else if( CONTROL_HOLD == isHold )
  {
#if 1
    switch( PowerCon )
    {
    case POWER_CON_NV:
      MicoGpioOutputHigh( MICO_GPIO_PWR_NV ); 
      break;
    case POWER_CON_DLP:
      MicoGpioOutputHigh( MICO_GPIO_PWR_DLP ); 
      break;
    case POWER_CON_PAD:
      MicoGpioOutputHigh( MICO_GPIO_PWR_PAD ); 
      break;
    case POWER_CON_X86:
      MicoGpioOutputHigh( MICO_GPIO_PWR_X86 ); 
      break;
    case POWER_CON_RES:
      MicoGpioOutputHigh( MICO_GPIO_PWR_RES );
      break;
    default:
      break;
    }
#else
    if( PowerCon & POWER_CON_NV )
    {
      MicoGpioOutputHigh( MICO_GPIO_PWR_NV ); 
    }
    if( PowerCon & POWER_CON_DLP )
    {
      MicoGpioOutputHigh( MICO_GPIO_PWR_DLP ); 
    }
    if( PowerCon & POWER_CON_PAD )
    {
      MicoGpioOutputHigh( MICO_GPIO_PWR_PAD ); 
    }
    if( PowerCon & POWER_CON_X86 )
    {
      MicoGpioOutputHigh( MICO_GPIO_PWR_X86 ); 
    }
    if( PowerCon & POWER_CON_RES )
    {
      MicoGpioOutputHigh( MICO_GPIO_PWR_RES ); 
    }
#endif
  }
}

#ifndef BOOTLOADER
void halEnterSleepMode( void )
{
//  BSP_LED_Off( LED_SYS );
//  HAL_PWR_EnterSTANDBYMode();
}

void halWakeupFormSLeep( void )
{
//  init_clocks();
}
#endif

