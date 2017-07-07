/* 
*  Author: Adam Huang
*  Date:2016/6/15
*/
#ifndef __SERIAL_LED_H__
#define __SERIAL_LED_H__
#include <stdint.h>
#include "Common.h"

#define BITBAND(addr, bitnum) 		((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)  					*((__IO uint32_t*)(addr))

#define GPIOout(GPIOx,bit)				MEM_ADDR(BITBAND((uint32_t)(&GPIOx->ODR), bit))
#define GPIOin(GPIOx,bit)					MEM_ADDR(BITBAND((uint32_t)(&GPIOx->IDR), bit))

#define LED		GPIOout(GPIOC, 6)//GPIOout(GPIOB, 12)

#define LIGHTS_EFFECT_DEFAULT                   0x0000
#define BASIC_LIGHTS_EFFECT_FLOW                0x0001
#define BASIC_LIGHTS_EFFECT_METEOR              0x0002
#define BASIC_LIGHTS_EFFECT_DOUBLE_METEOR       0x0003
#define BASIC_LIGHTS_EFFECT_SHINE               0x0004
#define BASIC_LIGHTS_EFFECT_RED_BEAT            0x0005
#define BASIC_LIGHTS_EFFECT_YELLOW_BEAT         0x0006
#define BASIC_LIGHTS_EFFECT_WATER_GREEN         0x0007
#define BASIC_LIGHTS_EFFECT_BREATH_COLORFUL     0x0008
#define BASIC_LIGHTS_EFFECT_BREATH_PURPLE       0x0009
#define BASIC_LIGHTS_EFFECT_RED_METEOR          0x000A
#define BASIC_LIGHTS_EFFECT_YELLOW_METEOR       0x000B
#define BASIC_LIGHTS_EFFECT_GREEN_METEOR        0x000C
#define BASIC_LIGHTS_EFFECT_RED_LONG            0x000D
#define BASIC_LIGHTS_EFFECT_GREEN_LONG          0x000E
#define BASIC_LIGHTS_EFFECT_BLUE_LONG           0x000F

#define DANCE_LIGHTS_EFFECT_DEFAULT             0xFF00
#define DANCE_LIGHTS_EFFECT_SONG1               0xFF01
#define DANCE_LIGHTS_EFFECT_SONG2               0xFF02

typedef enum {
  LIGHTS_MODE_DEFAULT                   = 0x00,
  LIGHTS_MODE_WELCOME                   = 0x01,
  LIGHTS_MODE_GOODEBYE                  = 0x02,
  LIGHTS_MODE_IDLE                      = 0x03,
  LIGHTS_MODE_DANCE                     = 0x04,
  LIGHTS_MODE_WARMING                   = 0x05,
  LIGHTS_MODE_COM_FAULT                 = 0x06,
  LIGHTS_MODE_BARRIER                   = 0x07,
  LIGHTS_MODE_SOLICIT                   = 0x08,
  LIGHTS_MODE_FREEDOM                   = 0x09,
  LIGHTS_MODE_IS_CHARGING               = 0x0A,
  LIGHTS_MODE_CHARGE_TO_ON              = 0x0B,
  LIGHTS_MODE_CHARGE_FINISH             = 0x0C,
  LIGHTS_MODE_MAX
} lightsMode_t;

/*
#define LIGHTS_MODE_DEFAULT             0x00
#define LIGHTS_MODE_WELCOME             0x01
#define LIGHTS_MODE_GOODEBYE            0x02
#define LIGHTS_MODE_IDLE                0x03
#define LIGHTS_MODE_DANCE               0x04
#define LIGHTS_MODE_WARMING             0x05
#define LIGHTS_MODE_COM_FAULT           0x06
#define LIGHTS_MODE_BARRIER             0x07
#define LIGHTS_MODE_SOLICIT             0x08
#define LIGHTS_MODE_FREEDOM             0x09
*/
#define TOTAL                   148
#define LEVEL                   20

#define SERIAL_LEDS_PERIOD      10 

#if 1
#define asm            __asm
#define delay_300ns()     do {asm("nop");asm("nop");asm("nop");asm("nop");\
                              asm("nop");asm("nop");asm("nop");asm("nop");\
                              asm("nop");asm("nop");asm("nop");asm("nop");\
                              asm("nop");asm("nop");asm("nop");asm("nop");\
                              asm("nop");asm("nop");asm("nop");asm("nop");\
                              asm("nop");asm("nop");} while(1==0)
                                
#define delay_600ns()     do { asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");asm("nop");asm("nop");\
                               asm("nop");asm("nop");} while(1==0)
                                
#define delay_us(n)       do { for(uint32_t i=0;i<n;i++){delay_300ns();delay_600ns();}\
                                } while(0==1)
//#else

#define delay_ms          HAL_Delay
#endif

typedef void (*freshSerialLedsFn_t)(void);

typedef struct _leds_effect_t {
  uint16_t               freshTime;
  freshSerialLedsFn_t    freshSerialLedshandle;
} leds_effect_t;

typedef struct _serial_leds_t {
  uint8_t                modeType;
  uint16_t               effectType;
  leds_effect_t          *leds_effect;
} serial_leds_t;


extern serial_leds_t *serial_leds;

void write_0(void);
void write_1(void);
void write_RESET(void);
void write_24bit(uint32_t word);
void reset_led(void);
//void delay_400ns(void);
//void delay_900ns(void);

OSStatus SerialLeds_Init( void );

uint32_t change_led(uint32_t word,uint8_t level);

void single_color_water_led(uint32_t color,uint8_t times);

void style_charge_function( uint32_t color );
//void test_nsDelay(void);

void setSerialLedsEffect( lightsMode_t lightsMode, uint16_t effect );

void setCurLedsMode( lightsMode_t lightsMode );

void startDanceLedsMode( void );
void stopDanceLedsMode( void );
void serialLedsTick( void );
#endif

