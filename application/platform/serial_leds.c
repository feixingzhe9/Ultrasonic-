/* 
*  Author: Adam Huang modfiy
*  Date:2016/6/4
*/
#include "serial_leds.h"
#include "app_platform.h"
#include "mico.h"
//#include "platform.h"
//#include "platform_internal.h"
//#include "platform_config.h"

#define LIGHTS_DEBUG

#define LEDS_MODES_N                    LIGHTS_MODE_MAX
#define LED_EFFECT_N                    10

#define serial_leds_log(M, ...) custom_log("SerialLeds", M, ##__VA_ARGS__)
#define serial_leds_log_trace() custom_log_trace("SerialLeds")

__IO uint32_t buff[TOTAL] = {0};
serial_leds_t *serial_leds;

typedef struct music_type_t {
  uint8_t   indexMax;
	uint8_t 	type[15];
	uint16_t 	rate[15];		//1ms
	uint8_t 	time[15];		//100ms
	uint8_t		strength[15];
} music_t;

#define  SONG1  0
#define  SONG2  1
#define  SONG3  2
const music_t music[] = {
 [SONG1] = {{12},
            {0,  7,  1,  2,  5,  7,  3,  4,  3,  5,  4,  7},
            {0, 20, 10, 20, 60, 20, 20,450, 20, 60,450, 20},
            {0, 38, 38, 76, 76, 95, 76, 76, 76, 38, 38,  3}
           },//dawang
#if 0
 [SONG2] = {{14},
            {0,  2,  3,  7,  1,  3,  4,  1,  8,  3,  2,  4,  7,  3},
            {0, 20, 20, 20,400, 20,350,400,100, 20, 20,450, 20, 20},
            {0, 20, 40, 30, 80, 80, 80, 40,100, 30,160,120, 20, 40}
           //0, 20, 60, 90,170,250,330,370,470,500,660,780,800,840
           },//dujuan--Adam
#else
 [SONG2] = {{13},
            {0,  4,  1,  2,  5,  7,  3,  4,  3,  5,  4,  7,  8},
            {0,450, 10, 20, 60, 10, 20,450, 20, 60,450, 40,150},
            {0, 41, 41, 41, 41, 82, 82, 82, 82, 82, 82, 82, 82}
           //0, 41, 82,123,164,246,328,410,492,574,656,738,820
           },//dujuan--Kiqi
#endif
};

enum {
  FLOW_WATER = 1,
  STAR = 2,
  DOUBLE_STAR = 3,
  SHINE = 4,
  RED_BEAT = 5,
  YELLOW_BEAT = 6,
  WATER_GREEN = 7,
  BREATH_COLORFUL = 8,
  BREATH_PURPLE = 9,
  RED_STAR = 10,
  YELLOW_STAR = 11,
  GREEN_STAR = 12,
  RED_LONG = 13,
  GREEN_LONG = 14,
  BLUE_LONG = 15,
};

static void send_data(void);
static void style_water_function(void);
static void style_star_function(void);
static void style_doublestar_function(void);
static void style_shine_function(void);
static void style_breath_function(void);
static void style_yellow_beat_function(void);	
static void style_red_beat_function(void);	
static void style_water_green_function(void);
static void style_breath_colorful_function(void);
static void style_red_star_function( void );
static void style_yellow_star_function( void );
static void style_green_star_function( void );
static void style_red_long_function( void );
static void style_green_long_function( void );
static void style_blue_long_function( void );

static leds_effect_t leds_effect[] = {
  [FLOW_WATER]          = {20,  style_water_function},
  [STAR]                = {30,  style_star_function},
  [DOUBLE_STAR]         = {30,  style_doublestar_function},
  [SHINE]               = {450, style_shine_function},
  [BREATH_PURPLE]       = {100, style_breath_function},
  [YELLOW_BEAT]         = {450, style_yellow_beat_function},
  [WATER_GREEN]         = {20,  style_water_green_function},
  [BREATH_COLORFUL]     = {100, style_breath_colorful_function},
  [RED_BEAT]            = {400, style_red_beat_function},
  [RED_STAR]            = {30, style_red_star_function},
  [YELLOW_STAR]         = {30, style_yellow_star_function},
  [GREEN_STAR]          = {30, style_green_star_function},
  [RED_LONG]            = {30, style_red_long_function},
  [GREEN_LONG]          = {30, style_green_long_function},
  [BLUE_LONG]           = {30, style_blue_long_function},
};

static serial_leds_t serialLeds[] = {
  {LIGHTS_MODE_DEFAULT, BASIC_LIGHTS_EFFECT_METEOR, &leds_effect[STAR]},
  {LIGHTS_MODE_WELCOME, BASIC_LIGHTS_EFFECT_SHINE, &leds_effect[SHINE]},
  {LIGHTS_MODE_GOODEBYE, BASIC_LIGHTS_EFFECT_BREATH_COLORFUL, &leds_effect[BREATH_COLORFUL]},
  {LIGHTS_MODE_IDLE, BASIC_LIGHTS_EFFECT_METEOR, &leds_effect[STAR]},
  {LIGHTS_MODE_DANCE, DANCE_LIGHTS_EFFECT_SONG1},
  {LIGHTS_MODE_WARMING, BASIC_LIGHTS_EFFECT_RED_BEAT, &leds_effect[RED_BEAT]},
  {LIGHTS_MODE_COM_FAULT, BASIC_LIGHTS_EFFECT_FLOW, &leds_effect[FLOW_WATER]},
  {LIGHTS_MODE_BARRIER, BASIC_LIGHTS_EFFECT_YELLOW_BEAT, &leds_effect[YELLOW_BEAT]},
  {LIGHTS_MODE_SOLICIT, BASIC_LIGHTS_EFFECT_SHINE, &leds_effect[SHINE]},
  {LIGHTS_MODE_FREEDOM, BASIC_LIGHTS_EFFECT_BREATH_PURPLE, &leds_effect[BREATH_PURPLE]},
  {LIGHTS_MODE_IS_CHARGING, BASIC_LIGHTS_EFFECT_RED_METEOR, &leds_effect[RED_STAR]},
  {LIGHTS_MODE_CHARGE_TO_ON, BASIC_LIGHTS_EFFECT_YELLOW_METEOR, &leds_effect[YELLOW_STAR]},
  {LIGHTS_MODE_CHARGE_FINISH, BASIC_LIGHTS_EFFECT_GREEN_METEOR, &leds_effect[GREEN_STAR]},
};

typedef struct _songLightsRhythmControl_t {
  uint8_t                               songList;
  uint8_t                               danceIndex;
  uint32_t                              changeEffectTimeCount;
  leds_effect_t                         leds_effect;
} songLightsRhythmControl_t;

typedef void (*isNeedChangeStyleThenChangeFn)(songLightsRhythmControl_t *);
typedef struct _songLightsRhythmRoutine_t {
  uint8_t                               danceFlag;
#define                         STOP_DANCE              0x00
#define                         START_DANCE             0x01
  songLightsRhythmControl_t             *RhythmControl;
  isNeedChangeStyleThenChangeFn         isNeedChangeStyleThenChange;
} songLightsRhythmRoutine_t;

static songLightsRhythmControl_t        RhythmControl;
static songLightsRhythmRoutine_t        songLightsRhythmRoutine;
static songLightsRhythmControl_t        *pRhythmControl = &RhythmControl;
static songLightsRhythmRoutine_t        *pSongLightsRhythmRoutine = &songLightsRhythmRoutine;
static void isNeedChangeStyleThenChange( songLightsRhythmControl_t *Rhythm );

OSStatus SerialLeds_Init( void )
{ 
  OSStatus err = kNoErr;
  
  platform_pin_config_t pin_config;
  pin_config.gpio_speed = GPIO_SPEED_HIGH;
  pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
  pin_config.gpio_pull = GPIO_PULLUP;
  MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_LED_PWM, &pin_config );
  MicoGpioOutputLow( (mico_gpio_t)MICO_GPIO_LED_PWM );
  
  if( serial_leds == NULL )
  {
    serial_leds = &serialLeds[LIGHTS_MODE_DEFAULT];
  }

  pRhythmControl->songList = SONG1;

  pSongLightsRhythmRoutine->danceFlag = STOP_DANCE;
  pSongLightsRhythmRoutine->RhythmControl = pRhythmControl;
  pSongLightsRhythmRoutine->isNeedChangeStyleThenChange = isNeedChangeStyleThenChange;
  
  serial_leds_log("serial leds init success!");
  return err;
}

static void write_0(void)
{
        LED = 1;
	delay_300ns();
        LED = 0;
        delay_300ns();
        delay_600ns();
}

static void write_1(void)
{
        LED = 1;
	delay_600ns();
        LED = 0;
        delay_600ns();
}

static void write_RESET(void)
{
	LED = 0;
	delay_us(80);	
}

static void write_24bit(uint32_t word)
{
	uint8_t i;
	uint8_t R;
	uint8_t G;
	uint8_t B;
	uint32_t RGB;
	
	R = (word >> 8) & 0xff;
	G = (word >> 16) & 0xff;
	B = (word >> 0) & 0xff;
	
	RGB = (R << 16)|(G << 8)|(B << 0);
	
	for(i=0;i<24;i++)
	{

		if((RGB & 0x800000) == 0)
		{
			write_0();
		}
		else
		{
			write_1();
		}			
		
		RGB <<= 1;

	}
}

static void reset_led(void)
{
//	uint8_t i = TOTAL+10;
	uint8_t i = TOTAL;
	
	write_RESET();
	while(i--)
	{
 		write_24bit(0x000000);	
	}
	
	write_RESET();
}

static uint32_t change_led(uint32_t word,uint8_t level)
{
	static uint32_t rt_word;
	uint8_t R;
	uint8_t G;
	uint8_t B;
	
	R = (word >> 16) & 0xff;
	G = (word >> 8) & 0xff;
	B = (word >> 0) & 0xff;
	
	rt_word = ((R*level/LEVEL) << 16) | ((G*level/LEVEL) << 8) | ((B*level/LEVEL) << 0);
	return rt_word;
	
}

void single_color_water_led(uint32_t color,uint8_t times)
{
	uint8_t i = 0;
	uint8_t j;
	
	while(times)
	{
		for(j=0;j<TOTAL;j++)
		{
			write_24bit(change_led(color,(j%(LEVEL+1)+i)%(LEVEL+1)));			
		}
		
		write_RESET();		
		delay_ms(5);
		i = (i+1)%LEVEL;
		if(i == (LEVEL - 1))
		{
			times--;
		}
	}
}

/*******************************************************************/

static void send_data(void)
{
	uint8_t i = TOTAL;
	
	while(i--)
	{
		write_24bit(buff[i]);
	}
}

static void style_water_function(void)
{
	static uint8_t i = 0;
	uint8_t j = TOTAL;
	uint32_t color = 0x44333e;//0xcc99bb;

	for(j=0;j<TOTAL;j++)
	{
		buff[j] = change_led(color,(j%(LEVEL+1)+i)%(LEVEL+1));			
	}
		
        write_RESET();		
        i = (i+1)%LEVEL;
}

static void style_water_green_function(void)
{
	static uint8_t i = 0;
	uint8_t j = TOTAL;
	uint32_t color = 0x295400;//0x7cfc00;

	for(j=0;j<TOTAL;j++)
	{
		buff[j] = change_led(color,(j%(LEVEL+1)+i)%(LEVEL+1));			
	}
		
        write_RESET();		
        i = (i+1)%LEVEL;
}

static void style_star_function(void)
{
	uint8_t i = 0;
	uint8_t j;
	static uint8_t k;
	
	uint8_t level = 20;
	uint32_t color = 0x005544;//0x00ffcc;

	for(i=0;i<TOTAL;i++)
	{
		if(k<TOTAL-level)
		{
			if((i<k)||(i>(k+level)))
			{
				buff[i] = 0;
				continue;
			}
		}
		else
		{
			if((i<k)&&(i>(k+level)%TOTAL))
			{
				buff[i] = 0;
				continue;
			}
		}
		for(j=0;j<level;j++)
		{
			buff[(k+j)%TOTAL] = change_led(color,j%(level+1));			
		}
	}

	write_RESET();		
	k=(k+2)%TOTAL;
}

static void style_doublestar_function(void)
{
	uint8_t i = 0;
	uint8_t j;
	static uint8_t t;
	
	uint8_t level = 20;
	uint8_t space = 10;
	uint32_t color1 = 0x005544;//0x00ffcc;
	uint32_t color2 = 0x330049;//0x9900dd;

	for(i=0;i<TOTAL;i++)
	{		
		if(t<(TOTAL-(2*level+space)))
		{
			if((i<t)||(i>(t+2*level+space)))
			{
				buff[i] = 0;
				continue;
			}
		}
		else
		{
			if((i<t)&&(i>(t+2*level+space)%TOTAL))
			{
				buff[i] = 0;
				continue;
			}
		}
		for(j=0;j<2*level+space;j++)
		{
			if(j<level)
			{
				buff[(j+t)%TOTAL] = change_led(color1,j%(level+1));				
			}
			else if(j<level+space)
			{
				buff[(j+t)%TOTAL] = 0;				
			}
			else
			{
				buff[(j+t)%TOTAL] = change_led(color2,(j-level-space)%(level+1));				
			}
		}
	}

	write_RESET();		
	t=(t+2)%TOTAL;
}

#define  color_num      4
static void style_shine_function(void)
{
	static uint8_t i = 0;
	uint8_t j = TOTAL;
	uint8_t width = 10;
//	const uint8_t color_num = 4;
	uint32_t color[color_num] = {0x443300,0x005544,0x44333e,0x330049};//{0xdd9900,0x00ffcc,0xcc99bb,0x9900dd};

	for(j=0;j<TOTAL;j++)
	{
		if(j%(color_num*width) < 10)
		{
			buff[j] = color[i];
		}		

		else if(j%(color_num*width) < 20)	
		{
			buff[j] = color[(i+1)%4];
		}			
		
		else if(j%(color_num*width) < 30)			
		{
			buff[j] = color[(i+2)%4];
		}		

		else
		{
			buff[j] = color[(i+3)%4];			
		}			
	}
		
        write_RESET();		
        i = (i+1)%4;
}

static void style_breath_function(void)
{
	static uint8_t i = 0;
	uint8_t j = TOTAL;
	uint32_t color = 0x331144;//0x9a32cd;
	for(j=0;j<TOTAL;j++)
	{
		buff[j] = change_led(color,(i%(LEVEL-2)));			
	}
		
        write_RESET();		
        i = (i+2)%(LEVEL-2);
}

static void style_yellow_beat_function(void)
{
	static uint8_t i = 0;
	uint8_t j = TOTAL;
	uint32_t color = 0x4f4f00;//0xeeee00;
	for(j=0;j<TOTAL;j++)
	{
		buff[j] = change_led(color,(LEVEL-2));	
		if(i > 0)
		{
			buff[j] = 0;
		}
	}
		
        write_RESET();		
        i = (i+1)%2;
}

static void style_red_beat_function(void)
{
	static uint8_t i = 0;
	uint8_t j = TOTAL;
	uint32_t color = 0x350505;//0xA00e0e;
	for(j=0;j<TOTAL;j++)
	{
		buff[j] = change_led(color,(LEVEL-2));	
		if(i > 0)
		{
			buff[j] = 0;
		}
	}
		
        write_RESET();		
        i = (i+1)%2;
}

static void style_breath_colorful_function(void)
{
	static uint8_t i = 0;
	static uint8_t x = 0;
	uint8_t j = TOTAL;
	uint32_t color[7] = {0x440c0c,0x4f2700,0x443104,0x294429,0x243744,0x172b3c,0x341055};//{0xCD2626,0xEE7600,0xCD950C,0x7CCD7C,0x6CA6CD,0x4682B4,0x9B30FF};
	for(j=0;j<TOTAL;j++)
	{
		buff[j] = change_led(color[x],(i%(LEVEL-2)));			
	}
		
        write_RESET();		
        i = (i+1)%(LEVEL-2);
        if(i == LEVEL - 3)
        {
                x = (x+1)%7;
        }
}

#define CHARGE_LIGHTS_RED             0x880000
#define CHARGE_LIGHTS_YELLOW          0x4A4A00
#define CHARGE_LIGHTS_GREEN           0x008800

static void style_red_star_function( void )
{
	uint8_t i = 0;
	uint8_t j;
	static uint8_t k;
	
	uint8_t level = 8;
        uint32_t color = 0x880000;
	for(i=0;i<TOTAL;i++)
	{
		if(k<TOTAL-level)
		{
			if((i<k)||(i>(k+level)))
			{
				buff[i] = 0;
				continue;
			}
		}
		else
		{
			if((i<k)&&(i>(k+level)%TOTAL))
			{
				buff[i] = 0;
				continue;
			}
		}
		for(j=0;j<level;j++)
		{
			buff[(k+j)%TOTAL] = change_led(color,j%(level+1));			
		}
	}

	write_RESET();		
	k=(k+2)%TOTAL;
}
static void style_yellow_star_function( void )
{
	uint8_t i = 0;
	uint8_t j;
	static uint8_t k;
	
	uint8_t level = 8;
        uint32_t color = 0x4A4A00;
	for(i=0;i<TOTAL;i++)
	{
		if(k<TOTAL-level)
		{
			if((i<k)||(i>(k+level)))
			{
				buff[i] = 0;
				continue;
			}
		}
		else
		{
			if((i<k)&&(i>(k+level)%TOTAL))
			{
				buff[i] = 0;
				continue;
			}
		}
		for(j=0;j<level;j++)
		{
			buff[(k+j)%TOTAL] = change_led(color,j%(level+1));			
		}
	}

	write_RESET();		
	k=(k+2)%TOTAL;
}
static void style_green_star_function( void )
{
	uint8_t i = 0;
	uint8_t j;
	static uint8_t k;
	
	uint8_t level = 8;
        uint32_t color = 0x008800;
	for(i=0;i<TOTAL;i++)
	{
		if(k<TOTAL-level)
		{
			if((i<k)||(i>(k+level)))
			{
				buff[i] = 0;
				continue;
			}
		}
		else
		{
			if((i<k)&&(i>(k+level)%TOTAL))
			{
				buff[i] = 0;
				continue;
			}
		}
		for(j=0;j<level;j++)
		{
			buff[(k+j)%TOTAL] = change_led(color,j%(level+1));			
		}
	}

	write_RESET();		
	k=(k+2)%TOTAL;
}


static void style_red_long_function( void )
{
	uint8_t i = 0;
	
	uint8_t level = 8;
    uint32_t color = 0xFF0000;
	for(i=0;i<TOTAL;i++)
    {
	   buff[i] = change_led(color,level);			
    }
	write_RESET();		
}

static void style_green_long_function( void )
{
	uint8_t i = 0;
	
	uint8_t level = 8;
    uint32_t color = 0x00FF00;
	for(i=0;i<TOTAL;i++)
    {
	   buff[i] = change_led(color,level);			
    }
	write_RESET();		
}

static void style_blue_long_function( void )
{
	uint8_t i = 0;
	
	uint8_t level = 8;
    uint32_t color = 0x0000FF;
	for(i=0;i<TOTAL;i++)
    {
	   buff[i] = change_led(color,level);			
    }
	write_RESET();		
}

#if 0
void test_nsDelay(void)
{
  while(1)
  {
        write_0();
        write_1();
        write_0();
        write_1();
        write_0();
        write_1();
        write_0();
        write_1();
        write_0();
        write_1();
  }
}
#endif

void setSerialLedsEffect( lightsMode_t lightsMode, uint16_t effect )
{
  if( lightsMode < LEDS_MODES_N )
  {
    if( (lightsMode == LIGHTS_MODE_DANCE) && (effect <= 0x00FF) )
      return;
    if( ((lightsMode != LIGHTS_MODE_DANCE) && (lightsMode != LIGHTS_MODE_FREEDOM)) && (effect >= 0xFF00) )
      return;
    
    serialLeds[lightsMode].effectType = effect;
    switch( effect )
    {
    case LIGHTS_EFFECT_DEFAULT:
      return;
    case BASIC_LIGHTS_EFFECT_FLOW:
      serialLeds[lightsMode].leds_effect = &leds_effect[FLOW_WATER];
      break;
    case BASIC_LIGHTS_EFFECT_METEOR:
      serialLeds[lightsMode].leds_effect = &leds_effect[STAR];
      break;
    case BASIC_LIGHTS_EFFECT_DOUBLE_METEOR:
      serialLeds[lightsMode].leds_effect = &leds_effect[DOUBLE_STAR];
      break;
    case BASIC_LIGHTS_EFFECT_SHINE:
      serialLeds[lightsMode].leds_effect = &leds_effect[SHINE];
      break;
    case BASIC_LIGHTS_EFFECT_BREATH_PURPLE:
      serialLeds[lightsMode].leds_effect = &leds_effect[BREATH_PURPLE];
      break;
    case BASIC_LIGHTS_EFFECT_YELLOW_BEAT:
      serialLeds[lightsMode].leds_effect = &leds_effect[YELLOW_BEAT];
      break;
    case BASIC_LIGHTS_EFFECT_WATER_GREEN:
      serialLeds[lightsMode].leds_effect = &leds_effect[WATER_GREEN];
      break;
    case BASIC_LIGHTS_EFFECT_BREATH_COLORFUL:
      serialLeds[lightsMode].leds_effect = &leds_effect[BREATH_COLORFUL];
      break;
    case BASIC_LIGHTS_EFFECT_RED_BEAT:
      serialLeds[lightsMode].leds_effect = &leds_effect[RED_BEAT];
      break;
    case BASIC_LIGHTS_EFFECT_RED_METEOR:
      serialLeds[lightsMode].leds_effect = &leds_effect[RED_STAR];
      break;
    case BASIC_LIGHTS_EFFECT_YELLOW_METEOR:
      serialLeds[lightsMode].leds_effect = &leds_effect[YELLOW_STAR];
      break;
    case BASIC_LIGHTS_EFFECT_GREEN_METEOR:
      serialLeds[lightsMode].leds_effect = &leds_effect[GREEN_STAR];
      break;
    case BASIC_LIGHTS_EFFECT_RED_LONG:
      serialLeds[lightsMode].leds_effect = &leds_effect[RED_LONG];
      break;
    case BASIC_LIGHTS_EFFECT_GREEN_LONG:
      serialLeds[lightsMode].leds_effect = &leds_effect[GREEN_LONG];
      break;
    case BASIC_LIGHTS_EFFECT_BLUE_LONG:
      serialLeds[lightsMode].leds_effect = &leds_effect[BLUE_LONG];
      break;
    case DANCE_LIGHTS_EFFECT_DEFAULT:
      serial_leds = &serialLeds[lightsMode];
      serial_leds->modeType = lightsMode;
      serial_leds->effectType = effect;
      pRhythmControl->songList = SONG1;
      setCurLedsMode(LIGHTS_MODE_DANCE);
      return;
    case DANCE_LIGHTS_EFFECT_SONG1:
      serial_leds = &serialLeds[lightsMode];
      serial_leds->modeType = lightsMode;
      serial_leds->effectType = effect;
      pRhythmControl->songList = SONG1;
      setCurLedsMode(LIGHTS_MODE_DANCE);
      return;
    case DANCE_LIGHTS_EFFECT_SONG2:
      serial_leds = &serialLeds[lightsMode];
      serial_leds->modeType = lightsMode;
      serial_leds->effectType = effect;
      pRhythmControl->songList = SONG2;
      setCurLedsMode(LIGHTS_MODE_DANCE);
      return;
    default:
      return;
    }
    
    if( LIGHTS_MODE_FREEDOM == lightsMode )
    {
      setCurLedsMode( LIGHTS_MODE_FREEDOM );
    }

//    setCurLedsMode( (lightsMode_t)lightsMode );
  }
}
#ifdef LIGHTS_DEBUG
const static uint8_t debugLigthsChar[LIGHTS_MODE_MAX][30] = {
  [LIGHTS_MODE_DEFAULT]         = {"lights start default mode"},
  [LIGHTS_MODE_WELCOME]         = {"lights start welcome mode"},
  [LIGHTS_MODE_GOODEBYE]        = {"lights start goodbye mode"},
  [LIGHTS_MODE_IDLE]            = {"lights start idle mode"},
  [LIGHTS_MODE_DANCE]           = {"lights start dance mode"},
  [LIGHTS_MODE_WARMING]         = {"lights start warming mode"},
  [LIGHTS_MODE_COM_FAULT]       = {"lights start com fault mode"},
  [LIGHTS_MODE_BARRIER]         = {"lights start barrier mode"},
  [LIGHTS_MODE_SOLICIT]         = {"lights start solicit mode"},
  [LIGHTS_MODE_FREEDOM]         = {"lights start freedom mode"},
  [LIGHTS_MODE_IS_CHARGING]     = {"lights start charging mode"},
};
#endif

void setCurLedsMode( lightsMode_t lightsMode )
{
  if( boardStatus->sysStatus & (uint16_t)STATE_IS_CHARGER_IN )
  {
    return;
  }
  reset_led();
#ifdef LIGHTS_DEBUG
  serial_leds_log("%s", debugLigthsChar[lightsMode] );
#endif
  if( lightsMode == LIGHTS_MODE_DANCE )
  {
    startDanceLedsMode();
    serial_leds_log("is song No.%d",pRhythmControl->songList);
  }
  else
  {
    if( pSongLightsRhythmRoutine->danceFlag == START_DANCE )
    {
      stopDanceLedsMode();
    }
    serial_leds = &serialLeds[lightsMode];
  }
}
#ifdef LIGHTS_DEBUG
static uint32_t testDanceTimeTag;
#endif
static void isNeedChangeStyleThenChange( songLightsRhythmControl_t *Rhythm )
{
  Rhythm->changeEffectTimeCount ++;
  if( Rhythm->changeEffectTimeCount >= music[Rhythm->songList].time[Rhythm->danceIndex]*100/SERIAL_LEDS_PERIOD )
  {
    Rhythm->changeEffectTimeCount = 0;
    if( music[Rhythm->songList].type[Rhythm->danceIndex] == 0 )
    {
      serial_leds = &serialLeds[LIGHTS_MODE_FREEDOM];
#ifdef LIGHTS_DEBUG
      testDanceTimeTag = os_get_time();
      serial_leds_log("[%d]ms song start play", (os_get_time() - testDanceTimeTag)*SYSTICK_PERIOD);
#endif
    }
    Rhythm->leds_effect.freshSerialLedshandle = leds_effect[music[Rhythm->songList].type[Rhythm->danceIndex + 1]].freshSerialLedshandle;
    Rhythm->leds_effect.freshTime = music[Rhythm->songList].rate[Rhythm->danceIndex + 1];
    serialLeds[LIGHTS_MODE_FREEDOM].leds_effect = &Rhythm->leds_effect;
//    serial_leds = &serialLeds[LIGHTS_MODE_FREEDOM];
#ifdef LIGHTS_DEBUG
    serial_leds_log("[%d]ms change style", (os_get_time() - testDanceTimeTag)*SYSTICK_PERIOD);
#endif
    Rhythm->danceIndex += 1;
    if( Rhythm->danceIndex >= music[Rhythm->songList].indexMax )
    {
      Rhythm->danceIndex = 0;
      setCurLedsMode( LIGHTS_MODE_IDLE );
    }
  }
}

void startDanceLedsMode( void )
{
  pSongLightsRhythmRoutine->RhythmControl->changeEffectTimeCount = 0;
  pSongLightsRhythmRoutine->RhythmControl->danceIndex = 0;
  pSongLightsRhythmRoutine->danceFlag = START_DANCE;
}

void stopDanceLedsMode( void )
{
  pSongLightsRhythmRoutine->danceFlag = STOP_DANCE;
  serial_leds_log("lights stop dance mode");
}

static uint8_t  tickTimerCount = 0;
static uint32_t    indicatorFreshCount = 0;

// period: 10ms
void serialLedsTick( void )
{
  indicatorFreshCount ++;
  if( indicatorFreshCount > 50 )
  {
    indicatorFreshCount = 0;
    MicoGpioOutputTrigger( MICO_GPIO_SYS_LED );
  }

//  if( (DEVICES_ON == boardStatus->devicesOnOff) || (NO == boardStatus->isPowerOnFinish)\
    || (NO == boardStatus->isPowerOffFinish) )
  if( serial_leds != NULL && STATE_POWER_OFF != (boardStatus->sysStatus & STATE_RUN_BITS) && !boardStatus->isUpgrading )
  {
    tickTimerCount ++;
    if( START_DANCE == pSongLightsRhythmRoutine->danceFlag )
    {
      pSongLightsRhythmRoutine->isNeedChangeStyleThenChange(pSongLightsRhythmRoutine->RhythmControl);
    }
    if( tickTimerCount >= serial_leds->leds_effect->freshTime/SERIAL_LEDS_PERIOD )
    {
      tickTimerCount = 0;
      if( serial_leds->leds_effect->freshSerialLedshandle != NULL )
      {
        serial_leds->leds_effect->freshSerialLedshandle();
      }
      DISABLE_INTERRUPTS();
      send_data();
      ENABLE_INTERRUPTS();
    }
  }
}
/*********************END OF FILE**************/
