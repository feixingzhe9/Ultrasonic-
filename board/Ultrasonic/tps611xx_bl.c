/**
 ******************************************************************************
 * @file    tps611xx_bl.c
 * @author  Adam Huang
 * @version V1.0.0
 * @date    29-Nov-2016
 * @brief   
 ******************************************************************************
*/

#include "tps611xx_bl.h"
#include "backlight.h"
#include "platform_peripheral.h"
#include "platform_mcu_peripheral.h"

#if (defined TPS_CONTROL_MODE) && (TPS_CONTROL_MODE == EASY_SCALE)

#define dev_err         printf
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
static inline void __udelay( unsigned long x )
{
  while( x-- )   
  {
    asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");asm("nop");asm("nop");
    asm("nop");asm("nop");
  }
}
                                
#ifndef ndelay
static inline void __ndelay( unsigned long x )
{
  unsigned long fixed_x;
  fixed_x = ( unsigned long )(x/1800.0);
  while(fixed_x--)
  {
   __asm("nop");
  }
}
#define ndelay(n) __ndelay((unsigned long long) (n))

void delay_nus(unsigned long long time)
{    
   unsigned long long i=0;  
   while(time--)
   {
      i=4;  //自己定义
      while(i--) ;    
   }
}
#define udelay(n) delay_nus(n)//__udelay((unsigned long long) (n))
#define mdelay(n) __udelay((unsigned long long) (n) * 1000)
#endif
//#if (defined TPS_CONTROL_MODE) && (TPS_CONTROL_MODE == EASY_SCALE)
#define CMD_FORWARD 0
#define CMD_BACKWARD 1

enum tps611xx_id {
  /*
	TPS61158_ID = 0,
	TPS61161_ID,
	TPS61163_ID,
  */
	TPS61165_ID = 0,
};

/*
 * easyscale time spec
 *  <at> es_delay : es delay time(us)
 *  <at> es_det   : es detection time(us)
 *  <at> start    : start time of data stream(us)
 *  <at> eos      : end time of data stream(us)
 *  <at> reset    : ic shutdown time(ms)
 *  <at> logic_1_low : low time high bit(us)
 *  <at> logic_0_low : low time low bit(us)
 *  <at> ackn        : duation of ack condistion(us)
 *  <at> ack_poll    : ack polling duration(us)
 */
struct tps611xx_time {
	unsigned int es_delay;
	unsigned int es_det;
	unsigned int start;
	unsigned int eos;
	unsigned int reset;
	unsigned int logic_1_low;
	unsigned int logic_0_low;
	unsigned int ackn;
	unsigned int ack_poll;
};

/*
 *  <at> seq : sequence of data transfer
 *  <at> size: size of data
 *  <at> brt_max : max brightness
 *  <at> brt_bmask : bit mask of dimming bits
 *  <at> rfa_bmask : bit mask of RFA(Request for Acknowledge condition)
 */
struct tps611xx_command {
	int seq;
	int size;
	int brt_max;
	int brt_bmask;
	int rfa_bmask;
};

/*
 *  <at> id : product id
 *  <at> name : product name
 *  <at> addr : device address
 *  <at> cmd  : es command info
 *  <at> time : es time info
 */
struct tps611xx_esdata {
	enum tps611xx_id id;
	char *name;
	int addr;
	struct tps611xx_command cmd;
	struct tps611xx_time time;
};

static const struct tps611xx_esdata tps611xx_info[] = {
  /*
	[TPS61158_ID] = {
			 .id = TPS61158_ID,
			 .name = "tps61158",
			 .addr = 0x5800,
			 .cmd = {
				 .seq = CMD_FORWARD,
				 .size = 16,
				 .brt_max = 31,
				 .brt_bmask = 0x1f,
				 .rfa_bmask = 0x80
				},
			 .time = {
				  .es_delay = 100000,
				  .es_det = 450000,
				  .start = 3500,
				  .eos = 3500,
				  .reset = 4,
				  .logic_1_low = 5000,
				  .logic_0_low = 15000,
				  .ackn = 900000,
				  .ack_poll = 2000
				},
			 },

	[TPS61161_ID] = {
			 .id = TPS61161_ID,
			 .name = "tps61161",
			 .addr = 0x7200,
			 .cmd = {
				 .seq = CMD_FORWARD,
				 .size = 16,
				 .brt_max = 31,
				 .brt_bmask = 0x1f,
				 .rfa_bmask = 0x80
				},
			 .time = {
				  .es_delay = 120000,
				  .es_det = 280000,
				  .start = 2000,
				  .eos = 2000,
				  .reset = 3,
				  .logic_1_low = 3000,
				  .logic_0_low = 7000,
				  .ackn = 512000,
				  .ack_poll = 2000
				},
			 },

	[TPS61163_ID] = {
			 .id = TPS61163_ID,
			 .name = "tps61163",
			 .addr = 0x8F0000,
			 .cmd = {
				 .seq = CMD_BACKWARD,
				 .size = 24,
				 .brt_max = 511,
				 .brt_bmask = 0x1ff,
				 .rfa_bmask = 0x400
				},
			 .time = {
				  .es_delay = 100000,
				  .es_det = 260000,
				  .start = 2000,
				  .eos = 2000,
				  .reset = 3,
				  .logic_1_low = 3000,
				  .logic_0_low = 7000,
				  .ackn = 512000,
				  .ack_poll = 2000
				},
			 },
                         */
	[TPS61165_ID] = {
			 .id = TPS61165_ID,
			 .name = "tps61165",
			 .addr = 0x7200,
			 .cmd = {
				 .seq = CMD_FORWARD,
				 .size = 16,
				 .brt_max = 31,
				 .brt_bmask = 0x1f,
				 .rfa_bmask = 0x80
				},
			 .time = {
				  .es_delay = 120,//120000,
				  .es_det = 280,//280000,
				  .start = 4,//4000,
				  .eos = 4,//4000,
				  .reset = 3,
				  .logic_1_low = 3,//3000,
				  .logic_0_low = 9,//7000,
				  .ackn = 512,//512000,
				  .ack_poll = 2,//2000
				},
			 },
};

extern const platform_gpio_t platform_gpio_pins[];

struct tps611xx_bl_data TPS61165_driver_data = {     
          .rfa_en = 0,
          .en_gpio = &platform_gpio_pins[MICO_GPIO_IRLED_PWM],
          .esdata = &tps611xx_info[0],
};

struct backlight_device bl_TPS61165 = {
  .dev_driver = &TPS61165_driver_data,
};

static int tps611xx_bl_update_status(struct backlight_device *bl)
{
	struct tps611xx_bl_data *pchip = bl_get_driver(bl);
	const struct tps611xx_esdata *esdata = pchip->esdata;
	int data_in, t_low, t_logic, max_bmask;

	data_in = esdata->addr | (bl->props.brightness & esdata->cmd.brt_bmask);
	if (pchip->rfa_en)
		data_in |= esdata->cmd.rfa_bmask;

	max_bmask = 0x1 << esdata->cmd.size;
	t_logic = esdata->time.logic_1_low + esdata->time.logic_0_low;

	DISABLE_INTERRUPTS();
	/* t_start : 2us high before data byte */
	platform_gpio_output_high( pchip->en_gpio );
	udelay(esdata->time.start);

	/* forward command transfer */
	if (esdata->cmd.seq == CMD_FORWARD) {
		int addr_bmask = max_bmask >> 8;

		for (max_bmask >>= 1; max_bmask > 0x0; max_bmask >>= 1) {
			if (data_in & max_bmask)
				t_low = esdata->time.logic_1_low;
			else
				t_low = esdata->time.logic_0_low;

			platform_gpio_output_low( pchip->en_gpio );
			udelay(t_low);
			platform_gpio_output_high( pchip->en_gpio );
			udelay(t_logic - t_low);

			if (max_bmask == addr_bmask) {
				platform_gpio_output_low( pchip->en_gpio );
				/* t_eos : low after address byte */
				udelay(esdata->time.eos);
				platform_gpio_output_high( pchip->en_gpio );
				/* t_start : high before data byte */
				udelay(esdata->time.start);
			}
		}
	} else {
		/* backward command tansfer */
		int bmask;

		for (bmask = 0x01; bmask < max_bmask; bmask <<= 1) {
			if (data_in & bmask)
				t_low = esdata->time.logic_1_low;
			else
				t_low = esdata->time.logic_0_low;

			platform_gpio_output_low( pchip->en_gpio );
			udelay(t_low);
			platform_gpio_output_high( pchip->en_gpio );
			udelay(t_logic - t_low);
		}
	}

	/*
	 * t_eos : low after address byte
	 * t_ackVal is also t_eos
	 */
	platform_gpio_output_low( pchip->en_gpio );
	udelay(esdata->time.eos);

	/* RFA management  */
	if (pchip->rfa_en) {
		int max_ack_time = esdata->time.ackn;
		/* set input */
		platform_pin_config_t pin_config;
                pin_config.gpio_speed = GPIO_SPEED_HIGH;
                pin_config.gpio_mode = GPIO_MODE_INPUT;
                pin_config.gpio_pull = GPIO_PULLUP;
                platform_gpio_init( pchip->en_gpio, &pin_config );
		/* read acknowledge from chip */
		while (max_ack_time > 0) {
			if (platform_gpio_input_get( pchip->en_gpio ) == 0)
				break;
			max_ack_time -= esdata->time.ack_poll;
		}
		if (max_ack_time <= 0)
                {
                        ENABLE_INTERRUPTS();
			dev_err("easyscale : no ack from %s\n",esdata->name);
                }
		else
                {
			udelay(max_ack_time);
                }
                pin_config.gpio_speed = GPIO_SPEED_HIGH;
                pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
                pin_config.gpio_pull = GPIO_PULLUP;
                platform_gpio_init( pchip->en_gpio, &pin_config );
	}
	platform_gpio_output_high( pchip->en_gpio );
	ENABLE_INTERRUPTS();

	return bl->props.brightness;
}

static int tps611xx_bl_get_brightness(struct backlight_device *bl)
{
	return bl->props.brightness;
}

static const struct backlight_ops tps611xx_bl_ops = {
	.update_status = tps611xx_bl_update_status,
	.get_brightness = tps611xx_bl_get_brightness,
};

static int tps611xx_backlight_probe(struct backlight_device *bl)
{
	struct tps611xx_bl_data *pchip = bl_get_driver(bl);
	struct backlight_properties props;
	const struct tps611xx_esdata *esdata;
	int ret;

        pchip->dev = bl->dev_driver->esdata->name;
	esdata = pchip->esdata;
        mico_rtos_init_mutex(bl->update_lock);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.brightness = esdata->cmd.brt_max;
	props.max_brightness = esdata->cmd.brt_max;
	props.type = BACKLIGHT_RAW;
        memcpy(&bl->props, &props, sizeof(struct backlight_properties));
        bl->ops = &tps611xx_bl_ops;
	/* EasyScale init */
	platform_pin_config_t pin_config;
        pin_config.gpio_speed = GPIO_SPEED_HIGH;
        pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
        pin_config.gpio_pull = GPIO_PULLUP;
        ret = platform_gpio_init( bl_TPS61165.dev_driver->en_gpio, &pin_config );
	if (ret) {
		dev_err("%s failed : get gpio\n", pchip->esdata->name);
		return ret;
	}
        
	/*
	 * ES Detection Window
	 *   - ES detect delay
	 *   - ES detect time
	 */
	DISABLE_INTERRUPTS();
	platform_gpio_output_high( pchip->en_gpio );
	//udelay(esdata->time.es_delay);
        udelay(103);
	platform_gpio_output_low( pchip->en_gpio );
	//udelay(esdata->time.es_det);
        udelay(300);
	platform_gpio_output_high( pchip->en_gpio );
	ENABLE_INTERRUPTS();
	dev_err("%s EasyScale is initialized\n", pchip->esdata->name);
	return 0;
}

static int tps611xx_backlight_remove(struct backlight_device *bl)
{
	struct tps611xx_bl_data *pchip = bl_get_driver(bl);
	const struct tps611xx_esdata *esdata = pchip->esdata;

	platform_gpio_output_low( pchip->en_gpio );
	mdelay(esdata->time.reset);
        dev_err("%s EasyScale was removed\n", pchip->esdata->name);
	return 0;
}

void startTps611xx( void )
{
  tps611xx_backlight_probe( &bl_TPS61165 );
  backlight_update_status( &bl_TPS61165 );
}

void brightness_dimming_by_duty( uint8_t duty )
{
  bl_TPS61165.props.brightness = (int)(duty/100.0*bl_TPS61165.props.max_brightness);
  stopTps611xx();
  tps611xx_backlight_probe( &bl_TPS61165 );
  backlight_update_status( &bl_TPS61165 );
}

void stopTps611xx( void )
{
  tps611xx_backlight_remove( &bl_TPS61165 );
}

#elif (defined TPS_CONTROL_MODE) && (TPS_CONTROL_MODE == PWM)

struct tps611xx_driver {
  uint8_t               initialized;
  uint8_t               lightness_percent;
  uint32_t              pwm_frequency;
};

static struct tps611xx_driver tps61151_driver = 
{
  .lightness_percent = 50,
  .pwm_frequency = 50000,
};
struct tps611xx_driver *pTps61151_driver = &tps61151_driver;

void startTps611xx( void )
{
  
  platform_pin_config_t pin_config;
  pin_config.gpio_speed = GPIO_SPEED_HIGH;
  pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
  pin_config.gpio_pull = GPIO_PULLUP;
  MicoGpioInitialize( (mico_gpio_t)MICO_PWM_IRLED, &pin_config );
  MicoGpioOutputLow( (mico_gpio_t)MICO_PWM_IRLED );

  if( !pTps61151_driver->initialized )
  {
    pTps61151_driver->initialized = 1;
    
    MicoPwmInitialize( MICO_PWM_IRLED, pTps61151_driver->pwm_frequency, pTps61151_driver->lightness_percent );
//    MicoPwmStart( MICO_PWM_IRLED );
  }
}

void brightness_dimming( uint32_t frequency, uint8_t duty )
{
#if 1
  stopTps611xx();
  pTps61151_driver->pwm_frequency = frequency;
  if( duty <= 100 )
  {
    pTps61151_driver->lightness_percent = duty;
  }
  startTps611xx();
#else
  MicoPwmSetDuty( MICO_PWM_IRLED, duty );
#endif
}

void stopTps611xx( void )
{
  if( pTps61151_driver->initialized )
  {
    MicoPwmStop( MICO_PWM_IRLED );
    pTps61151_driver->initialized = 0;
  }
  
  platform_pin_config_t pin_config;
  pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
  pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;
  pin_config.gpio_pull = GPIO_PULLUP;
  MicoGpioInitialize( (mico_gpio_t)MICO_PWM_IRLED, &pin_config );
  MicoGpioOutputLow( (mico_gpio_t)MICO_PWM_IRLED );
  
}

#endif