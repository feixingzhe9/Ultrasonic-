#include "mico.h"
#include "platform.h"
#include "platform_internal.h"
#include "platform_config.h"
#include "board_init.h"

#include "upgrade_flash.h"
#include "can_protocol.h"
#include "UltraSonic.h"
#include "platform_tim.h"
#define ultrasonic_log(format, ...)  custom_log("Ultrasonic", format, ##__VA_ARGS__)
#define Application_REVISION "v0.1"

const  char menu[] =
"\r\n"
"Application for %s,\r\nSOFTWARE_VERSION: %s,\r\nHARDWARE_REVISION: %s\r\n";


//#define HOMWEE_TEST

void sys_indicator(void);
void ultrasonic_start_tick(void);
#ifdef HOMWEE_TEST
void emergency_stop_test(void);
#endif

#ifdef OPEN_WATCH_DOG
static void init_iwdg(uint32_t period);
static void start_iwdg(void);
#endif

void feed_dog(void);

IWDG_HandleTypeDef iwdg_handle;

int main( void )
{
    init_clocks();
    init_architecture();
    init_platform();

    ultrasonic_log( "System clock = %d Hz",HAL_RCC_GetHCLKFreq() );
    printf ( menu, MODEL, SW_VERSION, HARDWARE_REVISION );
    board_gpios_init();

    can_long_buf_init();
    init_can(MICO_CAN1);

    delay_ms(10);
    ultrasonic_init();
    delay_ms(10);

#ifdef OPEN_WATCH_DOG
    init_iwdg(550);
    start_iwdg();
#endif

    for(;;)
    {
        can_protocol_period();
        //ultrasonic_start_tick();
        ultrasonic_data_tick();
        sys_indicator();

#ifdef HOMWEE_TEST
        emergency_stop_test();
#endif

#ifdef OPEN_WATCH_DOG
        feed_dog();
#endif
    }
}

#define ULTRASONIC_SEND_TIME   100/SYSTICK_PERIOD
void ultrasonic_start_tick(void)
{
    static uint32_t start_time_1 = 0;
    static uint32_t start_time2 = 0;
    static uint8_t flag = 0;

    if(os_get_time() - start_time_1 >= ULTRASONIC_SEND_TIME)
    {
        ultrasonic_start();
        start_time_1 = os_get_time();
        start_time2 = os_get_time();
        flag = 1;
    }

    if((os_get_time() - start_time2 >= ULTRASONIC_SEND_TIME/2) && (flag == 1))
    {
        show_test_log();
        flag = 0;
    }
}

#define TEST_PERID      500/SYSTICK_PERIOD
uint32_t sys_led_start_time = 0;
void sys_indicator(void)
{
    static uint32_t cnt = 0;

    platform_pin_config_t pin_config;
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;// GPIO_MODE_AF_PP;//
    pin_config.gpio_pull = GPIO_PULLUP;

    //  Initialise system led
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_SYS_LED, &pin_config );

    if(os_get_time() - sys_led_start_time >= TEST_PERID)
    {
        cnt++;
        MicoGpioOutputTrigger(MICO_GPIO_SYS_LED);
        sys_led_start_time = os_get_time();
    }
    if(cnt % 2 == 1)
    {
        MicoGpioOutputLow(MICO_GPIO_SYS_LED);
    }
    else
    {
        MicoGpioOutputHigh(MICO_GPIO_SYS_LED);
    }
}


#ifdef HOMWEE_TEST

#define EMG_TEST_PERIOD     1000/SYSTICK_PERIOD
static uint32_t emg_test_start_time = 0;

void emergency_stop_test(void)
{

    //platform_pin_config_t pin_config;
    //pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    //pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;// GPIO_MODE_AF_PP;//
    //pin_config.gpio_pull = GPIO_PULLUP;

    //  Initialise system led
    //MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_EMG_STOP, &pin_config );

    if(os_get_time() - emg_test_start_time >= EMG_TEST_PERIOD)
    {
        MicoGpioOutputTrigger(MICO_GPIO_EMG_STOP);
        emg_test_start_time = os_get_time();
    }
}
#endif


#ifdef OPEN_WATCH_DOG
/* IWDG init function */
static void init_iwdg(uint32_t period)
{

    iwdg_handle.Instance = IWDG;
    iwdg_handle.Init.Prescaler = IWDG_PRESCALER_32;
    iwdg_handle.Init.Reload = period;
    if (HAL_IWDG_Init(&iwdg_handle) != HAL_OK)
    {
        //_Error_Handler(__FILE__, __LINE__);
        printf("error");
    }
}

static void start_iwdg(void)
{
    HAL_IWDG_Start(&iwdg_handle);
}
#endif

#define FEED_DOG_PERIOD     50/SYSTICK_PERIOD
void feed_dog(void)
{
    static uint32_t feed_dog_start_time = 0;
    if(os_get_time() - feed_dog_start_time > FEED_DOG_PERIOD)
    {
        HAL_IWDG_Refresh(&iwdg_handle);
        feed_dog_start_time = os_get_time();
    }
}
