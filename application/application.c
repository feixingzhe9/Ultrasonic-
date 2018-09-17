#include "mico.h"
#include "platform.h"
#include "platform_internal.h"
#include "platform_config.h"
#include "board_init.h"

#include "upgrade_flash.h"
#include "can_protocol.h"
#include "UltraSonic.h"
#include "platform_tim.h"
#define os_PowerBoard_log(format, ...)  custom_log("Ultrasonic", format, ##__VA_ARGS__)
#define Application_REVISION "v0.1"

const  char menu[] =
"\r\n"
"Application for %s,\r\nSOFTWARE_VERSION: %s,\r\nHARDWARE_REVISION: %s\r\n";


//#define HOMWEE_TEST

void SysLed(void);
void UltraSonicStartTick(void);
#ifdef HOMWEE_TEST
void EmergencyStopTest(void);
#endif

#ifdef OPEN_WATCH_DOG
static void MX_IWDG_Init(uint32_t period);
#endif

void feed_dog(void);

IWDG_HandleTypeDef hiwdg;

int main( void )
{
    init_clocks();
    init_architecture();
    init_platform();

    os_PowerBoard_log( "System clock = %d Hz",HAL_RCC_GetHCLKFreq() );
    printf ( menu, MODEL, SW_VERSION, HARDWARE_REVISION );
    board_gpios_init();
    CanLongBufInit();
    MicoCanInitialize( MICO_CAN1 );

    delay_ms(10);
    UltraSonicInit();
    delay_ms(10);
#ifdef OPEN_WATCH_DOG
    MX_IWDG_Init(550);
    HAL_IWDG_Start(&hiwdg);
#endif

    extern void Ultra_IO_InputIT();
    for(;;)
    {
        can_protocol_period();
        //UltraSonicStartTick();
        UltraSonicDataTick();
        SysLed();
#ifdef HOMWEE_TEST
        EmergencyStopTest();
#endif
#ifdef OPEN_WATCH_DOG
        feed_dog();
#endif
    }
}

#define ULTRASONIC_SEND_TIME   100/SYSTICK_PERIOD
void UltraSonicStartTick(void)
{
    static uint32_t start_time_1 = 0;
    static uint32_t start_time2 = 0;
    static uint8_t flag = 0;

    if(os_get_time() - start_time_1 >= ULTRASONIC_SEND_TIME)
    {
        UltraSonicStart();
        start_time_1 = os_get_time();
        start_time2 = os_get_time();
        flag = 1;
    }

    if((os_get_time() - start_time2 >= ULTRASONIC_SEND_TIME/2) && (flag == 1))
    {
        ShowTestLog();
        flag = 0;
    }
}


#define TEST_PERID      500/SYSTICK_PERIOD
uint32_t sys_led_start_time = 0;
void SysLed(void)
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

void EmergencyStopTest(void)
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
static void MX_IWDG_Init(uint32_t period)
{

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
    hiwdg.Init.Reload = period;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
        //_Error_Handler(__FILE__, __LINE__);
        printf("error");
    }

}
#endif

#define FEED_DOG_PERIOD     50/SYSTICK_PERIOD
void feed_dog(void)
{
    static uint32_t feed_dog_start_time = 0;
    if(os_get_time() - feed_dog_start_time > FEED_DOG_PERIOD)
    {
        HAL_IWDG_Refresh(&hiwdg);
        feed_dog_start_time = os_get_time();
    }
}
