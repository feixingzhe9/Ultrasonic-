#include "mico.h"
#include "platform.h"
#include "platform_internal.h"
#include "platform_config.h"
#include "board_init.h"
#include "protocol.h"
#include "app_platform.h"
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
int main( void )
{
  init_clocks();
  init_architecture();
  init_platform();

  os_PowerBoard_log( "System clock = %d Hz",HAL_RCC_GetHCLKFreq() );
  printf ( menu, MODEL, SW_VERSION, HARDWARE_REVISION );
  bsp_Init();
  Platform_Init();
  CanLongBufInit();
  MicoCanInitialize( MICO_CAN1 );
  
  delay_ms(10);
  UltraSonicInit();
  delay_ms(10);
  

  for(;;)
  { 
    can_protocol_period(); 

    UltraSonicDataTick();
    SysLed();
#ifdef HOMWEE_TEST 
    EmergencyStopTest();
#endif
  }
}

#define ULTRASONIC_SEND_TIME   200/SYSTICK_PERIOD
void UltraSonicStartTick(void) 
{
    static uint32_t start_time_1 = 0;
    static uint32_t start_time2 = 0;
    static uint8_t flag = 0;
    
    //V24OutputHigh();//

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
  
    platform_pin_config_t pin_config;
    pin_config.gpio_speed = GPIO_SPEED_MEDIUM;
    pin_config.gpio_mode = GPIO_MODE_OUTPUT_PP;// GPIO_MODE_AF_PP;// 
    pin_config.gpio_pull = GPIO_PULLUP;

    
    
    //  Initialise system led
    MicoGpioInitialize( (mico_gpio_t)MICO_GPIO_EMG_STOP, &pin_config );
    
    if(os_get_time() - emg_test_start_time >= EMG_TEST_PERIOD)
    {
        MicoGpioOutputTrigger(MICO_GPIO_EMG_STOP);
        emg_test_start_time = os_get_time();
    }
}
#endif
