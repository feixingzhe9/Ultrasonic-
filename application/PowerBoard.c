


#include "mico.h"
#include "platform.h"
#include "platform_internal.h"
#include "platform_config.h"

#define os_PowerBoard_log(format, ...)  custom_log("PowerBoard", format, ##__VA_ARGS__)

extern void Main_Menu(void);
#define Application_REVISION "v2.1"

#ifdef SIZE_OPTIMIZE
const t char menu[] =
"\r\n"
"PowerBoard application for %s, %s, HARDWARE_REVISION: %s\r\n"
"0:BOOTUPDATE,"
"1:FWUPDATE,"
"2:DRIVERUPDAT,"
"3:PARAUPDATE,"
"4:FLASHUPDATE,"
"5:MEMORYMAP,"
"6:BOOT,"
"7:REBOOT";
#else
const char menu[] =
"\r\n"
"PowerBoard application for %s, %s, HARDWARE_REVISION: %s\r\n"
"+ command -------------------------+ function ------------+\r\n"
"| 0:BOOTUPDATE    <-r>             | Update bootloader    |\r\n"
"| 1:FWUPDATE      <-r>             | Update application   |\r\n"
"| 2:PARUPDATE     <-id n><-r><-e>  | Update MICO partition|\r\n"
"| 3:FLASHUPDATE   <-dev device>    |                      |\r\n"
"|  <-e><-r><-start addr><-end addr>| Update flash content |\r\n"
"| 4:MEMORYMAP                      | List flash memory map|\r\n"
"| 5:BOOT                           | Excute application   |\r\n"
"| 6:REBOOT                         | Reboot               |\r\n"
"+----------------------------------+----------------------+\r\n"
"|    (C) COPYRIGHT 2016 MUYE Corporation  By Driver Group |\r\n"
" Notes:\r\n"
" -e Erase only  -r Read from flash -dev flash device number\r\n"
"  -start flash start address -end flash start address\r\n"
" Example: Input \"4 -dev 0 -start 0x400 -end 0x800\": Update \r\n"
"          flash device 0 from 0x400 to 0x800\r\n";
#endif
void cli_main( void *data );

/* Private define ------------------------------------------------------------*/
typedef enum
{
  THREAD_1 = 0,
  THREAD_2
} Thread_TypeDef;
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
osThreadId LEDThread1Handle, LEDThread2Handle;
/* Private function prototypes -----------------------------------------------*/
static void LED_Thread1(void const *argument);
static void LED_Thread2(void const *argument);

int main( void )
{
  OSStatus err = kNoErr;
  init_architecture();
  //printf ( menu, MODEL, Application_REVISION, HARDWARE_REVISION );
//  err = mico_rtos_create_thread( NULL, 3, "cli", cli_main, 0x100, NULL );
//  require_noerr_action( err, exit, printf("ERROR: Unable to start the cli_main thread.") );

    /* Thread 1 definition */
  osThreadDef(THREAD_1, LED_Thread1, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  
  /*  Thread 2 definition */
  osThreadDef(THREAD_2, LED_Thread2, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
  
  /* Start thread 1 */
  LEDThread1Handle = osThreadCreate(osThread(THREAD_1), NULL);

  /* Start thread 2 */
  LEDThread2Handle = osThreadCreate(osThread(THREAD_2), NULL); 
  
  osKernelStart();
  for(;;)
  {
    //Main_Menu();    
  }
  goto exit;
exit:
  mico_rtos_delete_thread(NULL);
  return err;
}

void cli_main( void *data)
{
  for(;;)
  {
    Main_Menu();
  }
  //mico_rtos_delete_thread(NULL);
}

/**
  * @brief  Toggle LED2 thread
  * @param  thread not used
  * @retval None
  */
static void LED_Thread1(void const *argument)
{
  uint32_t count = 0;
  (void) argument;

  for (;;)
  {
    count = osKernelSysTick() + 5000;

    while (count >= osKernelSysTick())
    {
      MicoGpioOutputHigh( MICO_GPIO_SYS_LED );
      osDelay(80);
      MicoGpioOutputLow( MICO_GPIO_SYS_LED );
      osDelay(80);
      MicoGpioOutputHigh( MICO_GPIO_SYS_LED );
      osDelay(80);
      MicoGpioOutputLow( MICO_GPIO_SYS_LED );
      osDelay(80);
      MicoGpioOutputHigh( MICO_GPIO_SYS_LED );
      osDelay(80);
      MicoGpioOutputLow( MICO_GPIO_SYS_LED );
      HAL_Delay(1500);
    }

    /* Turn off LED2 */
    MicoGpioOutputLow( MICO_GPIO_SYS_LED );

    /* Suspend Thread 2 */
    osThreadSuspend(NULL);

    count = osKernelSysTick() + 5000;

    while (count >= osKernelSysTick())
    {
      MicoGpioOutputHigh( MICO_GPIO_SYS_LED );
      osDelay(1000);
      MicoGpioOutputLow( MICO_GPIO_SYS_LED );
      HAL_Delay(500);
    }

    /* Resume Thread 2*/
    osThreadResume(LEDThread2Handle);
  
  }
}

/**
  * @brief  Toggle LED2 thread
  * @param  argument not used
  * @retval None
  */
static void LED_Thread2(void const *argument)
{
  uint32_t count;
  (void) argument;

  for (;;)
  {
    count = osKernelSysTick() + 10000;

    while (count >= osKernelSysTick())
    {
      MicoGpioOutputHigh( MICO_GPIO_SYS_LED );
      osDelay(100);
      MicoGpioOutputLow( MICO_GPIO_SYS_LED );
      osDelay(100);
      MicoGpioOutputHigh( MICO_GPIO_SYS_LED );
      osDelay(100);
      MicoGpioOutputLow( MICO_GPIO_SYS_LED );
      HAL_Delay(1000); 
    }

    /* Turn off LED2 */
    MicoGpioOutputLow( MICO_GPIO_SYS_LED );

    /* Resume Thread 1 */
    osThreadResume(LEDThread1Handle);

    /* Suspend Thread 2 */
    osThreadSuspend(NULL);
  }
}