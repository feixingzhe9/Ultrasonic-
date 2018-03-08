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
 
/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
uint8_t test_data[] = {0x5a,6,0,0,0x60,0xa5};
uint8_t rcv_buf[50];
HAL_StatusTypeDef uart_err;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
//static void UartSendTest(void);
static void MX_NVIC_Init(void);



const  char menu[] =
"\r\n"
"Application for %s,\r\nSOFTWARE_VERSION: %s,\r\nHARDWARE_REVISION: %s\r\n";


//#define HOMWEE_TEST 

void SysLed(void);
void UltraSonicStartTick(void);
#ifdef HOMWEE_TEST 
void EmergencyStopTest(void);
#endif

static void MX_IWDG_Init(uint32_t period);
void feed_dog(void);

IWDG_HandleTypeDef hiwdg;

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
  
  MX_IWDG_Init(550);
  HAL_IWDG_Start(&hiwdg);
    HAL_Init();
    MX_GPIO_Init();

    MX_USART1_UART_Init();
    MX_NVIC_Init();


    if(HAL_UART_Receive_IT(&huart1,rcv_buf,1)!=HAL_OK)
    {
        //Error_Handler();
    }
  
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
    
    feed_dog();
  }
}

#define ULTRASONIC_SEND_TIME   100/SYSTICK_PERIOD
void UltraSonicStartTick(void) 
{
    static uint32_t start_time_1 = 0;

    if(os_get_time() - start_time_1 >= ULTRASONIC_SEND_TIME)
    {
        UltraSonicStart();
        start_time_1 = os_get_time();
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



#if 0
uint32_t uart_send_test_start_time = 0;
#define UART_SEND_TEST_PERIOD       500/SYSTICK_PERIOD
static void UartSendTest(void)
{
    if(os_get_time() - uart_send_test_start_time >= UART_SEND_TEST_PERIOD)
    {
        uart_send_test_start_time = os_get_time();
        HAL_UART_Transmit(&huart2, test_data, sizeof(test_data), 10);
    }
}
#endif
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
/* USART2 init function */
static void MX_USART1_UART_Init(void)
{
  __HAL_RCC_USART1_CLK_ENABLE();
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
  }
  
}

static void MX_NVIC_Init(void)
{
  /* USART2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 2);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

}