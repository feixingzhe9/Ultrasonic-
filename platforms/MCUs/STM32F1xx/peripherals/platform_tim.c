
#include "platform.h"
#include "platform_peripheral.h"
#include "platform_logging.h"
#include "app_platform.h"
#include "platform_tim.h"


void TimerInit(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    //TIM_MasterConfigTypeDef sMasterConfig;
    TIM_HandleTypeDef htimx;
    __HAL_RCC_TIM2_CLK_ENABLE();    //enable clock
      
    htimx.Instance = TIM2;
    htimx.Init.Prescaler = 71;
    htimx.Init.CounterMode = TIM_COUNTERMODE_UP;
    htimx.Init.Period = 0xffff;
    htimx.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htimx);
    
    //HAL_NVIC_SetPriority(TIM2_IRQn,2,0);

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(&htimx, &sClockSourceConfig);
    
}



void StartTimer(void)
{
    TIM_HandleTypeDef htimx;
    htimx.Instance = TIM2;
    
    HAL_TIM_Base_Start(&htimx);
    //HAL_TIM_Base_Start_IT(&htimx);
}
void StopTimer(void)
{
    //__HAL_RCC_TIM2_CLK_DISABLE();
    TIM_HandleTypeDef htimx;
    htimx.Instance = TIM2;
    HAL_TIM_Base_Stop(&htimx);
}
uint32_t GetTimerCount(void)
{
    return TIM2->CNT;
}



void CaptureCompareInit(void)
{
    //void TIM_CCxChannelCmd(TIM_TypeDef* TIMx, uint32_t Channel, uint32_t ChannelState)
  
  
    //uint32_t HAL_TIM_ReadCapturedValue(TIM_HandleTypeDef *htim, uint32_t Channel)
}



void TIM2_IRQHandler(void)
{
    uint8_t test = 0;
    test++;
    if(test == 1)
    {
        test = 0;
    }
}