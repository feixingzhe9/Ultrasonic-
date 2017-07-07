#ifndef __PLATFORM_TIM_H
#define __PLATFORM_TIM_H


void TimerInit(void);
void StartTimer(void);
void StopTimer(void);
uint32_t GetTimerCount(void);

#endif
