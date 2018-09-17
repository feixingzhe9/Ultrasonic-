#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "mico.h"

#define NO_OBJ_DETECTED                 0x00000000
#define DANGER_DISTANCE                 50
#define DANGER_DISTANCE_FILTER_CNT      2

#define INTERVAL_TIME_MAX   10

typedef struct
{
    uint32_t time[INTERVAL_TIME_MAX];
    uint8_t cnt;
}interval_time_t;

typedef struct
{
    uint32_t send_time;
    uint32_t rcv_time;
    interval_time_t interval_time;
    uint16_t compute_ditance[INTERVAL_TIME_MAX];
    uint8_t data_ready_flag;
#define DATA_NEW_COMING     0x01
#define DATA_READY          0x02
#define DATA_EXPIRED        0x04
#define DATA_NOT_READY      0x08
    uint8_t start_flag;
    uint8_t end_flag;
    uint8_t i_am_en;
    uint8_t group;
    uint8_t my_num;
}ultra_sonic_data_t;

typedef struct
{
    uint32_t low_start_time;
    uint32_t low_end_time;
    //uint32_t high_start_time;
    //uint32_t high_end_time;
}ultra_sonic_read_data_t;

void UltraSonicStart(void);
extern void UltraSonicInit(void);
extern void ShowTestLog(void);
extern void UltraSonicDataTick(void);
extern uint16_t UltraSonicGetMeasureData(void);
extern ultra_sonic_data_t *ultra_sonic_data;

extern ultra_sonic_read_data_t * ultra_sonic_read_data;


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


#define delay_ms          HAL_Delay
#endif
#endif
