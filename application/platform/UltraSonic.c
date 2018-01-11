/////////////Ultrasonic Double//////////
////////////////////////////////////////
#include "platform.h"
#include "stm32f1xx.h"
#include "UltraSonic.h"

#include "Debug.h"

#include "platform_internal.h"
#include "platform_config.h"

#include "app_platform.h"

#include "UltraSonic.h"

#include "platform_tim.h"

#define UltraSonicLog(format, ...)  custom_log("Ultrasonic", format, ##__VA_ARGS__)

extern void Set_IO_Direction(mico_gpio_t gpio, io_dir_t b);
extern void UltraIoOutputLow(void);
extern void UltraIoOutputHigh(void);
extern void UltraDataIO_Output(void);
extern void UltraDataIO_Input(void);
extern void UltraDataIO_Output(void);
extern void UltraDataIO_InputIT(void);




ultra_sonic_data_t ultra_sonic_data_ram;
ultra_sonic_data_t *ultra_sonic_data = &ultra_sonic_data_ram; 



uint8_t measure_repeat_filter = 0;





void UltraSonicInit(void)
{  
    memset(ultra_sonic_data, 0, sizeof(ultra_sonic_data_t));
    ultra_sonic_data->data_ready_flag = DATA_NOT_READY;
    ultra_sonic_data->i_am_en = true;
   
}


void Ultra_IO_Output(void)
{
    UltraDataIO_Output();
}

void Ultra_IO_Input(void)
{
    UltraDataIO_Input();
}
void Ultra_IO_InputIT(void)
{
    UltraDataIO_InputIT();
}

extern void UltraTrigOutputHigh(void);
extern void UltraTrigOutputLow(void);
uint32_t measure_cnt = 0;
void UltraSonicStart(void)
{
  
    //uint32_t start_time = 0;
    measure_cnt++;

    TimerInit();
    StartTimer();
    
    DISABLE_INTERRUPTS();
    UltraTrigOutputHigh();
    delay_us(10);
    UltraTrigOutputLow();
    
    ultra_sonic_data->interval_time.cnt = 0;
    memset(ultra_sonic_data->interval_time.time, 0, INTERVAL_TIME_MAX);  
    memset(ultra_sonic_data->compute_ditance, 0, INTERVAL_TIME_MAX);
    ultra_sonic_data->end_flag = 0;
    ultra_sonic_data->start_flag = 1;
    ultra_sonic_data->data_ready_flag = DATA_NOT_READY;
    ENABLE_INTERRUPTS();
    delay_us(400);
    Ultra_IO_InputIT();
    
    ultra_sonic_data->send_time = GetTimerCount();
    
    
}

#include "can_protocol.h"
extern uint32_t ultrasonic_src_id;
//static uint16_t last_distance = NO_OBJ_DETECTED;

void CompleteAndUploadData(void)
{
    uint8_t i = 0;
    uint16_t distance = 0;
    CAN_ID_UNION id;
    //uint16_t interval_time = 0;
    id.CanID_Struct.SourceID = 0x80;
    id.CanID_Struct.DestMACID = 0x01;
    id.CanID_Struct.SrcMACID = ultrasonic_src_id;
    id.CanID_Struct.ACK = 1;
    id.CanID_Struct.FUNC_ID = CAN_FUN_ID_READ;
    id.CanID_Struct.res = 0;
    
    if( (ultra_sonic_data->data_ready_flag != DATA_EXPIRED) && (ultra_sonic_data->data_ready_flag != DATA_NOT_READY) && (ultra_sonic_data->interval_time.cnt > 0) )
    {
        DISABLE_INTERRUPTS();
        ultra_sonic_data->compute_ditance[i] = ultra_sonic_data->interval_time.time[i] * 17 /1000;
        
        distance = ultra_sonic_data->compute_ditance[i];
        CanTX( MICO_CAN1, id.CANx_ID, (uint8_t *)&distance, sizeof(distance) ); 
        ENABLE_INTERRUPTS();
        printf("%d\n",distance);
    }
    else
    {
        distance = NO_OBJ_DETECTED;
        CanTX( MICO_CAN1, id.CANx_ID, (uint8_t *)&distance, sizeof(distance) ); 
        printf("%d\n",distance);
    }
    UltraDataIO_Output();
    StopTimer();
}

#define ULTRASONIC_MEASURE_TIME                 13/SYSTICK_PERIOD //unit: ms
#define ULTRASONIC_DATA_EXIST_TIME              500/SYSTICK_PERIOD//unit: ms
#define ULTRASONIC_MEASURE_CRITICAL_TIME        50/SYSTICK_PERIOD //unit: ms
extern platform_can_driver_t  platform_can_drivers[];
void UltraSonicDataTick(void)
{
    static uint32_t start_time_1 = 0;
    //static uint32_t start_time_2 = 0;
    static uint8_t flag_1 = 0;
    static uint8_t flag_2 = 0;
    
    if((ultra_sonic_data->start_flag == 1) && (ultra_sonic_data->end_flag == 0))
    {
        if(flag_1 == 0)
        {
            start_time_1 = os_get_time();
            flag_1 = 1;
            flag_2 = 0;
        }
        if(os_get_time() - start_time_1 >= ULTRASONIC_MEASURE_TIME)
        {
            if(flag_2 == 0)
            {
                CompleteAndUploadData();
                flag_2 = 1;
            }
            
        }   
        if(os_get_time() - start_time_1 >= ULTRASONIC_MEASURE_CRITICAL_TIME)
        {
           
            ultra_sonic_data->start_flag = 0;
            ultra_sonic_data->end_flag = 1;
            flag_1 = 0; 
            __HAL_CAN_ENABLE_IT( platform_can_drivers[MICO_CAN1].handle, CAN_IT_FMP0 | CAN_IER_FFIE0 | CAN_IT_FOV0 );
        }   
    }
 /*
    if(ultra_sonic_data->data_ready_flag == DATA_NEW_COMING)
    {
        start_time_2 = os_get_time();
        ultra_sonic_data->data_ready_flag  = DATA_READY;
    }
    if(os_get_time() - start_time_2 >= ULTRASONIC_DATA_EXIST_TIME)
    {
        ultra_sonic_data->data_ready_flag = DATA_EXPIRED;
    }  
    */
}


void ShowTestLog(void)
{
    uint8_t i = 0;
    //uint32_t tmp;
    if((ultra_sonic_data->data_ready_flag != DATA_EXPIRED) && (ultra_sonic_data->data_ready_flag != DATA_NOT_READY))
    {
        //for(i = 0; i < ultra_sonic_data->interval_time.cnt; i++)
        {
            ultra_sonic_data->compute_ditance[i] = ultra_sonic_data->interval_time.time[i] * 17 /1000;
#if 1
            UltraSonicLog("%d - distance : %d cm\r\n",i + 1,ultra_sonic_data->compute_ditance[i]);
#else
            CanTX( MICO_CAN1, 0x12345678, (uint8_t *)&ultra_sonic_data->compute_ditance[i], sizeof(ultra_sonic_data->compute_ditance[i]) );
#endif
        }
    }
    

#if 0   
    if(ultrasonic_frq_calibration->start_flag == 1)
    {
        tmp = 1000000/ultrasonic_frq_calibration->interval_time;
        UltraSonicLog("frq : %d\r\n", tmp*2*12);
    }
#endif    
}


uint16_t UltraSonicGetMeasureData(void)
{
    if((ultra_sonic_data->data_ready_flag == DATA_NEW_COMING) || (ultra_sonic_data->data_ready_flag == DATA_READY))
    {
        ultra_sonic_data->compute_ditance[0] = ultra_sonic_data->interval_time.time[0] * 17 /1000;  //the nearest distance data
        return ultra_sonic_data->compute_ditance[0];
    }
    else
    {
        return NO_OBJ_DETECTED;
    }
          
}
