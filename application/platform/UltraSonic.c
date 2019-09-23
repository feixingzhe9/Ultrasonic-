/////////////Ultrasonic HC-SR-04//////////
////////////////////////////////////////
#include "platform.h"
#include "stm32f1xx.h"
#include "UltraSonic.h"
#include "Debug.h"
#include "platform_internal.h"
#include "platform_config.h"
#include "UltraSonic.h"
#include "platform_tim.h"

#define ultrasonic_log(format, ...)  custom_log("Ultrasonic", format, ##__VA_ARGS__)

extern void set_us_data_io_input_it(void);

ultra_sonic_data_t ultra_sonic_data_ram;
ultra_sonic_data_t *ultra_sonic_data = &ultra_sonic_data_ram;

uint8_t measure_repeat_filter = 0;

void ultrasonic_init(void)
{
    memset(ultra_sonic_data, 0, sizeof(ultra_sonic_data_t));
    ultra_sonic_data->data_ready_flag = DATA_NOT_READY;
    ultra_sonic_data->i_am_en = true;

    timer_init();
    start_timer();
}

void set_us_io_input_it(void)
{
    set_us_data_io_input_it();
}

extern void set_us_trig_output_high(void);
extern void set_us_trig_output_low(void);
uint32_t measure_cnt = 0;
void ultrasonic_start(void)
{
    measure_cnt++;

    DISABLE_INTERRUPTS();
    set_us_trig_output_high();
    delay_us(10);
    set_us_trig_output_low();

    ultra_sonic_data->interval_time.cnt = 0;
    memset(ultra_sonic_data->interval_time.time, 0, INTERVAL_TIME_MAX);
    memset(ultra_sonic_data->compute_ditance, 0, INTERVAL_TIME_MAX);
    ultra_sonic_data->end_flag = 0;
    ultra_sonic_data->start_flag = 1;
    ultra_sonic_data->data_ready_flag = DATA_NOT_READY;

    delay_us(400);
    //set_us_io_input_it();

    //ultra_sonic_data->send_time = get_timer_count();
    ENABLE_INTERRUPTS();
}

#include "can_protocol.h"
extern uint32_t ultrasonic_src_id;

void complete_upload_data(void)
{
    uint8_t i = 0;
    uint16_t distance = 0;
    can_id_union id;
    //uint16_t interval_time = 0;
    id.canx_id_t.source_id = 0x80;
    id.canx_id_t.dest_mac_id = 0x01;
    id.canx_id_t.src_mac_id = ultrasonic_src_id;
    id.canx_id_t.ack = 1;
    id.canx_id_t.func_id = CAN_FUN_ID_READ;
    id.canx_id_t.res = 0;

    if( (ultra_sonic_data->data_ready_flag != DATA_EXPIRED) && (ultra_sonic_data->data_ready_flag != DATA_NOT_READY) && (ultra_sonic_data->interval_time.cnt > 0) )
    {
        DISABLE_INTERRUPTS();
        ultra_sonic_data->compute_ditance[i] = ultra_sonic_data->interval_time.time[i] * 17 /1000;

        distance = ultra_sonic_data->compute_ditance[i];
        tx_can_data( MICO_CAN1, id.canx_id, (uint8_t *)&distance, sizeof(distance) );
        ENABLE_INTERRUPTS();
        //printf("%d\n",distance);
    }
    else
    {
        distance = NO_OBJ_DETECTED;
        tx_can_data( MICO_CAN1, id.canx_id, (uint8_t *)&distance, sizeof(distance) );
        //printf("%d\n",distance);
    }
}

#define ULTRASONIC_MEASURE_TIME                 13/SYSTICK_PERIOD //unit: ms
#define ULTRASONIC_DATA_EXIST_TIME              500/SYSTICK_PERIOD//unit: ms
#define ULTRASONIC_MEASURE_CRITICAL_TIME        50/SYSTICK_PERIOD //unit: ms
extern platform_can_driver_t  platform_can_drivers[];
void ultrasonic_data_tick(void)
{
    static uint8_t state = 0;
    static uint32_t measure_start_time = 0;

    switch(state)
    {
        case 0:
            if((ultra_sonic_data->start_flag == 1) && (ultra_sonic_data->end_flag == 0))
            {
                state = 1;
                measure_start_time = os_get_time();
            }
            else
            {
                break;
            }

        case 1:
            if(os_get_time() - measure_start_time >= ULTRASONIC_MEASURE_TIME)
            {
                complete_upload_data();
                state = 2;
            }
            else
            {
                break;
            }

        case 2:
            if(os_get_time() - measure_start_time >= ULTRASONIC_MEASURE_CRITICAL_TIME)
            {
                ultra_sonic_data->start_flag = 0;
                ultra_sonic_data->end_flag = 1;
                __HAL_CAN_ENABLE_IT( platform_can_drivers[MICO_CAN1].handle, CAN_IT_FMP0 | CAN_IER_FFIE0 | CAN_IT_FOV0 );
                state = 0;
            }
            break;

        default:
            break;
    }
}

void show_test_log(void)
{
    uint8_t i = 0;
    //uint32_t tmp;
    if((ultra_sonic_data->data_ready_flag != DATA_EXPIRED) && (ultra_sonic_data->data_ready_flag != DATA_NOT_READY))
    {
        //for(i = 0; i < ultra_sonic_data->interval_time.cnt; i++)
        {
            ultra_sonic_data->compute_ditance[i] = ultra_sonic_data->interval_time.time[i] * 17 /1000;
#if 1
            ultrasonic_log("%d - distance : %d cm\r\n",i + 1,ultra_sonic_data->compute_ditance[i]);
#else
            tx_can_data( MICO_CAN1, 0x12345678, (uint8_t *)&ultra_sonic_data->compute_ditance[i], sizeof(ultra_sonic_data->compute_ditance[i]) );
#endif
        }
    }

#if 0
    if(ultrasonic_frq_calibration->start_flag == 1)
    {
        tmp = 1000000/ultrasonic_frq_calibration->interval_time;
        ultrasonic_log("frq : %d\r\n", tmp*2*12);
    }
#endif
}

uint16_t get_us_measure_data(void)
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
