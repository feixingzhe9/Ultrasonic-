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
extern void UltraDataIO_InputIT(void);
extern void V24OutputHigh(void);
extern void V24OutputLow(void);

ultra_sonic_frq_calibration_t ultra_sonic_frq_calibration_ram;
ultra_sonic_frq_calibration_t *ultrasonic_frq_calibration = &ultra_sonic_frq_calibration_ram; 

ultra_sonic_data_t ultra_sonic_data_ram;
ultra_sonic_data_t *ultra_sonic_data = &ultra_sonic_data_ram; 

ultra_sonic_read_data_t ultra_sonic_read_data_ram;
ultra_sonic_read_data_t * ultra_sonic_read_data = &ultra_sonic_read_data_ram;

ultra_sonic_time_t ultra_sonic_time_ram;
ultra_sonic_time_t *ultra_sonic_time = &ultra_sonic_time_ram;

ultra_sonic_ee_data_ut ultra_sonic_ee_data_ram;
ultra_sonic_ee_data_ut * ultra_sonic_ee_data = &ultra_sonic_ee_data_ram;

uint8_t measure_repeat_filter = 0;
uint8_t random_time[5] = {25,40,55,70,85};
#if 1
ultra_sonic_threshold_t ultra_sonic_threshold_ram = 
{
    //.threshold[0] = 31,
    .threshold[1]   = 31,
    .threshold[2]   = 28,
    .threshold[3]   = 25,
    .threshold[4]   = 23,
    .threshold[5]   = 18,
    .threshold[6]   = 15,
    .threshold[7]   = 8,
    .threshold[8]   = 6,
    .threshold[9]   = 5,
    .threshold[10]  = 4,
    .threshold[11]  = 4,
    .threshold[12]  = 4,
    .threshold[13]  = 4,
    .threshold[14]  = 4,
    .tres_scale     = 2,
    .thres_ini      = 0,
    .thres_len      = 0,
 
    /////parity value 
    .parity_0       = 0,
    .parity_1       = 0,
    .parity_2       = 0,
    .parity_3       = 0,
    .parity_4       = 0,
};
#else
ultra_sonic_threshold_t ultra_sonic_threshold_ram = 
{
    //.threshold[0] = 31,
    .threshold[1]   = 31,
    .threshold[2]   = 31,
    .threshold[3]   = 31,
    .threshold[4]   = 30,
    .threshold[5]   = 29,
    .threshold[6]   = 28,
    .threshold[7]   = 27,
    .threshold[8]   = 25,
    .threshold[9]   = 22,
    .threshold[10]  = 14,
    .threshold[11]  = 10,
    .threshold[12]  = 6,
    .threshold[13]  = 0,
    .threshold[14]  = 0,
    .tres_scale     = 2,
    .thres_ini      = 0,
    .thres_len      = 0,
 
    /////parity value 
    .parity_0       = 0,
    .parity_1       = 0,
    .parity_2       = 0,
    .parity_3       = 0,
    .parity_4       = 0,
};
#endif
ultra_sonic_threshold_t * ultra_sonic_threshold = &ultra_sonic_threshold_ram;

volatile uint32_t estimate_frq = F_DRV;

uint32_t f_drv;
uint32_t t_deb;
uint32_t t_snd;
uint32_t t_rec;
uint32_t t_cmd;
uint32_t t_cmd_prog;
uint32_t t_d;
uint32_t t_vprog;
uint32_t t_prog;
uint32_t t_0_low;
uint32_t t_0_high;
uint32_t t_1_low;
uint32_t t_1_high;
uint32_t t_bit;
uint32_t t_bit0;
uint32_t t_bit1;
uint32_t t_cal;
    
void UltraSonicTimeInit(void)
{
    estimate_frq = F_DRV + SEARCH_FRQ_COMPENSATION;
    
    t_snd = T_SND;
    t_rec = T_REC;
    t_cmd = T_CMD*8/10;
    //t_cmd = T_CMD;
    t_cmd_prog = T_CMD_PROG*8/10;
    t_d = T_D;
    t_vprog = T_VPROG;
    t_0_low = T_0_LOW;
    t_0_high = T_0_HIGH;
    t_1_low = T_1_LOW;
    t_1_high = T_1_HIGH;
    t_bit = T_BIT;
    t_bit0 = T_BIT0;
    t_bit1 = T_BIT1;
    t_cal = T_CAL;
}
void UltraSonicTimeChange(uint32_t frq)
{
    t_snd = T_SND*F_DRV/frq;
    t_rec = T_REC*F_DRV/frq;
    t_cmd = (T_CMD*8/10)*F_DRV/frq;
    //t_cmd = T_CMD;
    t_cmd_prog = (T_CMD_PROG*8/10)*F_DRV/frq;
    t_d = T_D*F_DRV/frq;
    //t_vprog = T_VPROG*F_DRV/frq;
    t_0_low = T_0_LOW*F_DRV/frq;
    t_0_high = T_0_HIGH*F_DRV/frq;
    t_1_low = T_1_LOW*F_DRV/frq;
    t_1_high = T_1_HIGH*F_DRV/frq;
    t_bit = T_BIT*F_DRV/frq;
    t_bit0 = T_BIT0*F_DRV/frq;
    t_bit1 = T_BIT1*F_DRV/frq;
    t_cal = T_CAL*F_DRV/frq;
}
void UltraSonicInit(void)
{
    TimerInit();
    StartTimer();
    V24OutputHigh();//
    
    memset(ultra_sonic_data, 0, sizeof(ultra_sonic_data_t));
    ultra_sonic_data->data_ready_flag = DATA_NOT_READY;
    
    ultrasonic_frq_calibration->start_flag = 0;
    ultrasonic_frq_calibration->over_time_flag = 0;
    
    memset(ultra_sonic_ee_data, 0, sizeof(ultra_sonic_ee_data_ut));
    ultra_sonic_ee_data->ee_data.amp_gain = 0x08;
    ultra_sonic_ee_data->ee_data.drv_cur = 0x08;
    
    
     
    
    UltraSonicTimeInit();
    
    //UltraSonicSetFrqToDest();
    delay_ms(20);
    
    //UltraSonicSetThreshold(ultra_sonic_threshold);
    delay_ms(10);
     
#if 0
    ultra_sonic_time->f_drv = F_DRV;
    //ultra_sonic_time->t_deb = T_DEB;
    ultra_sonic_time->t_snd = T_SND;
    ultra_sonic_time->t_rec = T_REC;
    ultra_sonic_time->t_cmd = T_CMD;
    ultra_sonic_time->t_cmd_prog = T_CMD_PROG;
    ultra_sonic_time->t_d = T_D;
    ultra_sonic_time->t_vprog = T_VPROG;
    ultra_sonic_time->t_0_low = T_0_LOW;
    ultra_sonic_time->t_0_high = T_0_HIGH;
    ultra_sonic_time->t_1_low = T_1_LOW;
    ultra_sonic_time->t_1_high = T_1_HIGH;
    ultra_sonic_time->t_bit = T_BIT;
    ultra_sonic_time->t_bit0 = T_BIT0;
    ultra_sonic_time->t_bit1 = T_BIT1;
    ultra_sonic_time->t_cal = T_CAL;
#endif

#if 0   
    //ultra_sonic_time->t_deb = T_DEB;
    t_snd = (uint32_t)t_snd;
    t_rec = (uint32_t)t_rec;
    t_cmd = (uint32_t)t_cmd;
    t_cmd_prog = (uint32_t)t_cmd_prog;
    t_d = (uint32_t)t_d;
    t_vprog = (uint32_t)t_vprog;
    t_0_low = (uint32_t)t_0_low;
    t_0_high = (uint32_t)t_0_high;
    t_1_low = (uint32_t)t_1_low;
    t_1_high = (uint32_t)t_1_high;
    t_bit = (uint32_t)t_bit;
    t_bit0 = (uint32_t)t_bit0;
    t_bit1 = (uint32_t)t_bit1;
    t_cal = (uint32_t)t_cal;
#endif
    

    
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


void UltraWriteBit(bool b)
{
    if(b == 1)
    {
        UltraIoOutputLow();
        delay_us(t_1_low);
        UltraIoOutputHigh();
        delay_us(t_1_high);
    }
    if(b == 0)
    {
        UltraIoOutputLow();
        delay_us(t_0_low);
        UltraIoOutputHigh();
        delay_us(t_0_high);
    }
}

void UltraSonicSendCMD(ultra_sonic_cmd_t cmd)
{
    uint8_t i;
    
    Ultra_IO_Output();
    UltraIoOutputLow();
    
    if((cmd == US_CMD_THRESHOLD_PROGRAM) || (cmd == US_CMD_READ_STATUS) || \
      (cmd == US_CMD_EE_WRITE) || (cmd == US_CMD_EE_READ) || (cmd == US_CMD_MEASURE_CONFIG))
    {
        delay_us(T_CMD);
    }
    else if((cmd == US_CMD_FREQUENCY_CALIBRARION) || (cmd == US_CMD_EE_PROGRAM))
    {
        delay_us(T_CMD_PROG);
    }

    for(i = 0; i < 4; i++)
    {
        if((cmd >> (4 - i - 1)) & 0x01 == 1)
        {
            UltraWriteBit(1);
        }
        if((cmd >> (4 - i - 1)) & 0x01 == 0)
        {
            UltraWriteBit(0);
        }
    }
}

extern void UltraTrigOutputHigh(void);
extern void UltraTrigOutputLow(void);
void UltraSonicStart(void)
{
    uint32_t start_time = 0;
#if 0
    Ultra_IO_Output();
	UltraIoOutputLow();
	delay_us(t_snd);
	UltraIoOutputHigh();
    
    delay_us(150);  
   
    DISABLE_INTERRUPTS();
    ultra_sonic_data->interval_time.cnt = 0;
    memset(ultra_sonic_data->compute_ditance, 0, INTERVAL_TIME_MAX);
    ultra_sonic_data->end_flag = 0;
    ultra_sonic_data->start_flag = 1;
    ENABLE_INTERRUPTS();
    
    Ultra_IO_InputIT();
    
    ultra_sonic_data->send_time = GetTimerCount();
#endif  
    
    UltraTrigOutputHigh();
    delay_us(10);
    UltraTrigOutputLow();
    
    DISABLE_INTERRUPTS();
    ultra_sonic_data->interval_time.cnt = 0;
    memset(ultra_sonic_data->compute_ditance, 0, INTERVAL_TIME_MAX);
    ultra_sonic_data->end_flag = 0;
    ultra_sonic_data->start_flag = 1;
    ENABLE_INTERRUPTS();
    Ultra_IO_InputIT();
    start_time = GetTimerCount();
    //while(GetTimerCount() - start_time < 400);
    delay_us(400);
    ultra_sonic_data->send_time = GetTimerCount();
    
    
}

#include "can_protocol.h"
extern uint32_t ultrasonic_src_id;
static uint16_t last_distance = NO_OBJ_DETECTED;

void CompleteAndUploadData(void)
{
    uint8_t i = 0;
    CAN_ID_UNION id;
    uint16_t interval_time = 0;
    id.CanID_Struct.SourceID = 0x80;
    id.CanID_Struct.DestMACID = 0x60;//test;
    id.CanID_Struct.SrcMACID = ultrasonic_src_id;
    id.CanID_Struct.ACK = 1;
    id.CanID_Struct.FUNC_ID = CAN_FUN_ID_READ;
    id.CanID_Struct.res = 0;
    if((ultra_sonic_data->data_ready_flag != DATA_EXPIRED) && (ultra_sonic_data->data_ready_flag != DATA_NOT_READY))
    {
/*
        do
        {
            ultra_sonic_data->compute_ditance[i] = ultra_sonic_data->interval_time.time[i] * 17 /1000;
        
        }while((i < ultra_sonic_data->interval_time.cnt) && (ultra_sonic_data->compute_ditance[i++] <= MEASURE_BLIND_DISTANCE));
        
        if(i > 0)i--;
*/
        
        {
            //if(ultra_sonic_data->interval_time.time[i] > 400)
            {
                //interval_time = ultra_sonic_data->interval_time.time[i] - 400;
            }
            ultra_sonic_data->compute_ditance[i] = ultra_sonic_data->interval_time.time[i] * 17 /1000;
            printf("%d\n",ultra_sonic_data->compute_ditance[i]);
/*           
            if((ultra_sonic_data->compute_ditance[i] <= DANGER_DISTANCE) && (++measure_repeat_filter < DANGER_DISTANCE_FILTER_CNT))
            {
                delay_ms(20 + GetTimerCount() % 30);//random time - 30ms ~ 79ms
                UltraSonicStart();  
            }
            else               
            {
                measure_repeat_filter = 0;   
                if(ultra_sonic_data->compute_ditance[i] <= MEASURE_BLIND_DISTANCE)
                {
                    uint16_t tmp = last_distance;           
                    CanTX( MICO_CAN1, id.CANx_ID, (uint8_t *)&tmp, sizeof(tmp) ); 
                    printf("%d\n",tmp);
                }
                else
                {
                    CanTX( MICO_CAN1, id.CANx_ID, (uint8_t *)&ultra_sonic_data->compute_ditance[i], sizeof(ultra_sonic_data->compute_ditance[i]) ); 
                    printf("%d\n",ultra_sonic_data->compute_ditance[i]);
                    last_distance = ultra_sonic_data->compute_ditance[i];
                    if(last_distance <= MEASURE_BLIND_DISTANCE)
                    {
                        printf("WTF ! \r\n");
                    }
                }
                
            }
*/
        }
    }
    else
    {
        measure_repeat_filter = 0;
        uint16_t tmp = NO_OBJ_DETECTED;
        CanTX( MICO_CAN1, id.CANx_ID, (uint8_t *)&tmp, sizeof(tmp) ); 
        printf("%d\n",tmp);
    }
}

#define ULTRASONIC_MEASURE_TIME         10/SYSTICK_PERIOD //unit: ms
#define ULTRASONIC_DATA_EXIST_TIME      500/SYSTICK_PERIOD//unit: ms
void UltraSonicDataTick(void)
{
    static uint32_t start_time_1 = 0;
    static uint32_t start_time_2 = 0;
    static uint8_t flag_1 = 0;
    
    if((ultra_sonic_data->start_flag == 1) && (ultra_sonic_data->end_flag == 0))
    {
        if(flag_1 == 0)
        {
            start_time_1 = os_get_time();
            flag_1 = 1;
        }
        if(os_get_time() - start_time_1 >= ULTRASONIC_MEASURE_TIME)
        {
           
            ultra_sonic_data->start_flag = 0;
            ultra_sonic_data->end_flag = 1;
            flag_1 = 0; 
            CompleteAndUploadData();
        }   
    }
 
    if(ultra_sonic_data->data_ready_flag == DATA_NEW_COMING)
    {
        start_time_2 = os_get_time();
        ultra_sonic_data->data_ready_flag  = DATA_READY;
    }
    if(os_get_time() - start_time_2 >= ULTRASONIC_DATA_EXIST_TIME)
    {
        ultra_sonic_data->data_ready_flag = DATA_EXPIRED;
    }    
}
#define ULTRASONIC_READ_ERR      0xffffffff
uint32_t UltraSonicReadData(uint8_t num)
{
#define READ_OVER_TIME      300
    uint32_t tmp;
    uint32_t readvalue = 0;
    uint32_t cnt = 0;
    uint8_t i;
    
    for(i = 0; i < num; i++)
    {
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 0) && (cnt < READ_OVER_TIME))
        {
            cnt++;
        }       
        ultra_sonic_read_data->low_start_time = GetTimerCount();
        if(cnt >= READ_OVER_TIME)
        {
            return ULTRASONIC_READ_ERR;
        }
        cnt = 0;
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 1) && (cnt < READ_OVER_TIME))
        {
            cnt++;
        }   
        ultra_sonic_read_data->low_end_time = GetTimerCount();
        if(cnt >= READ_OVER_TIME)
        {
            return ULTRASONIC_READ_ERR;
        }
        cnt = 0;
#if 0
        tmp = (ultra_sonic_read_data->low_end_time > ultra_sonic_read_data->low_start_time) ? (ultra_sonic_read_data->low_end_time - ultra_sonic_read_data->low_start_time) : \
          (USER_TIM_MAX_CNT - (ultra_sonic_read_data->low_start_time - ultra_sonic_read_data->low_end_time));
#else
          if(ultra_sonic_read_data->low_end_time > ultra_sonic_read_data->low_start_time)
          {
                tmp = ultra_sonic_read_data->low_end_time - ultra_sonic_read_data->low_start_time;
          }
          else
          {
                tmp = USER_TIM_MAX_CNT - (ultra_sonic_read_data->low_start_time - ultra_sonic_read_data->low_end_time);
          }
#endif    
          
#if 1
        if((tmp < T_0_LOW_MAX) && (tmp > T_0_LOW_MIN))
        {
            readvalue &= ~(1<<(num - 1 - i));
        }
        else if((tmp < T_1_LOW_MAX) && (tmp > T_1_LOW_MIN))
        {
            readvalue |= 1<<(num - 1 - i);
        }
#else
        if((tmp < T_0_LOW_MAX) && (tmp > T_0_LOW_MIN))
        {
            readvalue &= ~(1<<i);
        }
        else if((tmp < T_1_LOW_MAX) && (tmp > T_1_LOW_MIN))
        {
            readvalue |= 1<<i;
        }
#endif
        else
        {
            UltraSonicLog("read error\r\n");
            return ULTRASONIC_READ_ERR;
        }  
    }
    return readvalue;
}
//uint32_t UltraSonicReadEEStart(void)
uint32_t UltraSonicReadEEStart(ultra_sonic_ee_data_ut * data)
{
    uint32_t readvalue = 0;

    Ultra_IO_Output();
	UltraIoOutputLow();
    delay_us(t_cmd);
	UltraIoOutputHigh();
    delay_us(t_d);
	UltraWriteBit(0);
	UltraWriteBit(1);
	UltraWriteBit(0);
	UltraWriteBit(0);
	Ultra_IO_Input();
#if 1
    readvalue = UltraSonicReadData(20);
    data->value = readvalue;
#else  
    data->ee_data.spare_bit = UltraSonicReadData(1);
    data->ee_data.io_mask = UltraSonicReadData(1);
    data->ee_data.noise_cfg = UltraSonicReadData(2);
    data->ee_data.filt_adjust = UltraSonicReadData(1);
    data->ee_data.f_drv_adj = UltraSonicReadData(7);
    data->ee_data.drv_cur = UltraSonicReadData(4);
    data->ee_data.amp_gain = UltraSonicReadData(4);  
#endif  
    
    //UltraSonicLog("EEPROM readvalue is %x \r\n",readvalue);
    //ultra_sonic_ee_data->value = readvalue;
    return readvalue;
}

//uint32_t UltraSonicReadEEStart(void)
uint32_t UltraSonicReadEETest(void)
{
    uint32_t readvalue = 0;

    Ultra_IO_Output();
	UltraIoOutputLow();
    delay_us(t_cmd);
	UltraIoOutputHigh();
    delay_us(t_d);
	UltraWriteBit(0);
	UltraWriteBit(1);
	UltraWriteBit(0);
	UltraWriteBit(0);
	Ultra_IO_Input();
#if 1 
    readvalue = UltraSonicReadData(20);
#else  
    data->ee_data.spare_bit = UltraSonicReadData(1);
    data->ee_data.io_mask = UltraSonicReadData(1);
    data->ee_data.noise_cfg = UltraSonicReadData(2);
    data->ee_data.filt_adjust = UltraSonicReadData(1);
    data->ee_data.f_drv_adj = UltraSonicReadData(7);
    data->ee_data.drv_cur = UltraSonicReadData(4);
    data->ee_data.amp_gain = UltraSonicReadData(4);  
#endif  
    
    //UltraSonicLog("EEPROM readvalue is %x \r\n",readvalue);
    //ultra_sonic_ee_data->value = readvalue;
    return readvalue;
}

#if 0
uint32_t UltraSonicParseEEData(ultra_sonic_ee_data_ut *data, uint32_t value)
{
    data->ee_data.spare_bit = (value&SPARE_BIT_MASK)>>SPARE_BIT_BIT_NUM;
    data->ee_data.io_mask = (value&IO_MASK_MASK)>>IO_MASK_BIT_NUM;
    data->ee_data.noise_cfg = (value&NOISE_CFG_MASK)>>NOISE_CFG_BIT_NUM;
    data->ee_data.filt_adjust = (value&FILT_ADJUST_MASK)>>FILT_ADJUST_BIT_NUM;
    data->ee_data.f_drv_adj = (value&F_DRV_ADJ_MASK)>>F_DRV_ADJ_BIT_NUM;
    data->ee_data.drv_cur = (value&DRV_CUR_MASK)>>DRV_CUR_BIT_NUM;
    data->ee_data.amp_gain = (value&AMP_GAIN_MASK)>>AMP_GAIN_BIT_NUM;  
    return 0;
}
uint32_t UltraSonciBuildEEData(ultra_sonic_ee_data_t * data)
{
    uint32_t value = 0;
    value = (data->spare_bit << SPARE_BIT_BIT_NUM) | (data->io_mask << IO_MASK_BIT_NUM) |\
      (data->noise_cfg << NOISE_CFG_BIT_NUM) | (data->filt_adjust);
    
    return 0;
}
#endif

uint32_t UltraSonicReadEE(ultra_sonic_ee_data_ut *ee_data)
{
    uint32_t frq_tmp = estimate_frq;
    ultra_sonic_ee_data_ut ee_data_tmp;
    
    //ultra_sonic_ee_data_ut ee_data_tmp;
    uint8_t cnt = 0;
    
    UltraSonicReadEEStart(&ee_data_tmp); 
    
    while(ee_data_tmp.ee_data.f_drv_adj == 0x7f/*ULTRASONIC_READ_ERR*/)
    {
        memset(&ee_data_tmp, 0, sizeof(ultra_sonic_ee_data_ut));
        if(cnt < 250)
        {
            cnt++;
        }
        else
        {
            UltraSonicLog("FATAL: CAN NOT READ ULTRASONIC EEPROM  VALUE ! ! ! !\r\nSYSTEM WILL RESET!\r\n");
            cnt = 0;
            platform_mcu_reset();
        }
      
        if(frq_tmp > MIN_FRQ)
        {
            frq_tmp -= ULTRA_SEARCH_FRQ_STEP_SIZE;
        }
        else
        {
            frq_tmp = MAX_FRQ;
        }
        
        UltraSonicTimeChange(frq_tmp);
        
        Ultra_IO_Output();//////
        UltraIoOutputHigh();//////
        
        delay_ms(10);
        UltraSonicReadEEStart(&ee_data_tmp);
    }
    //estimate_frq = frq_tmp;
    
    //UltraSonicLog("estimate_frq : %d\r\n",estimate_frq);
    memcpy(ee_data, &ee_data_tmp, sizeof(ultra_sonic_ee_data_ut) );
    return 0;
}
     
uint32_t UltraSonicReadStatusStart(void)
{
    uint32_t readvalue;
    
    Ultra_IO_Output();
	UltraIoOutputLow();	
    delay_us(t_cmd);
	UltraIoOutputHigh();
    delay_us(t_d);
    
	UltraWriteBit(0);
	UltraWriteBit(0);
	UltraWriteBit(1);
	UltraWriteBit(0);
	Ultra_IO_Input();
    
    ultra_sonic_data->send_time = GetTimerCount();
    
    readvalue = UltraSonicReadData(2);
    printf("status is %x\r\n", readvalue);
    return readvalue;
}

uint32_t UltraSonicReadStatus(void)
{
    uint32_t frq_tmp = estimate_frq;
    uint32_t status = UltraSonicReadStatusStart();
    uint8_t cnt = 0;
    while(status == ULTRASONIC_READ_ERR)
    {
        status = 0;
        if(cnt < 250)
        {
            cnt++;
        }
        else
        {
            UltraSonicLog("FATAL: CAN NOT READ ULTRASONIC EEPROM  VALUE ! ! ! !\r\nSYSTEM WILL RESET!\r\n");
            cnt = 0;
            platform_mcu_reset();
        }
      
        if(frq_tmp > MIN_FRQ)
        {
            frq_tmp -= ULTRA_SEARCH_FRQ_STEP_SIZE;
        }
        else
        {
            frq_tmp = MAX_FRQ;
        }
        
        UltraSonicTimeChange(frq_tmp);
        
        //Ultra_IO_Output();///////
        //UltraIoOutputHigh();//////
        
        delay_ms(15);////////////
        status = UltraSonicReadStatusStart();
    }
    return status;
}



#define CALIBRATION_FILTER_NUM              7
#define CALIBRATION_OVER_TIME               500
#define ULTRASONIC_FRQ_CALIBRATION_ERR      0 
uint32_t UltraReadCalibrationData(void)
{
    uint32_t cnt = 0;
    uint32_t i;
    uint32_t tmp;
    
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    uint32_t frq;
#if 1
    uint32_t sum = 0;
    for(i = 0; i < CALIBRATION_FILTER_NUM; i++)
    {
      //measure low level time
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 0) && (cnt < CALIBRATION_OVER_TIME))
        {
            cnt++;
        }       
        start_time = GetTimerCount();
        if(cnt >= CALIBRATION_OVER_TIME)
        {
            return ULTRASONIC_FRQ_CALIBRATION_ERR;
        }
        cnt = 0;
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 1) && (cnt < CALIBRATION_OVER_TIME))
        {
            cnt++;
        }  

        end_time = GetTimerCount();
        if(cnt >= CALIBRATION_OVER_TIME)
        {
            return ULTRASONIC_FRQ_CALIBRATION_ERR;
        }
        cnt = 0;
          if(end_time > start_time)
          {
                tmp = end_time - start_time;
          }
          else
          {
                tmp = USER_TIM_MAX_CNT - (start_time - end_time);
          }
          sum += tmp;         
          //measure high level time
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 1) && (cnt < CALIBRATION_OVER_TIME))
        {
            cnt++;
        }       
        start_time = GetTimerCount();
        if(cnt >= CALIBRATION_OVER_TIME)
        {
            return ULTRASONIC_FRQ_CALIBRATION_ERR;
        }
        cnt = 0;
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 0) && (cnt < CALIBRATION_OVER_TIME))
        {
            cnt++;
        }   
        end_time = GetTimerCount();
        if(cnt >= CALIBRATION_OVER_TIME)
        {
            return ULTRASONIC_FRQ_CALIBRATION_ERR;
        }
        cnt = 0;
        if(end_time > start_time)
        {
            tmp = end_time - start_time;
        }
        else
        {
            tmp = USER_TIM_MAX_CNT - (start_time - end_time);
        }
        sum += tmp;
    }
    tmp = sum/(CALIBRATION_FILTER_NUM*2);
    
    
    ////////////////////////////////////////////////
    tmp += 2;////////////////////???????????????????????
    /////////////////////////////////////////////////
    
    
    frq = 1000000 * 12 /tmp;
#else
    for(i = 0; i < CALIBRATION_FILTER_NUM; i++)
    {
      //measure low level time
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 0) && (cnt < CALIBRATION_OVER_TIME))
        {
            cnt++;
        }
        if(i == 0)
        {
            start_time = GetTimerCount();
        }

        if(cnt >= CALIBRATION_OVER_TIME)
        {
            return ULTRASONIC_FRQ_CALIBRATION_ERR;
        }
        cnt = 0;
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 1) && (cnt < CALIBRATION_OVER_TIME))
        {
            cnt++;
        }  

        //end_time = GetTimerCount();
        if(cnt >= CALIBRATION_OVER_TIME)
        {
            return ULTRASONIC_FRQ_CALIBRATION_ERR;
        }
        cnt = 0;
          
          //measure high level time
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 1) && (cnt < CALIBRATION_OVER_TIME))
        {
            cnt++;
        }       
        //start_time = GetTimerCount();
        if(cnt >= CALIBRATION_OVER_TIME)
        {
            return ULTRASONIC_FRQ_CALIBRATION_ERR;
        }
        cnt = 0;
        while((MicoGpioInputGet(MICO_GPIO_ULTRA_DATA) != 0) && (cnt < CALIBRATION_OVER_TIME))
        {
            cnt++;
        }  
        
        if(cnt >= CALIBRATION_OVER_TIME)
        {
            return ULTRASONIC_FRQ_CALIBRATION_ERR;
        }
        if(i == CALIBRATION_FILTER_NUM - 1)
        {
            end_time = GetTimerCount(); 
            cnt = 0;
            if(end_time > start_time)
            {
                tmp = end_time - start_time;
            }
            else
            {
                tmp = USER_TIM_MAX_CNT - (start_time - end_time);
            }
        }
        
    }
    //tmp = tmp/(CALIBRATION_FILTER_NUM*2);
    
    
    ////////////////////////////////////////////////
    //tmp += 2;////////////////////???????????????????????
    /////////////////////////////////////////////////
    
    
    frq = 1000000 * 12 * (CALIBRATION_FILTER_NUM*2) /tmp;
    tmp = tmp/(CALIBRATION_FILTER_NUM*2);
#endif
    UltraSonicLog("average time :%d , frq : %d\r\n",tmp,frq);
    return frq;
}
uint32_t UltraSonicFrqCalibrationStart(void)
{
    Ultra_IO_Output();
	UltraIoOutputLow();	
    delay_us(t_cmd_prog);
	UltraIoOutputHigh();

    delay_us(t_d);
    
	UltraWriteBit(0);
	UltraWriteBit(0);
	UltraWriteBit(1);
	UltraWriteBit(1);
 
    delay_us(t_d);
	Ultra_IO_Input();	   

    ultrasonic_frq_calibration->start_time = GetTimerCount();
    return UltraReadCalibrationData();   
}
uint32_t UltraSonicFrqCalibration(void)
{
    uint32_t frq_tmp = estimate_frq;
    uint32_t frq = UltraSonicFrqCalibrationStart();
    uint8_t cnt = 0;
    
    while(frq == ULTRASONIC_FRQ_CALIBRATION_ERR)
    {
        if(cnt < 150)
        {
            cnt++;
        }
        else
        {
            UltraSonicLog("FATAL: CAN NOT READ ULTRASONIC EEPROM  VALUE ! ! ! !\r\nSYSTEM WILL RESET!\r\n");
            cnt = 0;
            platform_mcu_reset();
        }
      
        if(frq_tmp > MIN_FRQ)
        {
            frq_tmp -= ULTRA_SEARCH_FRQ_STEP_SIZE;
        }
        else
        {
            frq_tmp = MAX_FRQ;
        }
        
        UltraSonicTimeChange(frq_tmp);
        
        Ultra_IO_Output();///////
        UltraIoOutputHigh();//////
        
        delay_ms(5);////////////
        frq = UltraSonicFrqCalibrationStart();
    }
 
    estimate_frq = frq;///////////////////////////
    UltraSonicLog("read frq is %d\r\n",frq);////////////////////
    //UltraSonicTimeChange(frq_tmp);////////////////////
    return frq; 
}
void UltraSonicWrteData(uint32_t data, uint8_t bit_num)
{
    uint8_t i;
    if(bit_num == 0)
    {
        return;
    }
#if 0
    for(i = 0; i < bit_num; i++)
    {
        if((data >> i) & 0x00000001 == 1)
        {
           UltraWriteBit(1);
        }
        else //if((data >>(i - 1)) & 0x01 == 0)
        {
            UltraWriteBit(0);
        }   
    }
#else
    for(i = bit_num; i > 0; i--)
    {
        if((data >>(i - 1)) & 0x01 == 1)
        {
           UltraWriteBit(1);
        }
        else //if((data >>(i - 1)) & 0x01 == 0)
        {
            UltraWriteBit(0);
        }   
    }
#endif
}
void UltraSonicWriteEE(ultra_sonic_ee_data_ut* data)
{
	Ultra_IO_Output();
	UltraIoOutputLow();
	delay_us(t_cmd);
	UltraIoOutputHigh();
	delay_us(t_d);
	UltraWriteBit(0);
	UltraWriteBit(1);
	UltraWriteBit(0);
	UltraWriteBit(1);
 
#if 0
    UltraSonicWrteData(data->value, 20);
#else
    UltraSonicWrteData(data->ee_data.spare_bit, 1);
    UltraSonicWrteData(data->ee_data.io_mask, 1);
    UltraSonicWrteData(data->ee_data.noise_cfg, 2);
    UltraSonicWrteData(data->ee_data.filt_adjust, 1);
    UltraSonicWrteData(data->ee_data.f_drv_adj, 7);
    UltraSonicWrteData(data->ee_data.drv_cur, 4);
    UltraSonicWrteData(data->ee_data.amp_gain, 4);
#endif

}

void UltraSonicPrgramEE(void)
{
    Ultra_IO_Output();
	UltraIoOutputLow();
	delay_us(t_cmd_prog);
	UltraIoOutputHigh();
	delay_us(t_d);
    UltraWriteBit(0);
	UltraWriteBit(1);
	UltraWriteBit(1);
	UltraWriteBit(0);
	
	delay_ms(2);
    V24OutputLow();
	delay_us(T_PROG);
	V24OutputHigh();
    UltraSonicLog("Ultrasonic Program EEPROM ! \r\n");
}

extern void CanTX(mico_can_t can_type, uint32_t CANx_ID,uint8_t* pdata,uint16_t len);
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

void UltraSonicSetFrqToDest(void)
{
    uint32_t frq_tmp;
    //uint32_t ee_data_value = UltraSonicReadEE();
    //uint32_t ee_test;
    uint8_t cnt = 0;
    uint8_t ee_data_change_flag = 0;
    
    frq_tmp = UltraSonicFrqCalibration();
    delay_ms(20);
    UltraSonicReadEE(ultra_sonic_ee_data);
    delay_ms(20);
    //ee_test = UltraSonicReadEETest();
    //delay_ms(20);
#if 0
    if(frq_tmp > DEST_FRQ)
    {
        if(ultra_sonic_ee_data->ee_data.f_drv_adj != 0)
        {
            if(ultra_sonic_ee_data->ee_data.f_drv_adj >= (frq_tmp - DEST_FRQ)/ULTRA_FRQ_STEP_SIZE)
            {
                ultra_sonic_ee_data->ee_data.f_drv_adj -= (frq_tmp - DEST_FRQ)/ULTRA_FRQ_STEP_SIZE;//
            }
            else
            {
                ultra_sonic_ee_data->ee_data.f_drv_adj = 0;
            }
        }        
    }
    else
    {
        if(ultra_sonic_ee_data->ee_data.f_drv_adj + (DEST_FRQ - frq_tmp)/ULTRA_FRQ_STEP_SIZE < 1<<7)
        {
            ultra_sonic_ee_data->ee_data.f_drv_adj += (DEST_FRQ - frq_tmp)/ULTRA_FRQ_STEP_SIZE;//
        }
        else
        {
            ultra_sonic_ee_data->ee_data.f_drv_adj = (1<<7) -1;
        }       
    }
    
    //ultra_sonic_ee_data->ee_data.amp_gain = 0x08;//////////////////?????????????
    //ultra_sonic_ee_data->ee_data.drv_cur = 0x08;//////////////////?????????????
    
    UltraSonicWriteEE(ultra_sonic_ee_data);
    delay_ms(100);
    frq_tmp = UltraSonicFrqCalibration();
    delay_ms(100);
#endif  
    
    //ultra_sonic_ee_data->ee_data.amp_gain = 0x08;//////////////////?????????????
    //ultra_sonic_ee_data->ee_data.drv_cur = 0x08;//////////////////?????????????
    while(abs(frq_tmp - DEST_FRQ) > DEST_FRQ_ERROR)
    {
        if(cnt < 250)
        {
            cnt++;
        }
        else
        {
            cnt = 0;
            UltraSonicLog("FATAL: CAN NOT SET FRQQUENCY TO DESTINATION ! ! ! !\r\nSYSTEM WILL RESET!\r\n");
            platform_mcu_reset();
        }
        
        if(frq_tmp > DEST_FRQ)
        {
            ultra_sonic_ee_data->ee_data.f_drv_adj -= 1;
        }
        else
        {
            ultra_sonic_ee_data->ee_data.f_drv_adj += 1;
        }
        UltraSonicWriteEE(ultra_sonic_ee_data);
        ee_data_change_flag = 1;
        delay_ms(5);
        frq_tmp = UltraSonicFrqCalibration();
        delay_ms(5);
    }  
    frq_tmp = UltraSonicFrqCalibration();
    delay_ms(50);
    if(ee_data_change_flag == 1)
    {
        //UltraSonicPrgramEE();
    }  
    delay_ms(10);
}
uint8_t UltraSonicEvenParityCompute(uint32_t data, uint8_t len)
{
    uint8_t tmp = 0;
    for(uint8_t i = 0; i < len; i++)
    {
        tmp ^= (data>>i) & 0x01;
    }
    //return tmp^0x01;
    return tmp;
}



uint32_t UltraSonicWriteThresholdData(ultra_sonic_threshold_t * threshold)
{ 
    uint32_t status = ULTRASONIC_READ_ERR;
    uint32_t frq_tmp = estimate_frq;
    uint32_t frq_search_near = 0;
    uint8_t cnt = 0;
    UltraSonicLog("write threshold data...\r\n");
    if(status == ULTRASONIC_READ_ERR)
    {
        Ultra_IO_Output();
        UltraIoOutputLow();
        delay_us(t_cmd);
        UltraIoOutputHigh();
        delay_us(t_d);
        UltraWriteBit(0);
        UltraWriteBit(0);
        UltraWriteBit(0);
        UltraWriteBit(1);
        //Ultra_IO_Input();
        
        for(uint8_t i = 1; i < THRES_NUM; i++)
        {
            UltraSonicWrteData(threshold->threshold[i], 5);
        }
        UltraSonicWrteData(threshold->tres_scale, 2);
        UltraSonicWrteData(threshold->thres_ini, 1);
        UltraSonicWrteData(threshold->thres_len, 1);
        UltraSonicWrteData(threshold->parity_0, 1);
        UltraSonicWrteData(threshold->parity_1, 1);
        UltraSonicWrteData(threshold->parity_2, 1);
        UltraSonicWrteData(threshold->parity_3, 1);
        UltraSonicWrteData(threshold->parity_4, 1);   
        
        delay_ms(1);
        status = UltraSonicReadStatusStart();
        frq_search_near = frq_tmp + SEARCH_FRQ_COMPENSATION;
        cnt = 0;
        while( (status == ULTRASONIC_READ_ERR) && (cnt <= SEARCH_FRQ_COMPENSATION * 2 / ULTRA_SEARCH_FRQ_STEP_SIZE) )
        {
            cnt++;
            status = 0;
          
            if(frq_search_near > MIN_FRQ)
            {
                frq_search_near -= ULTRA_SEARCH_FRQ_STEP_SIZE;
            }
            else
            {
                frq_search_near = MAX_FRQ;
            }
            
            UltraSonicTimeChange(frq_search_near);
            
            //Ultra_IO_Output();///////
            //UltraIoOutputHigh();//////
            delay_ms(1);////////////
            status = UltraSonicReadStatusStart();
        }
        if(cnt <= SEARCH_FRQ_COMPENSATION * 2 / ULTRA_SEARCH_FRQ_STEP_SIZE)
        {
            return status;
        }
    }
    else
    {
        return status;
    }
    return ULTRASONIC_READ_ERR;
    
}

uint32_t test_status;
uint32_t test_cnt = 0;
void UltraSonicSetThreshold(ultra_sonic_threshold_t * threshold)
{
    uint32_t parity_0_data = (threshold->threshold[12]&0x03) | (threshold->threshold[13]<<2) | (threshold->threshold[14]<<7) | \
                                (threshold->tres_scale<<12) | (threshold->thres_ini<<14) | (threshold->thres_len<<15);
    uint32_t parity_1_data = (threshold->threshold[9]&0x07) | (threshold->threshold[10]<<3) |(threshold->threshold[11]<<8) |\
                                (((threshold->threshold[12]&0x1c)>>2)<<13);
    uint32_t parity_2_data = (threshold->threshold[6]&0x0f) | (threshold->threshold[7]<<4) | (threshold->threshold[8]<<9) |\
                                (((threshold->threshold[9]&0x18)>>3)<<14);
    uint32_t parity_3_data = (threshold->threshold[3]) | (threshold->threshold[4]<<5) | (threshold->threshold[5]<<10) |\
                                (((threshold->threshold[6]&0x10)>>4)<<15);
    uint32_t parity_4_data = (threshold->threshold[1]) | (threshold->threshold[2]<<5);
    uint32_t status = 0;
    uint8_t cnt = 0;
    uint32_t frq_tmp = estimate_frq;
    
    threshold->parity_0 = UltraSonicEvenParityCompute(parity_0_data, 16);
    threshold->parity_1 = UltraSonicEvenParityCompute(parity_1_data, 16);
    threshold->parity_2 = UltraSonicEvenParityCompute(parity_2_data, 16);
    threshold->parity_3 = UltraSonicEvenParityCompute(parity_3_data, 16);
    threshold->parity_4 = UltraSonicEvenParityCompute(parity_4_data, 10);
    
    status = UltraSonicWriteThresholdData(threshold);
    while( ((status & 0x02) == 0) || (status == ULTRASONIC_READ_ERR))  
    //while(((status & 0x80) == 0) || (status == ULTRASONIC_READ_ERR))
    {   
        //delay_ms(10);
        //status = UltraSonicReadStatus();
        //delay_ms(10);
        cnt++;
        if(cnt >= 250)
        {
            UltraSonicLog("FATAL: CAN NOT WRITE THRESHOLD VALUE ! ! ! ! ! \r\nSYSTEM WILL RESET!\r\n");
            platform_mcu_reset();
        }

        
        if(frq_tmp > MIN_FRQ)
        {
            frq_tmp -= ULTRA_SEARCH_FRQ_STEP_SIZE;
        }
        else
        {
            frq_tmp = MAX_FRQ;
        }
        delay_ms(1);
        UltraSonicTimeChange(frq_tmp);
        status = UltraSonicWriteThresholdData(threshold);
    }
    
    test_status = status;
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
