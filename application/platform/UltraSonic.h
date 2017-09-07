
#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#include "mico.h"



//////////define the time init value//////////
///   uint:  us  ////
#define F_DRV			    52000
#if 1
#define T_DEB               0
#define T_SND 				(uint32_t)(6*1000*1000/F_DRV)  //  
#define T_REC 				(uint32_t)(12*1000*1000/F_DRV)
#define T_CMD 				(uint32_t)(18*1000*1000/F_DRV)
#define T_CMD_PROG 		    (uint32_t)(18*1000*1000/F_DRV)
#define T_D 				(uint32_t)(3*1000*1000/F_DRV)
#define T_VPROG 			(uint32_t)(5000)
#define T_PROG 				(uint32_t)(20000)
#define T_0_LOW				(uint32_t)(6*1000*1000/F_DRV)
#define T_0_HIGH 			(uint32_t)(6*1000*1000/F_DRV)
#define T_1_LOW 			(uint32_t)(3*1000*1000/F_DRV)
#define T_1_HIGH 			(uint32_t)(9*1000*1000/F_DRV)
#define T_BIT 				(uint32_t)(12*1000*1000/F_DRV)
#define T_BIT0 				(uint32_t)(6*1000*1000/F_DRV)
#define T_BIT1 				(uint32_t)(3*1000*1000/F_DRV)
#define T_CAL 				(uint32_t)(12*1000*1000/F_DRV)
#endif



//////////define the destination frequency and allowed errors
#define DEST_FRQ                    52000
#define DEST_FRQ_ERROR              250

#define MIN_FRQ                     30000
#define MAX_FRQ                     70000

#define ULTRA_FRQ_STEP_SIZE         300
#define ULTRA_SEARCH_FRQ_STEP_SIZE  300 


#define SEARCH_FRQ_COMPENSATION     1000


#define MEASURE_BLIND_DISTANCE      12


#define T_0_LOW_MAX         t_0_low*1.3 
#define T_0_LOW_MIN         t_0_low*0.7
#define T_0_HIGH_MAX
#define T_0_HIGH_MIN

#define T_1_LOW_MAX         t_1_low*1.3
#define T_1_LOW_MIN         t_1_low*0.7
#define T_1_HIGH_MAX
#define T_1_HIGH_MIN


#define NO_OBJ_DETECTED                 0x00000000
#define DANGER_DISTANCE                 50
#define DANGER_DISTANCE_FILTER_CNT      2
typedef enum
{
    US_CMD_SEND_REQUEST,
    US_CMD_RECEIVE_REQUEST,
    US_CMD_THRESHOLD_PROGRAM = 0x01,
    US_CMD_READ_STATUS = 0x02,
    US_CMD_EE_WRITE = 0x05,
    US_CMD_EE_READ = 0x04,
    US_CMD_EE_PROGRAM = 0x06,
    US_CMD_MEASURE_CONFIG,
    US_CMD_FREQUENCY_CALIBRARION = 0x03,    
}ultra_sonic_cmd_t;


 
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
}ultra_sonic_data_t;

typedef struct 
{
    uint32_t start_time;
    uint32_t end_time;
    uint32_t interval_time;
    uint32_t time_cnt;
    uint8_t start_flag;
    uint8_t over_time_flag;
}ultra_sonic_frq_calibration_t;

typedef struct
{
    uint32_t low_start_time;
    uint32_t low_end_time;
    //uint32_t high_start_time;
    //uint32_t high_end_time;
}ultra_sonic_read_data_t;

typedef struct
{
    uint16_t status;
    uint8_t start_flag;
    uint8_t cnt;
#define ULTRA_READ_EE     (1<<0)   
}ultra_sonic_status_t;

#define THRES_NUM   15
typedef struct 
{ 
    uint32_t threshold[THRES_NUM];
    uint32_t tres_scale;
    uint32_t thres_ini;
    uint32_t thres_len;
    uint32_t parity_0;
    uint32_t parity_1;
    uint32_t parity_2;
    uint32_t parity_3;
    uint32_t parity_4;   
}ultra_sonic_threshold_t;

typedef struct
{
#if 0
    //volatile uint32_t nothing     :12;
    volatile uint32_t amp_gain    :4;   
    volatile uint32_t drv_cur     :4;
    volatile uint32_t f_drv_adj   :7;
    volatile uint32_t filt_adjust :1;
    volatile uint32_t noise_cfg   :2;
    volatile uint32_t io_mask     :1;
    volatile uint32_t spare_bit   :1;
#endif
#if 1
    volatile uint32_t amp_gain          :5;   
    volatile uint32_t vdrv_cfg          :4;
    volatile uint32_t f_drv_adj         :8;
    volatile uint32_t n_pulse           :1;
    volatile uint32_t el_damp           :4;
    volatile uint32_t rd_cfg            :1;
    volatile uint32_t comp_mask         :1;
    volatile uint32_t atg_cfg           :1;
    volatile uint32_t noise_cfg         :2;
    volatile uint32_t customer_bits     :5;
#endif
            
}ultra_sonic_ee_data_t;

typedef union
//typedef struct
{
    ultra_sonic_ee_data_t ee_data;
    volatile uint32_t value;
}ultra_sonic_ee_data_ut;

#if 0
typedef enum
{
    //spare_bit_bit_num = 0,
    SPARE_BIT_BIT_NUM = 0,
    //io_mask_bit_num = 1,
    IO_MASK_BIT_NUM = 1,
    //noise_cfg_bit_num = 2
    NOISE_CFG_BIT_NUM = 2,
    //filt_adjust_bit_num = 4,
    FILT_ADJUST_BIT_NUM = 4,
    //f_drv_adj_bit_num = 5,
    F_DRV_ADJ_BIT_NUM = 5,
    //drv_cur_bit_num = 12,
    DRV_CUR_BIT_NUM = 12,
    //amp_gain_bit_num = 16
    AMP_GAIN_BIT_NUM = 16
}ultra_sonic_ee_data_bit_num_e;

typedef enum
{
    //spare_bit_mask = 1<<SPARE_BIT_BIT_NUM,
    SPARE_BIT_MASK = 1<<SPARE_BIT_BIT_NUM,
    //io_mask_mask = 1<<IO_MASK_BIT_NUM,
    IO_MASK_MASK = 1<<IO_MASK_BIT_NUM,
    //noise_cfg_mask = 3<<NOISE_CFG_BIT_NUM,
    NOISE_CFG_MASK = 3<<NOISE_CFG_BIT_NUM,
    //filt_adjust_mask = 1<<FILT_ADJUST_BIT_NUM,
    FILT_ADJUST_MASK = 1<<FILT_ADJUST_BIT_NUM,
    //f_drv_adj_mask = 0x7f<<F_DRV_ADJ_BIT_NUM,
    F_DRV_ADJ_MASK = 0x7f<<F_DRV_ADJ_BIT_NUM,
    //drv_cur_mask = 0x0f<<DRV_CUR_BIT_NUM,
    DRV_CUR_MASK = 0x0f<<DRV_CUR_BIT_NUM,
    //amp_gain_mask = 0x0f<<AMP_GAIN_BIT_NUM
    AMP_GAIN_MASK = 0x0f<<AMP_GAIN_BIT_NUM
}ultra_sonic_ee_data_mask_e;
#endif

typedef struct
{
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
}ultra_sonic_time_t;

void UltraSonicStart(void); 
//extern uint32_t UltraSonicReadEE(void);
extern uint32_t UltraSonicReadEE(ultra_sonic_ee_data_ut *ee_data);
extern uint32_t UltraSonicFrqCalibrationStart(void);
extern uint32_t UltraSonicReadStatus(void);
extern void V24OutputHigh(void);
extern void UltraSonicInit(void);
extern void UltraSonicWriteEE(ultra_sonic_ee_data_ut* data);
extern void UltraSonicPrgramEE(void);
extern uint32_t UltraReadCalibrationData(void);
extern void ShowTestLog(void);
//extern uint32_t UltraSonicReadEEStart(void);
extern uint32_t UltraSonicReadEEStart(ultra_sonic_ee_data_ut * data);
extern uint32_t UltraSonicFrqCalibration(void);
extern void UltraSonicSetFrqToDest(void);
extern void UltraSonicSetThreshold(ultra_sonic_threshold_t * threshold);
extern void UltraSonicDataTick(void);
extern uint16_t UltraSonicGetMeasureData(void);

extern ultra_sonic_data_t *ultra_sonic_data;
extern ultra_sonic_status_t * ultra_sonic_status;
extern ultra_sonic_read_data_t * ultra_sonic_read_data;
extern ultra_sonic_frq_calibration_t *ultrasonic_frq_calibration;
extern volatile uint32_t estimate_frq;

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
//#else

#define delay_ms          HAL_Delay
#endif
#endif