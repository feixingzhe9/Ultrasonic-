/* 
*  Author: Adam Huang
*  Date:2016/12/22
*/


#include "temp_reference.h"

typedef struct _ntc_temp_voltage_mapping_t {
  signed short          temp;
  unsigned short        voltage;// X 0.0001V
} ntc_temp_voltage_mapping_t;

const static ntc_temp_voltage_mapping_t temp_voltage_mapping[] = {
  { -40,        16416},
  { -35,        16389},
  { -30,        16355},
  { -25,        16313},
  { -20,        16261},
  { -15,        16198},
  { -10,        16120},
  {  -5,        16027},
  {   0,        15915},
  {   5,        15783},
  {  10,        15628},
  {  15,        15447},
  {  20,        15238},
  {  25,        15000},
  {  30,        14728},
  {  35,        14424},
  {  40,        14085},
  {  45,        13711},
  {  50,        13302},
  {  55,        12861},
  {  60,        12389},
  {  65,        11898},
  {  70,        11388},
  {  75,        10858},
  {  80,        10317},
  {  85,         9770},
  {  90,         9224},
  {  95,         8680},
  { 100,         8141},
  { 105,         7619},
  { 110,         7114},
  { 115,         6631},     
  { 120,         6161}, 
  { 125,         5722},   
  {   0,            0},
};

int16_t get_ntc_temp_from_voltage( uint16_t voltage )
{
  uint16_t voltage_temp;
  int16_t convert_temp;
  int i;
  
  voltage_temp = voltage * 10;
  for( i = 0; ; i++ )
  {
    if( !temp_voltage_mapping[i].temp && !temp_voltage_mapping[i].voltage )
    {
      convert_temp = 125;
      break;
    }
    if( temp_voltage_mapping[i].voltage <= voltage_temp )
    {
      convert_temp = temp_voltage_mapping[i].temp;
      break;
    }
  }
  return convert_temp;
}

typedef struct _battery_voltage_percentage_mapping_t {
  unsigned char         percentage;
  unsigned short        voltage;// X 0.0001V
} battery_voltage_percentage_mapping_t;









#if 0
const static battery_voltage_percentage_mapping_t battery_voltage_percentage_mapping[] = 
{
  { 100, 25943},
  {  99, 25728},
  {  98, 25708},
  {  97, 25696},
  {  96, 25689},
  {  95, 25682},
  {  94, 25673},
  {  93, 25668},
  {  92, 25660},
  {  91, 25655},
  {  90, 25649},
  {  89, 25643},
  {  88, 25638},
  {  87, 25632},
  {  86, 25625},
  {  85, 25619},
  {  84, 25611},
  {  83, 25605},
  {  82, 25598},
  {  81, 25593},
  {  80, 25584},
  {  79, 25577},
  {  78, 25570},
  {  77, 25562},
  {  76, 25554},
  {  75, 25545},
  {  74, 25537},
  {  73, 25528},
  {  72, 25518},
  {  71, 25509},
  {  70, 25500},
  {  69, 25490},
  {  68, 25481},
  {  67, 25470},
  {  66, 25459},
  {  65, 25450},
  {  64, 25441},
  {  63, 25431},
  {  62, 25422},
  {  61, 25413},
  {  60, 25401},
  {  59, 25393},
  {  58, 25384},
  {  57, 25376},
  {  56, 25368},
  {  55, 25361},
  {  54, 25352},
  {  53, 25344},
  {  52, 25336},
  {  51, 25331},
  {  50, 25324},
  {  49, 25315},
  {  48, 25306},
  {  47, 25299},
  {  46, 25290},
  {  45, 25283},
  {  44, 25276},
  {  43, 25267},
  {  42, 25256},
  {  41, 25249},
  {  40, 25238},
  {  39, 25228},
  {  38, 25219},
  {  37, 25208},
  {  36, 25196},
  {  35, 25185},
  {  34, 25171},
  {  33, 25158},
  {  32, 25144},
  {  31, 25132},
  {  30, 25115},
  {  29, 25099},
  {  28, 25082},
  {  27, 25066},
  {  26, 25046},
  {  25, 25032},
  {  24, 25010},
  {  23, 24989},
  {  22, 24969},
  {  21, 24949},
  {  20, 24923},
  {  19, 24902},
  {  18, 24878},
  {  17, 24852},
  {  16, 24825},
  {  15, 24796},
  {  14, 24766},
  {  13, 24736},
  {  12, 24707},
  {  11, 24675},
  {  10, 24643},
  {   9, 24608},
  {   8, 24572},
  {   7, 24535},
  {   6, 24494},
  {   5, 24453},
  {   4, 24398},
  {   3, 24334},
  {   2, 24250},
  {   1, 24145},
  {   0, 24010},
};
#endif

const static battery_voltage_percentage_mapping_t battery_voltage_percentage_mapping[] = 
{
  { 100, 27380},
  
  {  95, 26540},
  
  {  90, 26530},
  
  {  85, 26520},
  
  {  80, 26510},
 
  {  75, 26500},
  
  {  70, 26460},
  
  {  65, 26380},
  
  {  60, 26260},
 
  {  55, 26200},
  
  {  50, 26190},
  
  {  45, 26150},
  
  {  40, 26140},
  
  {  35, 26140},
  
  {  30, 26060},
  
  {  25, 26030},
  
  {  20, 25890},
  
  {  15, 25750},
  
  {  10, 25540},
  
  {   5, 25430},
 
  {   0, 24930}
};



uint8_t get_percentage_from_battery_voltage( uint16_t battery_voltage )
{
  uint16_t battery_voltage_temp;
  uint8_t percentage_temp;
  int i;
  
  battery_voltage_temp = battery_voltage;
  if( battery_voltage_temp >= battery_voltage_percentage_mapping[0].voltage )
  {
    percentage_temp = battery_voltage_percentage_mapping[0].percentage;
  }
  else if( battery_voltage_temp <= battery_voltage_percentage_mapping[sizeof(battery_voltage_percentage_mapping)/sizeof(battery_voltage_percentage_mapping[0]) - 1].voltage )
  {
    percentage_temp = battery_voltage_percentage_mapping[sizeof(battery_voltage_percentage_mapping)/sizeof(battery_voltage_percentage_mapping[0]) - 1].percentage;
  }
  else
  {
    for( i = 0; i < sizeof(battery_voltage_percentage_mapping)/sizeof(battery_voltage_percentage_mapping[0]) - 1; i++ )
    {
      if( battery_voltage_temp >= battery_voltage_percentage_mapping[i+1].voltage && 
         battery_voltage_temp <= battery_voltage_percentage_mapping[i].voltage )
      {
        percentage_temp = battery_voltage_percentage_mapping[i+1].percentage;
        break;
      }
    }
  }
  return percentage_temp;
}

