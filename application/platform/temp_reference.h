/* 
*  Author: Adam Huang
*  Date:2016/12/22
*/

#ifndef __TEMP_REFERENCE_H__
#define __TEMP_REFERENCE_H__

#pragma once

#include <stdint.h>

int16_t get_ntc_temp_from_voltage( uint16_t voltage );
uint8_t get_percentage_from_battery_voltage( uint16_t battery_voltage );

#endif

