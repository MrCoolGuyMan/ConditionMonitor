/* 
 * File:   Accelerometer.h
 * Author: Administrator
 *
 * Created on 1 August, 2022, 3:44 PM
 */

#ifndef ADXL345_H
#define	ADXL345_H

#include "SPI_Interface.h"
#include "/ObjectDictionary/CAN Object Dictionary.h"
#include <stdint.h>
#include "Sensors.h"
#include "ObjectDictionary.h"

#define ACC_SPI_SETTINGS        0,1,_4MHz_CLK

#ifdef	__cplusplus
extern "C" {
#endif
void setup_ACCL(void);
void Format_ACCL_Data(uint16_t axis_accl_data,uint8_t direct_CAN_Transmission,uint8_t Sensor_ID_axis);

uint16_t Read_ACCL_Sensor(SensorClass Sensor,uint8_t direct_CAN_Transmission );

uint16_t TwosComplement(uint16_t Data);
#ifdef	__cplusplus
}
#endif

#endif	/* ADXL345_H */

