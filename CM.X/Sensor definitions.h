/* 
 * File:   ADC.h
 * Author: Administrator
 *
 * Created on 24 November, 2021, 12:50 PM
 */

#ifndef SENSOR_DEFINITIONS_H
#define	SENSOR_DEFINITIONS_H

#include <stdint.h>
#include <stdbool.h>

#define CURRENT_SENSOR_START_PIN 2          // Analog channel
#define TEMPERATURE_SENSOR_START_PIN 22     // Analog Channel
#define ACCL_SENSOR_START_PIN 40            // Virtual
#define HUMID_SENSOR_START_PIN 60           // Virtual
#define CAN_INTERRUPT_PIN                   PORTBbits.RB2
//NumberOfSensorsInSensorList must be the final value
enum SensorListPositions {TempSensor=0,CurrentSensor=1,VibrationSensor=2,HumiditySensor=3,Encoder1=4,Encoder2=5, NumberOfSensorsInSensorList};
//bit shifted because of eeprom formatting
#define MetaDataOffsetAmount            5
#define Temperature_Sensor_type         (TempSensor<<MetaDataOffsetAmount)         //0
#define Current_Sensor_type             (CurrentSensor<<MetaDataOffsetAmount)      //1
#define Vibration_Sensor_type           (VibrationSensor<<MetaDataOffsetAmount)    //2
#define Humidity_Sensor_type            (HumiditySensor<<MetaDataOffsetAmount)     //5
#define CAN_MESSAGE_as_sensor_type      7
#define Encoder_Sensor_type             Encoder1
#define Encoder_Meta_Data               (unsigned) (Encoder1 << (MetaDataOffsetAmount+8)) 
#define Encoder2_Meta_Data              (unsigned) ((Encoder1+1) << (MetaDataOffsetAmount+8)) 
#define CAN_MSG_Meta_Data               (CAN_MESSAGE_as_sensor_type << MetaDataOffsetAmount)
//SensorData bit shifted for Meta Data Formatting
#define X_AXIS (1<<2)
#define Y_AXIS (2<<2)
#define Z_AXIS (3<<2)
#define HUMIDITY (4<<2)


#define EEPROM_OUTPUT_MODE  0
#define CAN_OUTPUT_MODE_1 1
#define CAN_OUTPUT_MODE_2 2
//for sensors that are constantly read during the Monitoring mode operation

#endif	/* SENSORS_H */

