/* 
 * File:   CAN Object Dictionary.h
 * Author: Administrator
 *
 * Created on 18 January, 2021, 2:14 PM
 */

#ifndef OBJECT_DICTIONARY_H
#define	OBJECT_DICTIONARY_H

#define DEVICE_TYPE                     0x100000 
#define DEVICE_NAME_INDEX               0x100800
#define HARDWARE_VERSION_INDEX          0x100900
#define VERSION_NUMBER_INDEX            0x100A00
#define FACTORY_RESET_CAN_INDEX         0x101101 
#define PRODUCT_CODE_INDEX              0x101802 
#define SERIAL_NUMBER_INDEX             0x101804
#define PROGRAM_CONTROL_NUM_ENTRIES     0x1F5100
#define PROGRAM_CONTROL_CAN_INDEX       0x1F5101
//sensors
#define SENSOR_NOT_IN_OBJECT_DICTIONARY         0xFFFFFF    //used for sensors not needing to be logged in EEPROM
#define TEMP_AMNT_TRANSFER_FLAG_INDEX           0x200001   
#define NUM_ADCS_TRANSFER_FLAG_INDEX            0x200002 
#define CRNT_AMNT_TRANSFER_FLAG_INDEX           0x200101   
#define CRNT_OFFSET_VOLTAGE_FLAG_INDEX          0x200103 
#define ACCL_AMNT_TRANSFER_FLAG_INDEX           0x200401
#define ACCL_SAMPLE_TRANSFER_FLAG_INDEX         0x200402
#define ACCL_ORIENTATION_TRANSFER_FLAG_INDEX    0x200403
#define HUMID_SENS_TRANSFER_FLAG_INDEX          0x200404
#define STEP_MOTOR_TRANSFER_FLAG_INDEX          0x200407
//not used yet
#define HALL_EFFECT_TRANSER_FLAG_INDEX          0x200500
#define HALL_EFFECT_AMNT_FLAG_INDEX             0x200501
#define HALL_EFFECT_ADCS_FLAG_INDEX             0x200502 

#define CAN_BAUD_CAN_INDEX              0x200300
#define SENSOR_MONITORING_OUTPUT_MODE   0x200301
#define SENSOR_LOGGING_RATE_CAN_INDEX   0x201000
//eeprom
#define TOTAL_EEPROM_MEM_USAGE_INDEX    0x200602


#define NODE_ID_CAN_INDEX               0x2F0001


//tests

#define MOTOR_MAX_DUTY_CYCLE_CAN_INDEX  0x300001
#define MOTOR_DIRECTION_CAN_INDEX       0x300002

#define MOTOR_ENCODER_ENABLED_CAN_INDEX 0x300004

#endif	

