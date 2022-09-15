/* 
 * File:   CAN Object Dictionary.h
 * Author: Administrator
 *
 * Created on 18 January, 2021, 2:14 PM
 */

#ifndef CAN_OBJECT_DICTIONARY_H
#define	CAN_OBJECT_DICTIONARY_H

//#include <xc.h>
#include <stdint.h>
#include "ConditionMonitorConfigFile.h"
#include "CustomDataTypes.h"
#include "InternalEEPROM.h"
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

//Operating Modes TRANSFER_SENSOR_DATA_FROM_EEP_INDEX
#define ENTER_SENSOR_MONITORING_MODE    0x200300
#define SENSOR_MONITORING_OUTPUT_MODE   0x200301
#define ENTER_CAN_BUS_MONITORING_MODE   0x200500
#define SENSOR_LOGGING_RATE_CAN_INDEX   0x201000
//eeprom
#define TOTAL_EEPROM_MEM_USAGE_INDEX    0x200602


#define NODE_ID_CAN_INDEX               0x2F0001


//tests

#define MOTOR_MAX_DUTY_CYCLE_CAN_INDEX  0x300001
#define MOTOR_DIRECTION_CAN_INDEX       0x300002

#define MOTOR_ENCODER_ENABLED_CAN_INDEX 0x300004

///populate the C_OD_ENTRY struct from EEPROM; 
///keep track of current EEPROM address
///write default values to EEPROM during factory reset
void  CHECK_COD_ENTRY(  CAN_INDEX_TYPE CAN_INDEX,CAN_INDEX_TYPE REFERENCE_CAN_INDEX,
                        uint8_t *Mem_Address, uint8_t default_value[],
                        uint8_t mem_usage,uint32_t max_value,uint32_t min_value,
                        bool Factory_Reset_Enabled,struct C_OD_ENTRY *OBJECT_DATA );
///populate C_OD_ENTRY struct values
void  setup_COD_data(uint8_t Mem_Address, uint8_t default_value[],uint8_t mem_usage,uint32_t max_value,uint32_t min_value,struct C_OD_ENTRY *OBJECT_DATA);
///converts a can object dictionary entry index to an EEPROM memory address
struct C_OD_ENTRY FIND_CAN_OBJECT(CAN_INDEX_TYPE CAN_INDEX,bool Factory_Reset_Enabled);
//Allows writing either a specified value "Data_To_Write" or a default value Data_To_Write=NULL
//using the CAN Address
//this is the one folks should use from external files
ERROR_CODE Edit_COD(CAN_INDEX_TYPE CAN_INDEX,uint8_t Data_to_Write[8], uint8_t data_length) ;
//Implements the Write Request from above
//you can call this directly if you already have "Object_Data"
//if you want to write the default value you need to use Edit_COD
//or call function like this Write_to_Code (Object,Object.default_value,Object.length)
ERROR_CODE Write_to_COD(struct C_OD_ENTRY Object_Data, uint8_t Data_to_Write[8], uint8_t data_length);
/************************
read EEPROM address coded as Object-Dictionary address and return value
 * 
 * check_current_value_of_8bit_OD_entry exists to save having to mask with 0xff, as most parameters are 8-bit
************************/
uint8_t check_current_value_of_8bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds);
uint16_t check_current_value_of_16bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds);
uint32_t check_current_value_of_32bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds);


#endif	/* CAN_OBJECT_DICTIONARY_H */

