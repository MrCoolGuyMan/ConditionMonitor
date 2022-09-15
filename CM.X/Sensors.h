/* 
 * File:   ADC.h
 * Author: Administrator
 *
 * Created on 24 November, 2021, 12:50 PM
 */

#ifndef SENSORS_H
#define	SENSORS_H
#include "CustomDataTypes.h"
#include "Sensor definitions.h"
#include "/23LC1024 SPI RAM/_23LC1024 Driver.h"


#include "/PIC18F Peripheral Drivers/SPI_Interface.h"
#include "/PIC18F Peripheral Drivers/Analog.h"
#include <stdint.h>
#include <stdbool.h>
#include "ConditionMonitorConfigFile.h"
#include "/CAN Protocols/ObjectDictionary/CAN Object Dictionary.h"
#include "/IC Drivers/MCP2515 CAN Controller/MCP2515.h"

//for sensors that are constantly read during the Monitoring mode operation

  extern void convertHexToDecAndTransmit(uint8_t identifier,uint32_t InputValue, bool TransmitOnCompletion);  
	typedef struct SensorType
	{
		uint8_t Sensor_Type;
		uint8_t number_of_connected_sensors;
		uint8_t Starting_Sensor_Number;
		uint8_t SensorReadings;
		uint24_t number_of_connected_sensors_index;
		uint16_t (*Sensorfunction)(struct SensorType,uint8_t); //this is called a function pointer
																 //the function is the one that will read the sensor value during the monitoring process
																 //it needs to accept an EEPROM address for storing the sensor data into EEPROM
        void (*SetupFunction)(void);
	}SensorClass;
    typedef uint16_t (*Sensorfunction_t)(struct SensorType,uint8_t);
    typedef void (*Sensorsetupfunction_t)(void);
    
    void AddSensor(uint8_t Type, uint8_t startingID, uint24_t COD_INDEX_NUM_CONNECTED, Sensorfunction_t Callback,Sensorsetupfunction_t SetupFunc);
    
void initSensorList(void);
void setupSensorListCODParameters(void);

uint16_t ReadADCSensor(SensorClass Sensor,uint8_t direct_CAN_Transmission);
void SensorHeader(uint8_t SensorListPosition,uint8_t direct_CAN_Transmission);
void ConvertADCToCurrent(int32_t *newdata);

void Transmit_Sensor_Data(uint16_t TX_Data, uint8_t Addr4, uint8_t Addr3, uint8_t Addr2, uint8_t Addr1);

uint16_t CopyTimerToEncoderValue(volatile uint16_t *Timer,uint8_t direct_CAN_Transmission, uint16_t MetaData);

void transmit_monitoring_summary(void);

void convertSensorFrequencyToClockMultiple(void);
void setup_monitoring_mode(uint8_t Mode);
void transmit_readable_sensor_data(uint32_t OutputData, char sensor_identifier, uint8_t Sensor_Type);
void FormatSensorMetaData(  uint8_t direct_CAN_Transmission,
                            uint8_t Data_MostSig2bits,              //upper 2 bits of 10 bit value; i.e. ADRESH
                            uint8_t Data_LSB,                       //lower 8 bits of 10 bit value; i.e. ADRESL
                            uint8_t Sensor_Type,                    //0-7 bit shifted << 5; forms bits 7:5 of MSB
                            uint8_t sensor_id_with_offset)          //0-7 bit shifted << 3; forms bits 4:2 of MSB
                                                                    ;
void store_CAN_MSG(uint8_t msg_marker, uint8_t data_value);
void log_CAN_MESSAGE_TO_EEPROM( uint8_t Data[8],uint8_t DLC, uint16_t Node_ID);

void Turn_off_Sensor_Monitoring(void);
void Begin_sensor_monitoring(bool realTimeTransfer);
void transferEntireRam(uint8_t data[8], uint8_t *DLC, uint16_t *NodeID);
void TransmitRawSensorData(uint8_t *TX_Data);
bool CheckIfSensorIntervalTimeHasArrived(void);

uint8_t checkMonitoringActive(void);


uint16_t StoreEncoder1(SensorClass Sensor,uint8_t direct_CAN_Transmission);
uint16_t StoreEncoder2(SensorClass Sensor,uint8_t direct_CAN_Transmission);
#endif	/* SENSORS_H */

