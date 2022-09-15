/* 
 * File:   Global Variables.h
 * Author: Administrator
 *
 * Created on 7 December, 2021, 11:30 AM
 */

#ifndef GLOBAL_VARIABLES_H
#define	GLOBAL_VARIABLES_H

#include "CustomDataTypes.h"


    //flags indicate the current operating mode
    ProgramFlags OperatingModeFlags ={};
    uint8_t Assigned_Node_ID;
    //SDO Protocol
    struct C_OD_ENTRY Last_accessed_object;     //stores the Can Object from the previous Multi-read SDO request
    
    //timing flags
    TimingFlags Timers ={};

    //motor controller 
    uint8_t EncoderOverflows;       //Motor encoder is connected to Timer1 (16 bit value), this acts as the upper 8 bits, giving us a 24 bit encoder reading
    uint8_t EncoderOverflows2;      //Motor encoder is connected to Timer3 (16 bit value), this acts as the upper 8 bits, giving us a 24 bit encoder reading     
    bool MotorAtMaxSpeed = false;
    
    //uses  LATAbits.LA4    //Dir1
    //      LATAbits.LA6    //dir2
    //      LATCbits.LC1    //dir3
   
    MotorClass DCMotor =
    {
        .Settings =         {   
                                .CurrentDutyCycle       =   0,
                                .MaxDutycyle            =   255,
                                .Direction              =   0,
                                .StatusUpdateRate       =   100
                            },
        .Connections =      {  
                                .directionPinLatch      =   &LATB,
                                .direction1PinNumber    =   7,
                                .enablePinLatch         =   &LATC,
                                .enablePinNumber        =   1
                            },
        .Records =          {}
    };
    //sensors
    uint16_t Sensor_Checking_Interval=Sampling_period_TMR_Interrupts;

    uint8_t MonitoringModeOutput = EEPROM_OUTPUT_MODE;
    uint24_t MonitoringModeStartTime;
    //holds list of sensors to be measured during monitoring process
    SensorClass SensorList[NumberOfSensorsInSensorList];
    
    //holds data intended for external EEPROM chip
    LOG_DATA_BUFFER  SENSOR_DATA_BUFFER={};
    
    uint8_t oldSensorType=0;
    uint8_t oldSensor_id=0;

    uint8_t oldADRESH=0;
    uint8_t oldADRESL=0;
#endif	/* GLOBAL_VARIABLES_H */

