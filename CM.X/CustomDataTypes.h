/* 
 * File:   CustomDataTypes.h
 * Author: Administrator
 *
 * Created on 2 December, 2021, 4:10 PM
 */

#ifndef CUSTOMDATATYPES_H
#define	CUSTOMDATATYPES_H

#include <stdbool.h>
#include <stdint.h>
typedef uint24_t CAN_INDEX_TYPE ;
typedef uint8_t ERROR_CODE;
typedef uint8_t tristate;


    /*
     * flags and timers
     */
        typedef struct
        {
            bool CAN_MonitoringModeActive;
            tristate MotorTestEnabled;      //1 = Active, 0 = Halting, 2 = Inactive
       //     bool MonitoringModeActive;
        }ProgramFlags;
                typedef struct
        {
            uint16_t SensorFlag;            //counts milliseconds since last sensor reading
            uint16_t HeartBeat_FLAG;        //counts milliseconds since last heartbeat message
            uint24_t OperatingTime_ms;      //counts milliseconds since power on
            uint16_t EncoderWriteFlag;      //counts milliseconds since last encoder value was transmitted
            uint16_t MotorAccelerateflag;    //counts milliseconds since motor acceleration was last altered

        }TimingFlags;

    //CAN-Object Dictionary
        struct C_OD_ENTRY
        {
            uint32_t max_value;
            uint32_t min_value; 
            uint8_t default_value[10];
            uint8_t read_index;
            uint8_t Start_Address;
            uint8_t length;
       };

        
    
    //EEPROM chip takes 6 ms to complete a write, the write size can be 1 - 256 bytes
    //so we will write the max number of bytes at once for max sampling rate
    //we need to balance this against the time it takes to dump a large amount of data at once
    //until then we store the data in a software buffer
    //the buffer will be dumped whenever currentSize rolls over, or when monitoring mode is turned off



#endif	/* CUSTOMDATATYPES_H */

