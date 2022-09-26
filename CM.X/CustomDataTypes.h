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

typedef uint8_t ERROR_CODE;
typedef uint8_t tristate;


/*
 * flags and timers
 */


    typedef struct
    {
        uint16_t SensorFlag;            //counts milliseconds since last sensor reading
        uint16_t HeartBeat_FLAG;        //counts milliseconds since last heartbeat message
        uint24_t OperatingTime_ms;      //counts milliseconds since power on
        uint16_t EncoderWriteFlag;      //counts milliseconds since last encoder value was transmitted
        uint16_t MotorAccelerateflag;    //counts milliseconds since motor acceleration was last altered

    }TimingFlags;


#endif	/* CUSTOMDATATYPES_H */

