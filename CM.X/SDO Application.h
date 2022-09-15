/* 
 * File:   SDO Application.h
 * Author: Administrator
 *
 * Created on 2 August, 2022, 3:30 PM
 */

#ifndef SDO_APPLICATION_H
#define	SDO_APPLICATION_H

#ifdef	__cplusplus
extern "C" {
#endif
    
    
#include <stdint.h>
#include "CustomDataTypes.h"
#include "/ObjectDictionary/CAN Object Dictionary.h"
#include "../CM.X/Sensors.h"
 

    extern uint8_t Assigned_Node_ID;

    extern void factory_reset(void);
    extern void Reset_CAN_CONTROLLER(void);
    
    extern void setup_heart_beat_command(void);
 

// this is our psuedo PDO protocol
// if you want something to be processed after an SDO write
// then add it here
void Process_SDO_REQUEST(uint24_t Index,uint8_t SDOMsg[8]);

#ifdef	__cplusplus
}
#endif

#endif	/* SDO_APPLICATION_H */

