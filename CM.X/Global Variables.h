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
    
    
    //timing flags
    TimingFlags Timers ={};

    //motor controller 
    uint8_t EncoderOverflows;       //Motor encoder is connected to Timer1 (16 bit value), this acts as the upper 8 bits, giving us a 24 bit encoder reading
    uint8_t EncoderOverflows2;      //Motor encoder is connected to Timer3 (16 bit value), this acts as the upper 8 bits, giving us a 24 bit encoder reading     
    

    
    
    
#endif	/* GLOBAL_VARIABLES_H */

