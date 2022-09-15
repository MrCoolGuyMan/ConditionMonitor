/* 
 * File:   MotorControl.h
 * Author: Administrator
 *
 * Created on 22 November, 2021, 12:01 PM
 */

#ifndef MOTORCONTROL_H
#define	MOTORCONTROL_H

#include "CustomDataTypes.h"

void SendEncoderData(void)
{
    if(DCMotor.Settings.StatusUpdateRate==0)    //user has disabled 
    {
        return;
    }
    
    if(Timers.EncoderWriteFlag>=DCMotor.Settings.StatusUpdateRate)
    {   
        Timers.EncoderWriteFlag=0;
        uint24_t temp_EncoderCh = TMR1+ ((uint24_t)EncoderOverflows<<16);
        TMR1=0;
        EncoderOverflows=0;
          
        if(MotorAtMaxSpeed == false)
        {
            MotorAtMaxSpeed = true;
        }
        else 
        {
            if(DCMotor.Settings.encoderConnected == true)
            {
                if(temp_EncoderCh==0)   //collision error
                {
                    turnoffMotorControl();
                    DCMotor.Settings.CurrentDutyCycle = 0;
                }
                compare_minmax(temp_EncoderCh,&DCMotor.Records.Min_Encoder_Reading,&DCMotor.Records.Max_Encoder_Reading); 

                convertHexToDecAndTransmit('E',temp_EncoderCh,true);    //E for encoder
            }

            uint16_t RMS_CURRENT = (ReadADCSensor(SensorList[CurrentSensor],2))&0x3ff;
            
            compare_minmax(RMS_CURRENT,&DCMotor.Records.Min_Current_Reading,&DCMotor.Records.Max_Current_Reading); 
        }
    }
}
void checkMotor()
{
    //start the motor
    if(OperatingModeFlags.MotorTestEnabled  == MOTOR_TEST_ENABLED)
    {
        if(DCMotor.Settings.CurrentDutyCycle == 0)  //Motor hasn't started yet
        {
            startMotor();
            DCMotor.Settings.MaxDutycyle     = check_current_value_of_8bit_OD_entry(MOTOR_MAX_DUTY_CYCLE_CAN_INDEX,true);
            DCMotor.Settings.CurrentDutyCycle= motor_duty_cycle_delta;
        }
        else 
        {
            AccelerateMotor(&DCMotor.Settings.CurrentDutyCycle, DCMotor.Settings.MaxDutycyle);
        }
    }
    //stop the motor
    if(OperatingModeFlags.MotorTestEnabled  == MOTOR_TEST_HALTING)
    {
        if(AccelerateMotor(&DCMotor.Settings.CurrentDutyCycle, 0)   == 1)
        {
            turnoffMotorControl();
        }
    }
    
    if(DCMotor.Settings.CurrentDutyCycle==DCMotor.Settings.MaxDutycyle)
    {
        SendEncoderData();   //don't send data while accelerating
    }
}

void turnoffMotorControl(void)
{
    disableCM1();
    //disable driver
    TurnLatchOff(DCMotor.Connections.enablePinLatch, DCMotor.Connections.enablePinNumber);
    CCP2CONbits.CCP2M = 0;  //turn off pwm
    DCMotor.Settings.CurrentDutyCycle   = 0;
   
    ///Motor Control in COD   
    Edit_COD(MAXON_GRIPPER_TEST_CAN_INDEX, NULL, 0) ;
    
    //program flag
    OperatingModeFlags.MotorTestEnabled = MOTOR_TEST_DISABLED;
    
    //display min/max records
    if(DCMotor.Records.Max_Encoder_Reading!=0)
    {
        convertHexToDecAndTransmit('E',DCMotor.Records.Min_Encoder_Reading,true);    //E for encoder
        convertHexToDecAndTransmit('E',DCMotor.Records.Max_Encoder_Reading,true);    //E for encoder
    }
    if(DCMotor.Records.Max_Current_Reading!=0)
    {
        transmit_readable_sensor_data(DCMotor.Records.Min_Current_Reading&0xffff, 'A',  Current_Sensor_type);
        transmit_readable_sensor_data(DCMotor.Records.Max_Current_Reading&0xffff, 'A',  Current_Sensor_type);
    }RED_LED_OFF;
}
void setMotorRecordDefaults(void)
{
    TMR1=1;                     //set to 1; setting to 0 causes program to detect a collision
    EncoderOverflows=0;
    Timers.EncoderWriteFlag=0;
    DCMotor.Records.Min_Encoder_Reading = 16777215; //24bit max
    DCMotor.Records.Max_Encoder_Reading = 0;
    DCMotor.Records.Min_Current_Reading = 65535; //16bit max
    DCMotor.Records.Max_Current_Reading = 0;
    MotorAtMaxSpeed = false;
}

void startMotor(void)
{
    
    //read parameters from object dictionary
    DCMotor.Settings.Direction          = check_current_value_of_8bit_OD_entry  (MOTOR_DIRECTION_CAN_INDEX,false);
    DCMotor.Settings.StatusUpdateRate   = check_current_value_of_16bit_OD_entry (MOTOR_STATUS_CAN_INDEX,false);
    DCMotor.Settings.encoderConnected   = check_current_value_of_8bit_OD_entry  (MOTOR_ENCODER_ENABLED_CAN_INDEX,false);
    
    //TurnLatchOff(DCMotor.Connections.directionPinLatch, DCMotor.Connections.direction1PinNumber);
    //TurnLatchOff(DCMotor.Connections.directionPinLatch, DCMotor.Connections.direction2PinNumber);
    
    setMotorRecordDefaults();
    
    if(DCMotor.Settings.Direction == 0 )
    {
        TurnLatchOn(DCMotor.Connections.directionPinLatch, DCMotor.Connections.direction1PinNumber);
    }
    else
    {
        TurnLatchOff(DCMotor.Connections.directionPinLatch, DCMotor.Connections.direction1PinNumber);
       // TurnLatchOn(DCMotor.Connections.directionPinLatch, DCMotor.Connections.direction2PinNumber);
    }

    TurnLatchOn(DCMotor.Connections.enablePinLatch, DCMotor.Connections.enablePinNumber);
    
    //PWM output connected to the driver enable pin
    setupPWM(1,255);
    enableCM1();
}


uint8_t AccelerateMotor(uint8_t *motor_duty_cycle, uint8_t EndDutyCycle)
{
    if(Timers.MotorAccelerateflag>=Motor_Acc_TMR_Interrupts)  //every 10 ms
    {
        if(*motor_duty_cycle!=EndDutyCycle)
        {
            CCPR2L = *motor_duty_cycle;
            
            if(*motor_duty_cycle<EndDutyCycle)
            {
                *motor_duty_cycle+=motor_duty_cycle_delta;
            }
            else
            {
                *motor_duty_cycle-=motor_duty_cycle_delta;
            }

            Timers.MotorAccelerateflag=0;
            
            return 0;
        }
        else
        {
            return 1;
        }
    } 
    return 0;    
}
#endif	/* MOTORCONTROL_H */

