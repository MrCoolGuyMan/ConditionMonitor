/* 
 * File:   MotorControl.h
 * Author: Administrator
 *
 * Created on 22 November, 2021, 12:01 PM
 */

#ifndef MOTORCONTROL_H
#define	MOTORCONTROL_H
#include "CustomDataTypes.h"
#include <stdint.h>
#include <stdbool.h>
#include <xc.h>
#include "PIC18F Peripheral Drivers/IO_Ports.h"
#include "ObjectDictionary/CAN Object Dictionary.h"
#define MOTOR_TEST_ENABLED 1
#define MOTOR_TEST_HALTING 0
#define MOTOR_TEST_DISABLED 2
#define MOTOR_TEST_EMERGENCY_HALT 3
#define MOTOR_DUTY_CYCLE_DELTA  1
#define MOTOR_MAX_DUTY_CYCLE    127
    //L6202 chip requires 2 direction pins and a PWM pin
        typedef struct 
        {
            //both direction pins must be connected to the same port
            volatile uint8_t *directionPinLatch;    //Points to a PORT's Latch register
            uint8_t direction1PinNumber;            //these represent the pin numbers on the above latch

            volatile uint8_t *enablePinLatch;       //Points to a PORT's Latch register
            uint8_t enablePinNumber;

        }MotorConnections;

        typedef struct 
        {
            uint8_t MaxDutycyle;                //0-255
            volatile uint8_t *CurrentDutyCycle;           
            uint8_t Direction;                  //0-1 clockwise or counter clockwise
            uint16_t StatusUpdateRate;          //how often sensor data is transmitted during DC motor tests
            bool encoderConnected;              //is encoder attached?
        }Motor_Parameters;
      
        typedef struct
        {
            Motor_Parameters Settings;
            MotorConnections Connections;
        }MotorClass;
        

void checkMotor(void);
void turnoffMotorControl(void);
void setMotorRecordDefaults(void);
void startMotor(void);
void disableCM1(void);
void enableCM1(void);
uint8_t AccelerateMotor(volatile uint8_t *motor_duty_cycle, uint8_t EndDutyCycle);
void setupPWM(uint8_t startDutyCycle, uint8_t PR_Counter);

#endif	/* MOTORCONTROL_H */

