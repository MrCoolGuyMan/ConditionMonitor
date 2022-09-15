#include "DCMotorControl.h"

static bool MotorAtMaxSpeed = false;
static MotorClass DCMotor =
{
    .Settings =         {   
                            .CurrentDutyCycle       =   &CCPR2L,
                            .MaxDutycyle            =   255,
                            .Direction              =   0,
                        },
    .Connections =      {  
                            .directionPinLatch      =   &LATB,
                            .direction1PinNumber    =   IO_PORT_PIN_7,
                            .enablePinLatch         =   &LATC,
                            .enablePinNumber        =   IO_PORT_PIN_1
                        }
};
extern TimingFlags Timers ;
extern ProgramFlags OperatingModeFlags ;

void checkMotor()
{
    //start the motor
    if(OperatingModeFlags.MotorTestEnabled  == MOTOR_TEST_ENABLED)
    {
        if(*DCMotor.Settings.CurrentDutyCycle == 0)  //Motor hasn't started yet
        {
            startMotor();
            DCMotor.Settings.MaxDutycyle     = check_current_value_of_8bit_OD_entry(MOTOR_MAX_DUTY_CYCLE_CAN_INDEX,true);
        }
        else 
        {
            AccelerateMotor(DCMotor.Settings.CurrentDutyCycle, DCMotor.Settings.MaxDutycyle);
        }
    }
    //stop the motor
    if(OperatingModeFlags.MotorTestEnabled  == MOTOR_TEST_HALTING)
    {
        if(AccelerateMotor(DCMotor.Settings.CurrentDutyCycle, 0)   == 1)
        {
            turnoffMotorControl();
        }
    }
}

void turnoffMotorControl(void)
{
    disableCM1();
    //disable driver
    TurnLatchOff(DCMotor.Connections.enablePinLatch, DCMotor.Connections.enablePinNumber);
    CCP2CONbits.CCP2M = 0;  //turn off pwm
   
    //program flag
    OperatingModeFlags.MotorTestEnabled = MOTOR_TEST_DISABLED;
    
}

void startMotor(void)
{
    //read parameters from object dictionary
    DCMotor.Settings.Direction          = check_current_value_of_8bit_OD_entry  (MOTOR_DIRECTION_CAN_INDEX,false);

    if(DCMotor.Settings.Direction == 0 )
    {
        TurnLatchOn(DCMotor.Connections.directionPinLatch, DCMotor.Connections.direction1PinNumber);
    }
    else
    {
        TurnLatchOff(DCMotor.Connections.directionPinLatch, DCMotor.Connections.direction1PinNumber);
    }

    TurnLatchOn(DCMotor.Connections.enablePinLatch, DCMotor.Connections.enablePinNumber);
    
    //PWM output connected to the driver enable pin
    setupPWM(1,127);
    enableCM1();
}

//disable comparator 1
void disableCM1(void)
{
    C1IE = 0;               //interrupts off
    C1IF = 0;               //interrupts off
    CM1CON0bits.C1ON = 0;   //turn off comparator
}
//enable comparator 1
void enableCM1(void)
{
    //Page 307
    CM1CON0 = 0x00;
    CM1CON0bits.C1SP = 1;   //high speed mode
    CM1CON0bits.C1R = 1;    //used internal voltage reference for Vin+
    CM2CON1bits.C1RSEL = 1; //FVR BUF1 routed to C1 Vref
    CM1CON0bits.C1CH = 0b01;//Connects to C12IN- (RA1)
    C1IE = 1;               //enable interrupt
    C1IF = 0;               //clear interrupt flag
    CM1CON0bits.C1ON = 1;   //turn on comparator
}

//turn on PWM using Timer 2
void setupPWM(uint8_t startDutyCycle, uint8_t PR_Counter)
{
    //setup PWM
    CCP2CONbits.CCP2M=0b1100; //PWM mode
    
    //set duty cycle
    CCP2CONbits.DC2B = 0b00;
    PR2 = PR_Counter;
    CCPR2L = startDutyCycle;
    T2CON = 0;
    TMR2 =0;
    TMR2ON = 1;
}


uint8_t AccelerateMotor(volatile uint8_t *motor_duty_cycle, uint8_t EndDutyCycle)
{
    if(Timers.MotorAccelerateflag>=Motor_Acc_TMR_Interrupts)  //every 10 ms
    {
        if(*motor_duty_cycle!=EndDutyCycle)
        {           
            if(*motor_duty_cycle<EndDutyCycle)
            {
                *motor_duty_cycle+=MOTOR_DUTY_CYCLE_DELTA;
            }
            else
            {
                *motor_duty_cycle-=MOTOR_DUTY_CYCLE_DELTA;
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
