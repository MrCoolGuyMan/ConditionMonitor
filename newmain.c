/* 
 * File:   newmain.c
 * Author: Track
 *
 * Created on 23 November 2020, 2:00 PM
 */
#define RED_LED         LATCbits.LATC2
#define RED_LED_ON      RED_LED=1;
#define RED_LED_OFF     RED_LED=0;
#include "Config.h"
#include <stdint.h>                 //uint8_t etc.
#include "CustomDataTypes.h"
#include <xc.h>
#include "Global Variables.h"
#include "Prototypes.h"
#include <stdarg.h>                 ///variable argument functions - > see can controller Fill_TX_REG_of_CAN_CONTROLLER
#include "SPI_Interface.h"
#include "CAN Object Dictionary.h"
#include "CAN Controller.h"
#include "SDO Protocol.h"
#include "External EEPROM.h"
#include "MotorControl.h"
#include "Sensors.h"

#define GREEN_LED LATCbits.LATC6


#define TOGGLE_GREEN    asm("BTG LATC, 6");

int main()
{
     
    /*****
     OSCCON -page 33
	 set internal clock rate then wait for stabilization 
     ***/

        OSCCONbits.IRCF=0b111;//16MHZ
        #if USE_PLL == 1
            OSCTUNE = 0b10000000;
        #endif  
        while(OSCCONbits.HFIOFS==0)
        {

        }
        
	/****
	basic pin configuration
	*****/
	   setup_IO_Ports();
	   INTCON2bits.RBPU=1;
	   
	   setup_ADC();
	   spi_comms_setup(DEFAULT_SPI_SETTINGS);
	      
	///check if the internal EEPROM is storing a node id already
		Assigned_Node_ID=check_current_value_of_8bit_OD_entry(NODE_ID_CAN_INDEX,false);
        
        //this will happen after a firmware update
        if(Assigned_Node_ID==0x00)
        {
            factory_reset();
            RESET();
        }
    
	/****
	CAN Controller configuration
	*****/
		
		Reset_CAN_CONTROLLER();   
    
        setup_heart_beat_command();
    
    //setup timer0,1 and INT2
        Interrupt_setup();
    
        initSensorList();
        
    /****
	 *if the logger was reset during monitoring mode, this will still be active
     *turn off monitoring and sensor transfer
     *************/
        Turn_off_Sensor_Monitoring();
        
        ZeroStorageBuffer();

    //infinite loop

        CheckFlags();
        
    return (EXIT_SUCCESS);
}
void CheckFlags(void)
{
    while(1)
    {
        //check if any messages are received
        while(CAN_INTERRUPT_PIN==0)///keep checking until all messages have been read
        {
            check_RX(); 
        }

        if(Timers.SensorFlag>=Sensor_Checking_Interval)  //user definable period; 1ms by default
        {
            Timers.SensorFlag=0;
            if(OperatingModeFlags.MonitoringModeActive==true)
            {
                Begin_sensor_monitoring();
            }
        }

        if(OperatingModeFlags.MonitoringModeActive==false)
        {
            ///send heartbeat every 500 ms 
            if(Timers.HeartBeat_FLAG>=Heart_Beat_TMR_Interrupts)
            {
                Timers.HeartBeat_FLAG=0;
                send_heartbeat_message();
                TOGGLE_GREEN;
            }

            if(OperatingModeFlags.MotorTestEnabled  != MOTOR_TEST_DISABLED)
            {
               checkMotor();
            }
        }
        if(OperatingModeFlags.MotorTestEnabled == MOTOR_TEST_EMERGENCY_HALT)
        {
            Turn_off_Sensor_Monitoring();
        }
    }
}
uint8_t Read_From_INTERNAL_EEPROM(uint8_t Address)
{
    EEADR = Address;
    EECON1bits.WREN=0;
    EECON1bits.EEPGD=0;
    EECON1bits.RD=1;
    
    return EEDATA;
}

///returns the data being written if successful, otherwise 0
void Write_Data_to_INTERNAL_EEPROM(uint8_t Address, uint8_t Data[8], uint8_t Total_Bytes)
{
    /***
     * EECON1 - page 105
     * bit 7 EEPGD - controls whether the program uses flash (1) or eeprom (0)
     * bit 6 CFGS - flash program/data or configuration select bit (1)
     * bit 5 N/A
     * bit 4 FREE - flash row erase enable bit
     * bit 3 WRERR - error flag
     * bit 2 WREN - enable flag
     * bit 1 WR - write control bit (automatically cleared)
     * bit 0 RD - Read control
     * 
     * 
     * EECON2 - no page ; referenced at 106
     * write 0x55 and then
     * write 0xAA 
     * it's recommended to disable interrupts
     ***/
    INTCONbits.GIE=0; //disable interrupts
    uint8_t prevent_infinite_loop = 0;
    for (uint8_t counter=0; counter< Total_Bytes;counter++)
    {
        EEADR = Address+counter;
        EEDATA = Data[counter];

        EECON1bits.EEPGD=0;
        EECON1bits.CFGS=0;
        EECON1bits.WREN=1;

        //the gods demand it
        EECON2=0x55;
        EECON2=0xAA;

        //start write
        EECON1bits.WR=1;
        
        //wait for write to complete
        while(PIR2bits.EEIF==0)
        {

        }
        PIR2bits.EEIF=0;

        ///confirm successful write
        uint8_t Check_Data=Read_From_INTERNAL_EEPROM(Address+counter);

        //if the data we read doesn't match what we wrote
        if(Data[counter] != Check_Data)
        {
            counter--;  //decrement and try again
        }
        prevent_infinite_loop++;
        if (STATUSbits.C == 1)  //overflow will cause loop to exit
        {
            break;
        }
    }
    INTCONbits.GIE=1; //enable interrupts
    return ;
}
//in case the controller is stuck in some continuous monitoring mode; we will turn off every flag during power-on
void Turn_off_Sensor_Monitoring(void)
{
    turnoffMotorControl();
    ///monitoring flag
    Edit_COD(ENTER_SENSOR_MONITORING_MODE, NULL, 0) ;

    ///eeprom transfer flag 
    Edit_COD(TRANSFER_SENSOR_DATA_FROM_EEP, NULL, 0) ;
        
    ///record can messages
    Edit_COD(ENTER_CAN_BUS_MONITORING_MODE, NULL, 0) ;
}

/****************************
record the value from each type of sensor in turn
****************************/
void Begin_sensor_monitoring(void)
{
    for(uint8_t index =0; index <NumberOfSensorsInSensorList ; index++)
    {
        SensorHeader(index,MonitoringModeOutput);
    }
}
void __interrupt(high_priority) tInt(void)
{
    //Timer 0 interrupts every 1 ms
    if(INTCONbits.TMR0IF==1)
    {
        TMR0 = TIMER0_START_COUNT;
        INTCONbits.TMR0IF=0;
        
        Timers.OperatingTime_ms++;

        Timers.HeartBeat_FLAG++;
        Timers.SensorFlag++;               //controls how often sensors are checked
        
        //if(OperatingModeFlags.MotorTestEnabled != MOTOR_TEST_DISABLED)
        {
            Timers.MotorAccelerateflag++;  
            Timers.EncoderWriteFlag++;
        }
    }

    //timer 1 will overflow when we hit 65536 encoder pulses
    if(TMR1IF==1)   
    {
        EncoderOverflows++;
        TMR1IF=0;
    }    
    if(TMR3IF==1)   
    {
        EncoderOverflows2++;
        TMR3IF=0;
    }
    
    //analog - overcurrent protection for motor controller
    if(C1IF==1)
    {
        C1IF=0;
        if(C1OUT == 0)  //Vin- > Vin+
        {
            OperatingModeFlags.MotorTestEnabled = MOTOR_TEST_EMERGENCY_HALT;
            RED_LED_ON;
            
        }
            
    }
  /*  if(ADIF==1)
    {
        ADCON0 = 0x00; //turn of ADC
        ADIF=0;
    }*/
}
void Interrupt_setup(void)
{
    INTCONbits.GIE    = 1;///enable global interrupts
    
        INTCON3bits.INT2IE =0;///DISABLED
        /*INTCON3bits.INT2IP =1;///High priority
        INTCON3bits.INT2IF =0;///clear flag
        INTCON2bits.INTEDG2=0;///falling Edge*/

        TRISBbits.TRISB2=1;//set as input for interrupt
   
    INTCON3bits.INT1IE=0;//disabled; port used for SPI Clock
    
    ///Timer0; used for heartbeat timing 
        INTCONbits.TMR0IE =1;
        INTCONbits.TMR0IF =0;
        INTCON2bits.TMR0IP=1;//High priority
        
        T0CONbits.T08BIT=0;//16 bit
        T0CONbits.T0CS  =0;//internal timer
        T0CONbits.T0SE  =0;//unused
        T0CONbits.PSA   =1;//bypass prescaler
        T0CONbits.T0PS  =0x000;//1:2  
        TMR0 = TIMER0_START_COUNT;
        T0CONbits.TMR0ON=1;
    
    ///Timer1 is used to measure encoder pulses
        
        T1CONbits.RD16=1;           //16 bit Read/Write
        T1CONbits.TMR1CS  =0b10;    //external timer on pin C0
        T1CONbits.SOSCEN = 0;       //dedicated secondary oscillator disabled
       
        TMR1IF = 0;
        TMR1IE = 1;
        T1CONbits.TMR1ON=1;
     ///Timer3 is used to measure encoder pulses               
        T3CONbits.RD16=1;           //16 bit Read/Write
        T3CONbits.TMR3CS  =0b10;    //external timer on pin C0
        T3CONbits.SOSCEN = 0;       //dedicated secondary oscillator disabled
       
        TMR3IF = 0;
        TMR3IE = 1;
        T3CONbits.TMR3ON=1;
    //ADC interrupts
      /*  ADIE = 1;
        ADIF = 0;*/
}
void setup_IO_Ports(void)
{
    SLRCONbits.SLRB = 0;
    LATA=0x80;
    LATB=0x00;
    LATE=0x00;
    /****************************
     * setup PORTA  as analog input
     * Temperature  Sensors are connected here
     * RA0 = AN0
     * RA1 = AN1
     * RA2 = AN2
     * RA3 = AN3
     * RA5 = AN4
     * 
     * RA4 = Motor input 1
     * RA6 = Motor input 2
     * RA7 = chip selected for ACC
    *****************************/    
    TRISA=0x0f;
    PORTA=0x80; 
    ANSELA=0x7f;
    /****************************
     * setup PORTB as output
     * RB0 = SDI/SDA
     * RB1 = SCK/SCL
     * RB3 = SDO
    *****************************/

    
    TRISB=0x00;
    PORTB=0x00;
    ANSELB=0x00;    
   
    TRISBbits.RB5 = 1;
    /****************************
     * setup PORTC as analog output
     * to control status LEDs
     * only RC0 - RC2 exist
     * RCO is used as ENCODER FEEDBACK
     * RC1 is enable pin for motor controller
    *****************************/
    TRISC=0x01;
    PORTC=0x00;
    ANSELC=0x00;     
      
    /****************************
     * setup PORTD  as analog input
     * Temperature  Sensors are connected here
     * RD0 = AN20
     * RD7 = AN27
    *****************************/
    TRISD=0xff;
    ANSELD=0xff;///port a is an analog input
       
    /****************************
     * setup PORTE  as output
     * only RE0 - RE 2 are usable
     * RE 3 is digital Input only
    *****************************/
    TRISE=0x00;
    PORTE=0x00;
    ANSELE=0x00;
}

void factory_reset(void)
{
    INTCONbits.GIE=0;///disable global interrupts
    FIND_CAN_OBJECT(0,1);
    
    Reset_EEPROM();
    INTCONbits.GIE=1;///enable global interrupts
}

//takes a value like 0x0FC3 (0d4035))
//and converts it to hex-coded decimal
//  0x00 0x00 0x40 0x35
void convertHexToDecAndTransmit(uint8_t identifier,uint32_t InputValue, bool TransmitOnCompletion)
{
    spi_comms_setup(DEFAULT_SPI_SETTINGS);

    CopyNodeIDToCANController(0x00,0x00, CAN_TXB1CTRL_REG,true);
    
    uint32_t hex = 0;
    uint32_t i = 1;

    while(InputValue!=0)
    { 
        uint32_t digit = InputValue%10;
        InputValue=InputValue/10;
        hex+=digit*i;
        i*=16;
    }
    uint8_t MSG_DLC = 5;
    uint8_t first  = ((hex &0xff000000 )>>24);
    uint8_t second = ((hex &0xff0000   )>>16);
    uint8_t third  = ((hex &0xff00     )>>8);
    uint8_t fourth =  (hex &0xff       );

    Fill_TX_REG_of_CAN_CONTROLLER(CAN_TXB1CTRL_REG+5,MSG_DLC+1,MSG_DLC,identifier,first,second,third,fourth);

    if(TransmitOnCompletion == True)
    {
        transmit_CAN_Message(CAN_RTS_TXB1);
    }
    SSPEN = 0;
}
void compare_minmax(uint24_t value, uint24_t *Min, uint24_t *Max)
{
    if(*Min > value)
    {
        *Min = value;
    }
    if(*Max < value)
    {
        *Max = value;
    }
}
// LATx<PinNum> = 0;
void TurnLatchOff(volatile uint8_t *LatchReg, uint8_t PinNum)
{
    *LatchReg &=(0xff ^ (1<<PinNum));
}
// LATx<PinNum> = 1;
void TurnLatchOn(volatile uint8_t *LatchReg, uint8_t PinNum)
{
    *LatchReg |=(1<<PinNum);
}
// LATx<PinNum> = ~LATx<PinNum> ;
void ToggleLatch(volatile uint8_t *LatchReg, uint8_t PinNum)
{
    *LatchReg ^=~(1<<PinNum);
}
//disable comparator 1
void disableCM1(void)
{
    RED_LED_OFF;
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

void setupCTMU(void)
{
    //configure ports for analog
    
    //Select the current source 
    CTMUICONbits.IRNG = 0;
    //adjust the current source trim
    CTMUICONbits.ITRIM = 0;
    //configure the edge input sources for Edge1 and Edge2
    CTMUCONLbits.EDG1SEL = 0;
    CTMUCONLbits.EDG2SEL = 0;
    //configure the polarities for the edge inputs
    CTMUCONLbits.EDG1POL = 0;
    CTMUCONLbits.EDG2POL = 0;
    //enable edge sequencing 
    CTMUCONHbits.EDGSEQEN = 0;
    //select operating mode (Measurement or time delay)
    TGEN = 0;
    //discharge the connected circuit
    CTMUCONHbits.IDISSEN = 1;
    //wait for 1 msecond for circuit to discarge
    __delay_ms(1);
    CTMUCONHbits.IDISSEN = 0;
    
    //disable the module
    CTMUCONHbits.CTMUEN = 0;
    //enable the module
    CTMUCONHbits.CTMUEN = 1;
    //clear the edge status bits
    CTMUCONLbits.EDG1STAT = 0;
    CTMUCONLbits.EDG2STAT = 0;
    //enable both edge inputs
    CTMUCONHbits.EDGEN = 1;
}