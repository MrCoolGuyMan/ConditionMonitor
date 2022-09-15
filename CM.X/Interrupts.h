/* 
 * File:   Interrupts.h
 * Author: Administrator
 *
 * Created on 2 August, 2022, 3:38 PM
 */

#ifndef INTERRUPTS_H
#define	INTERRUPTS_H

#ifdef	__cplusplus
extern "C" {
#endif


void __interrupt(high_priority) tInt(void)
{
    //Timer 0 interrupts every x us
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
            static uint8_t makeSureThisIsReal = 0;
            
            if(makeSureThisIsReal++ > 5){
                OperatingModeFlags.MotorTestEnabled = MOTOR_TEST_EMERGENCY_HALT;
                RED_LED_ON;
                makeSureThisIsReal=0;
            }
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


#ifdef	__cplusplus
}
#endif

#endif	/* INTERRUPTS_H */

