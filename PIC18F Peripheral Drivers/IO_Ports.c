/* 
 * File:   SPI_Interface.c
 * Author: Administrator
 *
 * Created on 15 August, 2022, 11:02 AM
 */

#include "IO_Ports.h"
/*
 * 
 */

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
     * RB2 = MCP2515 interrupt
     * RB3 = SDO
     * RB5 = Timer 3 (input)
    *****************************/

    
    TRISB=0b00100100;
    PORTB=0x00;
    ANSELB=0x00;    

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
// LATx<PinNum> = 0;
void TurnLatchOff(volatile uint8_t *LatchReg, uint8_t PinNum)
{
    *LatchReg &=(0xff ^ (PinNum));
}
// LATx<PinNum> = 1;
void TurnLatchOn(volatile uint8_t *LatchReg, uint8_t PinNum)
{
    *LatchReg |=(PinNum);
}
// LATx<PinNum> = ~LATx<PinNum> ;
void ToggleLatch(volatile uint8_t *LatchReg, uint8_t PinNum)
{
    *LatchReg ^=~(PinNum);
}

