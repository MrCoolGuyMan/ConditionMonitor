/* 
 * File:   SPI_Interface.c
 * Author: Administrator
 *
 * Created on 15 August, 2022, 11:02 AM
 */

#include "SPI_Interface.h"
/*
 * 
 */
CS_PORT_t CS_PORTS[MAX_NUM_SPI_DEVICES];
static uint8_t currentDeviceID;
void SET_CS_LOW(uint8_t deviceID)
{
    *CS_PORTS[deviceID].LATCH_REG &=  (0xff^CS_PORTS[deviceID].LATCH_PIN_NUM);
}
void SET_CS_HIGH(uint8_t deviceID)
{
    *CS_PORTS[deviceID].LATCH_REG |=  (CS_PORTS[deviceID].LATCH_PIN_NUM);
}
// returns the current device ID, which must be retained by user
uint8_t Add_CS_LATCH(volatile uint8_t *LAT_REG, uint8_t PinNumber)
{
	CS_PORTS[currentDeviceID].LATCH_REG = LAT_REG;
    int16_t mem_offset = &TRISA - &LATA;
    CS_PORTS[currentDeviceID].TRIS_REG = LAT_REG + mem_offset;
	CS_PORTS[currentDeviceID].LATCH_PIN_NUM = 0xff&(1<<PinNumber);

    return currentDeviceID++;
}
static void SetCSPORTsDDR(void)
{
    // set low for output
    for(uint8_t index=0; index < currentDeviceID; ++index)
    {
        *CS_PORTS[index].TRIS_REG &=  (0xff^CS_PORTS[index].LATCH_PIN_NUM);
    }
}
static void SetCSHIGH(void)
{
    for(uint8_t index=0; index < currentDeviceID; ++index)
    {
        SET_CS_HIGH(index);
    }
}
void SPI_WRITE(unsigned char msg)
{
    unsigned char data_flush;
    PIR1bits.SSPIF=0;
    SSPBUF=msg;
    
    while(!PIR1bits.SSPIF)//wait until message complete
    {

    }
    
    data_flush=SSPBUF;
}
void SPI_MULTI_WRITE(uint8_t num_msgs, ...)
{
    ///FYI - "Warning: (1496)" is bugged
    va_list args;
    va_start(args,num_msgs);
    for (uint8_t counter=0; counter<num_msgs;counter++)
    {
        SPI_WRITE(va_arg(args,uint8_t));
    }
    va_end(args);
}
char READ_SPI(void)
{
    //read and write are functionally identical
    SPI_WRITE(0xff);
    
    return SSPBUF;
}
void spi_comms_setup(uint8_t CKE,uint8_t CKP,uint8_t SSPM)
{
    //SSPCON1bits.SSPEN=0;
    
    SetCSPORTsDDR();
    SetCSHIGH();
     /****************************
     setup SPI interface
     ****************************/
    
    SDI_TRIS        =   1;//input (SPI in) (SDI))
    SCK_TRIS        =   0;///SCK
    SDO_TRIS        =   0;///SPI OUT (SDO)

    /******
     * SSP1STATbits - page 251
     **********/
    SSPSTAT=0x00;
    SSPSTATbits.CKE = CKE;          //clock edge

     /******
     * SSP1CON1 - page 252
     **********/
    SSPCON1=0;//CKP | SSPM;
    SSPCON1bits.CKP     =   CKP;    //clock polarity
    SSPCON1bits.SSPM    =   SSPM;   //Clock speed
        
    SSPCON1bits.SSPEN=1;
    PIR1bits.SSPIF=0;
}