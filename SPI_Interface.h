/* 
 * File:   newfile.h
 * Author: Administrator
 *
 * Created on 18 January, 2021, 11:05 AM
 * 
 * implements SPI interface as per datasheet
 * 
 * to add new components to SPI, do the following
 * 
 * define which port the chip/slave select line is connected to
 * 
 * we need both a TRIS register and a LATCH register
 * 
 * the TRIS selects the input/output state of the pin 
 * 
 * the LATCH register sets the high/low status
 * 
 */

#ifndef SPI_INTERFACE_H
#define	SPI_INTERFACE_H

/**
 * 
 * User inputs go here
 * 
 */
    /*
     * set LATCH register
     */
    #define	EXTERNAL_EEPROM_CHIP_SELECT LATAbits.LA4    //was b5 
    #define ChipSelect_CAN_CONTROLLER   LATBbits.LB4  
    #define ChipSelect_ACCL             LATAbits.LA6

    /*
     * set TRIS register
     */
    #define CS_TRIS                     TRISAbits.TRISA4
    #define CS_TRIS_EEPROM              TRISBbits.TRISB5
    #define CS_TRIS_ACCL                TRISAbits.TRISA6

    /*
     * add TRIS register to this list so that it is set as an output
     */
    #define MACRO_SET_CS_TRIS_STATUS    CS_TRIS_EEPROM = 0;\
                                        CS_TRIS = 0;\
                                        CS_TRIS_ACCL = 0;
    /*
     * add latch register to this list so that it is set as high (off) during initialization
     */
    #define MACRO_SET_CS_LATCH_STATUS   EXTERNAL_EEPROM_CHIP_SELECT = 1;\
                                        ChipSelect_CAN_CONTROLLER = 1;\
                                        ChipSelect_ACCL = 1;
/*          
 * end of user input
 */
#define SDI_TRIS                    TRISBbits.TRISB0
#define SCK_TRIS                    TRISBbits.TRISB1
#define SDO_TRIS                    TRISBbits.TRISB3
#define MCU_SS_TRIS                 TRISAbits.TRISA5

#define NOT_CS_LOW  0
#define NOT_CS_HIGH 1
#if USE_PLL == 1 //48Mhz clock
    #define _4MHz_CLK 0b0001    
    #define _12MHz_CLK 0b0000
    #define EEPROM_SPI_SETTINGS 1,0,_12MHz_CLK
#else
    #define _4MHz_CLK 0b0000
    #define EEPROM_SPI_SETTINGS 1,0,_4MHz_CLK
#endif
#define ACC_SPI_SETTINGS 0,1,_4MHz_CLK
#define DEFAULT_SPI_SETTINGS 1,0,_4MHz_CLK



//xc8 does not support in-line functions in free compiler version//
#ifdef _12MHz_CLK

// wait for SSP interrupt flag to set (i.e. for SSP Buffer to Fill up)
#define SPI_WAIT_MACRO \
                        asm("BTFSS PIR1,3,0                 ; while(!PIR1bits.SSPIF) ; ");\
                        asm("BRA $-2                        ; branch back two bytes (one instruction)");

//  uses "File select register" (FSR) 
//  you need to save and restore FSR2 context before/after using this
//
//  FAST_SPI_WRITE_HEADER(YourArrayOrPointer)
//
//      FAST_SPI_WRITE
//      FAST_SPI_WRITE
//      .
//      .
//      .
//      FAST_SPI_WRITE                  // however many times you need
//
//  FAST_SPI_WRITE_FOOTER           // restore context


#define FAST_SPI_WRITE_HEADER(YourArrayOrPointer)   \
                                                    uint16_t FSR_Save = FSR2;\
                                                    FSR2 = (volatile uint16_t)YourArrayOrPointer;
#define FAST_SPI_WRITE \
                                                    asm("BCF PIR1, 3, 0                 ; clear PIR1bits.SSPIF");\
                                                    asm("MOVFF POSTINC2, SSP1BUF        ; copy value in FSR2 to SSP1BUF, increment address of FSR2 afterwards");\
                                                    index++;\
                                                    SPI_WAIT_MACRO;

#define FAST_SPI_WRITE_FOOTER                       FSR2 = FSR_Save;

#endif

// cause SPI peripheral to begin reading
#define SPI_READ_MACRO  \
                                                    asm("BCF PIR1, 3, 0                 ; clear PIR1bits.SSPIF");\
                                                    asm("SETF SSP1BUF, 0                ; set SSPBUF to arbitrary value");\
                                                    SPI_WAIT_MACRO;

// read an 8 bit SPI value quickly (without function call overhead); no limits on usage
#define READ_8_BIT_SPI(output)\
                                                    SPI_READ_MACRO;\
                                                    output = SSPBUF;

// read a 16 bit SPI value quickly (without function call overhead); no limits on usage
#define READ_16_BIT_SPI(output)\
                                                    READ_8_BIT_SPI(output);\
                                                    SPI_READ_MACRO;\
                                                    output |= (uint16_t)(SSPBUF<<8);

void SPI_WRITE(unsigned char msg)
{
    unsigned char data_flush;
    SSPBUF=msg;

    while(!PIR1bits.SSPIF)//wait until message complete
    {

    }
    PIR1bits.SSPIF=0;
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
    SSPCON1bits.SSPEN=0;

     /****************************
     setup SPI interface
     ****************************/
    
    SDI_TRIS        =1;//input (SPI in) (SDI))
    SCK_TRIS        =0;///SCK
    SDO_TRIS        =0;///SPI OUT (SDO)
    MCU_SS_TRIS     =0;//SS' in disabled
    
    MACRO_SET_CS_TRIS_STATUS;   //set pins as outputs
    
    /******
     * SSP1STATbits - page 251
     **********/
    SSPSTAT=0x00;
    SSPSTATbits.SMP = 0;
    SSPSTATbits.CKE = CKE;
    SSPSTATbits.BF=0;
     /******
     * SSP1CON1 - page 252
     **********/
    SSPCON1bits.WCOL =0;
    SSPCON1bits.SSPOV =0;

    SSPCON1bits.CKP =CKP;
    SSPCON1bits.SSPM =SSPM;//0b0000;   //Fosc/4 = 4Mhz Clock
    
    SSPCON2 =0x00; ///only used in I2C

    MACRO_SET_CS_LATCH_STATUS;
    
    SSPCON1bits.SSPEN=1;
    PIR1bits.SSPIF=0;
}
#endif	/* NEWFILE_H */

