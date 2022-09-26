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

#include <xc.h>
#include <stdint.h>
#include <stdarg.h>
/**
 * 
 * User inputs go here
 * 
 */
#ifndef MAX_NUM_SPI_DEVICES
#define MAX_NUM_SPI_DEVICES 10
#endif
typedef struct
{
    volatile uint8_t *LATCH_REG;
    volatile uint8_t *TRIS_REG;
    uint8_t LATCH_PIN_NUM;
}CS_PORT_t;

#define ChipSelect_HUMID            LATAbits.LA2
#define CS_TRIS_HUMID               TRISAbits.TRISA2
/*          
 * end of user input
 */
#define SDI_TRIS                    TRISBbits.TRISB0
#define SCK_TRIS                    TRISBbits.TRISB1
#define SDO_TRIS                    TRISBbits.TRISB3
#define MCU_SS_TRIS                 TRISAbits.TRISA5

#define NOT_CS_LOW  0
#define NOT_CS_HIGH 1


#define CKP_HIGH_IDLE       (1<<4)
#define CKP_LOW__IDLE       (0<<4)
#define CKE_ACTIVE_TO_IDLE  (1<<6)
#define CKE_IDLE_TO_ACTIVE  (0<<6)
#define SYS_CLOCK_OVER_4    (0b000)
#define SYS_CLOCK_OVER_16   (0b001)
#define SYS_CLOCK_OVER_64   (0b010)
#if USE_PLL == 1 //48Mhz clock
    #define _4MHz_CLK   SYS_CLOCK_OVER_16    
    #define _12MHz_CLK  SYS_CLOCK_OVER_4
    #define EEPROM_SPI_SETTINGS 1,0,_12MHz_CLK
#else
    #define _4MHz_CLK   SYS_CLOCK_OVER_4
    #define EEPROM_SPI_SETTINGS 1,0,_4MHz_CLK
#endif

#define DEFAULT_SPI_SETTINGS    1,0,_4MHz_CLK
#define HUMID_SPI_SETTINGS      1,0,_4MHz_CLK

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
/*
 \                                                        asm("CLRF POSTINC0,0              ; clear olD value");\
                                                    else\
                                                        */

#define FAST_SPI_WRITE_HEADER(YourArrayOrPointer)   \
                                                    uint16_t FSR_Save = FSR0;\
                                                    FSR0 = (volatile uint16_t)YourArrayOrPointer;
#define FAST_SPI_WRITE\
                                                    asm("BCF PIR1, 3, 0               ; clear PIR1bits.SSPIF");\
                                                    asm("MOVFF INDF0, SSP1BUF         ; copy value in FSR to SSP1BUF, increment address of FSR afterwards" );\
                                                    if(++index == 0){\
                                                        break;}\
                                                    asm("CLRF POSTINC0,0              ; clear old value");\
                                                    SPI_WAIT_MACRO;
#define FAST_SPI_WRITE2 \
                                                    asm("BCF PIR1, 3, 0                 ; clear PIR1bits.SSPIF");\
                                                    asm("MOVFF POSTINC0, SSP1BUF        ; copy value in FSR to SSP1BUF, increment address of FSR afterwards");\
                                                    index++;\
                                                    SPI_WAIT_MACRO;

#define FAST_SPI_WRITE_FOOTER                       FSR0 = FSR_Save;

#endif

// cause SPI peripheral to begin reading
#define SPI_READ_MACRO  \
                                                    asm("BCF PIR1, 3, 0                 ; clear PIR1bits.SSPIF");\
                                                    asm("SETF SSP1BUF, 0                ; set SSPBUF to arbitrary value");\
                                                    asm("BTFSS PIR1,3,0                 ; while(!PIR1bits.SSPIF) ; ");\
                                                    asm("BRA $-2                        ; branch back two bytes (one instruction)");

// read an 8 bit SPI value quickly (without function call overhead); no limits on usage
#define READ_8_BIT_SPI(output)\
                                                    SPI_READ_MACRO;\
                                                    output = SSPBUF;

// read a 16 bit SPI value quickly (without function call overhead); no limits on usage
#define READ_16_BIT_SPI(output)\
                                                    SPI_READ_MACRO;\
                                                    output = SSPBUF;\
                                                    SPI_READ_MACRO;\
                                                    output |= (uint16_t)(SSPBUF<<8);

void SPI_WRITE(unsigned char msg);
void SPI_MULTI_WRITE(uint8_t num_msgs, ...);
char READ_SPI(void);
void spi_comms_setup(uint8_t CKE,uint8_t CKP,uint8_t SSPM);

uint8_t Add_CS_LATCH(volatile uint8_t *LAT_REG, uint8_t PinNumber);
void SET_CS_LOW(uint8_t deviceID);
void SET_CS_HIGH(uint8_t deviceID);
#endif	/* NEWFILE_H */

