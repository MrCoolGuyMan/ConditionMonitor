/* 
 * File:   External EEPROM.h
 * Author: Administrator
 *
 * Created on 12 February, 2021, 1:05 PM
 */

#ifndef EXTERNAL_EEPROM_H
#define	EXTERNAL_EEPROM_H

#include "CustomDataTypes.h"
#define BUFFER_TRANSFER_SIZE 256
typedef union
{
    uint24_t _24_bit_Address;

    struct {
        uint8_t Addr_3;  
        uint8_t Addr_2;    
        uint8_t Addr_1; 
    }bytes;

}MemoryAddress;
 typedef struct 
 {
	 uint8_t StorageArray[BUFFER_TRANSFER_SIZE];
	 uint8_t currentSize;
	 MemoryAddress CurrentAddress;
 }LOG_DATA_BUFFER;

#define EXT_EEPROM_READ_COMMAND    0b00000011
#define EXT_EEPROM_WRITE_COMMAND   0b00000010
#define EXT_EEPROM_WREN_COMMAND    0b00000110///write enable
#define EXT_EEPROM_WRDI_COMMAND    0b00000100///write disable
#define EXT_EEPROM_RDSR_COMMAND    0b00000101///read status
#define EXT_EEPROM_WRSR_COMMAND    0b00000001///write status
#define EXT_EEPROM_PE_COMMAND      0b00000010//page erase
#define EXT_EEPROM_SE_COMMAND      0b11011000//sector erase
#define EXT_EEPROM_CE_COMMAND      0b11000111//chip erase
#define EXT_EEPROM_RDID_COMMAND    0b10101011//release from deep power down
#define EXT_EEPROM_DPD_COMMAND     0b10111001//deep power down
static LOG_DATA_BUFFER  SENSOR_DATA_BUFFER ={.currentSize=0, .CurrentAddress = {} };
//read data from EEPROM_ADDRESS
void handle_Ext_EEPROM_READ_REQUEST(MemoryAddress Address)
{
    SSPEN=0;

    uint16_t READ_Data = Read_From_Ext_EEPROM(Address); 

    Transmit_Sensor_Data(   READ_Data, 
                            Address.bytes.Addr_3,
                            Address.bytes.Addr_2,
                            Address.bytes.Addr_1,
                            0x00);
    
    SSPEN=0;
    
}
/*****************************************************
    increase EEPROM address pointer by 256 bytes
*****************************************************/
void Increment_Ext_EEPROM_PAGE(void)
{
#if BUFFER_TRANSFER_SIZE == 256
    SENSOR_DATA_BUFFER.CurrentAddress.bytes.Addr_2++;
#else
    SENSOR_DATA_BUFFER.CurrentAddress.bytes.Addr_3+=BUFFER_TRANSFER_SIZE;
    SENSOR_DATA_BUFFER.CurrentAddress.bytes.Addr_2+=STATUSbits.C;
#endif
    SENSOR_DATA_BUFFER.CurrentAddress.bytes.Addr_1+=STATUSbits.C;
    if(SENSOR_DATA_BUFFER.CurrentAddress.bytes.Addr_1>1)
    {
        transmit_monitoring_summary();
        Turn_off_Sensor_Monitoring();///avoid overwriting data
        return ; //mem overflow
    }
}
void ZeroStorageBuffer(void)
{
    for(uint8_t index=0;;)//fill the entire page up index !=limit
    {
        SENSOR_DATA_BUFFER.StorageArray[index++]=0;

        if(STATUSbits.C==1)//if currentSize rolls over
        {
            break;
        }
    }
    SENSOR_DATA_BUFFER.currentSize=0;
}

void transfer_Entire_Page(void)
{   
    FAST_SPI_WRITE_HEADER(&SENSOR_DATA_BUFFER.StorageArray[0]);
    
    uint8_t index=0;
    do
    {
        FAST_SPI_WRITE;
        
        FAST_SPI_WRITE;
        
        FAST_SPI_WRITE;
        
        FAST_SPI_WRITE;
        
        FAST_SPI_WRITE;
        
        FAST_SPI_WRITE;
        
        FAST_SPI_WRITE;
        
        FAST_SPI_WRITE;

    }while(index!=0);
    
    FAST_SPI_WRITE_FOOTER;
}
void transfer_partial_page(void)
{    
    FAST_SPI_WRITE_HEADER(&SENSOR_DATA_BUFFER.StorageArray[0]);

    for(uint8_t index=0;index !=SENSOR_DATA_BUFFER.currentSize;)
    {
        // each entry takes up 16 bits, so we can always write atleast twice in each loop

        FAST_SPI_WRITE;
        
        FAST_SPI_WRITE;

    }

    FAST_SPI_WRITE_FOOTER;
}
void transfer_EEPROM_BUFFER(void)
{
    
    //waitForEEPROMReady();   //see that write has completed
    WREN_Ext_EERPOM();

    ///write command
    EXTERNAL_EEPROM_CHIP_SELECT=0;

    SPI_WRITE(EXT_EEPROM_WRITE_COMMAND);

    ///followed by 24 bit address
    SPI_WRITE(SENSOR_DATA_BUFFER.CurrentAddress.bytes.Addr_1);
    SPI_WRITE(SENSOR_DATA_BUFFER.CurrentAddress.bytes.Addr_2);
    SPI_WRITE(SENSOR_DATA_BUFFER.CurrentAddress.bytes.Addr_3);
    
    ///followed by data      
    if(SENSOR_DATA_BUFFER.currentSize==0)
    {
        transfer_Entire_Page();
    }
    else
    {
        transfer_partial_page();
    }
 
    EXTERNAL_EEPROM_CHIP_SELECT=1;
    SENSOR_DATA_BUFFER.currentSize=0;
    Increment_Ext_EEPROM_PAGE(); 
   // unsigned char data_flush=SSPBUF;
    //PIR1bits.SSPIF=0;
   // SSPEN=0;
}

void Write_To_Ext_EEPROM(uint16_t Data)
{
    SENSOR_DATA_BUFFER.StorageArray[SENSOR_DATA_BUFFER.currentSize++]=(Data>>8)&0xff;
    SENSOR_DATA_BUFFER.StorageArray[SENSOR_DATA_BUFFER.currentSize++]=Data&0xff;

#if BUFFER_TRANSFER_SIZE == 256
    if(STATUSbits.C==1)//if currentSize rolls over
#else
    if (SENSOR_DATA_BUFFER.currentSize == BUFFER_TRANSFER_SIZE  )
#endif
    {
        transfer_EEPROM_BUFFER();   
        SENSOR_DATA_BUFFER.currentSize=0;
    }
}

uint16_t Read_From_Ext_EEPROM(MemoryAddress Address)
{
    spi_comms_setup(EEPROM_SPI_SETTINGS);
   
    ///write command
    EXTERNAL_EEPROM_CHIP_SELECT=0;

    SPI_MULTI_WRITE(4,
                    EXT_EEPROM_READ_COMMAND,
                    Address.bytes.Addr_1,
                    Address.bytes.Addr_2,
                    Address.bytes.Addr_3);

    uint16_t Data=READ_SPI();

    Data<<=8;
    Data+=READ_SPI();
    
    EXTERNAL_EEPROM_CHIP_SELECT=1;

    SSPEN=0;
    
    return Data;
}

/****
Enable Write Command for external EEPROM
as per datasheet
*****/
void WREN_Ext_EERPOM(void)
{
    spi_comms_setup(EEPROM_SPI_SETTINGS);

    ///Enable Write
    EXTERNAL_EEPROM_CHIP_SELECT=0;
    
    SPI_WRITE(EXT_EEPROM_WREN_COMMAND);   
    EXTERNAL_EEPROM_CHIP_SELECT=1;

}
void waitForEEPROMReady(void)
{
    spi_comms_setup(EEPROM_SPI_SETTINGS);
    //wait for the WIP status bit to indicate that the write/erase is complete

   // uint8_t infinite_loop_protection = 5;
    while(1)
    {
        
        EXTERNAL_EEPROM_CHIP_SELECT=0;
        NOP();
        SPI_WRITE(EXT_EEPROM_RDSR_COMMAND);
 
        uint8_t WRITE_STATUS = READ_SPI()&0x01;
        
        EXTERNAL_EEPROM_CHIP_SELECT=1;

        
        if(WRITE_STATUS==0)
        {
            break;
        }
//        if(--infinite_loop_protection == 0)
//        {
//            break;
//        }

        __delay_ms(5);  //max write time taken from datasheet
                        //otherwise it locks up the system
    }SSPEN=0;
}
void Reset_EEPROM(void)
{
     /***
     * Clear EEPROM
     * -enable write
     * -set status to unprotected
     * -enable write
     * -send chip erase instruction
     ****/        
    WREN_Ext_EERPOM();
    EXTERNAL_EEPROM_CHIP_SELECT=0;
    NOP();
    SPI_WRITE(EXT_EEPROM_CE_COMMAND);   

    EXTERNAL_EEPROM_CHIP_SELECT=1;
    __delay_ms(10);  //max erase time
    waitForEEPROMReady();
}
/*
 *
 *
 * Total Bytes = 2^17
 * 
 */
#endif	/* EXTERNAL_EEPROM_H */

