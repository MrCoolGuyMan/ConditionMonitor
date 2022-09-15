/* 
 * File:   EEPROM.h
 * Author: Administrator
 *
 * Created on 28 March, 2022, 3:54 PM
 */

#ifndef EEPROM_H
#define	EEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif


void transmit_internal_EEPROM(uint16_t startAddress, uint16_t endAddress, uint8_t EEPROM_OR_RAM)
{
    transmitln();
    for(uint16_t Addr = startAddress; Addr <=endAddress;Addr++)
    {
        uint8_t data = 0;
        
        if(EEPROM_OR_RAM==0)
        {
            data = Read_From_INTERNAL_EEPROM(Addr&0xff);        
        }
        if(EEPROM_OR_RAM==1)
        {
            FSR2 = Addr;
            data = INDF2;
        }

        USART_TX_STRING_AS_ASCII(data);

        USART_TX_char(' ');

        if (Addr != 0)
        {     
            if (Addr % 10 == 0)
            {
                transmitln();
            }
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


#ifdef	__cplusplus
}
#endif

#endif	/* EEPROM_H */

