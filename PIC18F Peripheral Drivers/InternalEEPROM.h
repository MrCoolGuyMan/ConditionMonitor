/* 
 * File:   InternalEEPROM.h
 * Author: Administrator
 *
 * Created on 2 August, 2022, 3:39 PM
 */

#ifndef INTERNALEEPROM_H
#define	INTERNALEEPROM_H

#ifdef	__cplusplus
extern "C" {
#endif
#include <xc.h>
#include <stdint.h>
    
uint8_t Read_From_INTERNAL_EEPROM(uint8_t Address);
///returns the data being written if successful, otherwise 0
void Write_Data_to_INTERNAL_EEPROM(uint8_t Address, uint8_t Data[8], uint8_t Total_Bytes);
#ifdef	__cplusplus
}
#endif

#endif	/* INTERNALEEPROM_H */

