#ifndef _23LC1024_DRIVER_H
#define _23LC1024_DRIVER_H
// instruction Set
#define _23LC1024_WRITE_MODE_REGISTER	(0x01)
#define _23LC1024_READ_MODE_REGISTER	(0x05)
#define _23LC1024_WRITE_COMMAND			(0x02)
#define _23LC1024_READ_COMMAND			(0x03)
#define _23LC1024_ENTER_SPI_MODE        (0xff)
// Operating Modes
#define _23LC1024_BYTE_MODE				(0)
#define _23LC1024_PAGE_MODE				(0b10 << 6)		// 36 byte page size
#define _23LC1024_SEQ_MODE				(0b01 << 6)		// preffered method

#define SET_23LC1024_CS_LOW 		*CHIP_SELECT.ChipSelect &=  (0xff^CHIP_SELECT.ChipSelect_bit)
#define SET_23LC1024_CS_HIGH		*CHIP_SELECT.ChipSelect |=  (CHIP_SELECT.ChipSelect_bit)

// chip communicates over SPI only
//
#include "_23LC1024RAM Definitions.h"	// application specific values
// microcontroller
#include "xc.h"
#include <stdbool.h>
#include <stdint.h>
// convenience struct
typedef union{

	
	struct {
		#ifdef STRUCT_HIGH_END_PACK
		uint8_t Addr3;
		uint8_t Addr2;
		uint8_t Addr1;	

		#else
		uint8_t Addr1;
		uint8_t Addr2;
		uint8_t Addr3;	
		#endif
	};
	_23LC1024_21bit_Address_t Addr;
	
}_23LC1024_BitAddress_t;

// for holding buffer before transfer
typedef struct
{
	uint8_t bf[_23LC1024_RAM_BUFFER_SIZE];			// store data to be copied to RAM
	_23LC1024_BitAddress_t currentAddress;			// address to store data at
	_23LC1024_RAM_BUFFER_SIZE_TYPE currentSize;
	
}_23LC1024_MEM_BUFFER_t;

// point to SPI chip select pin
typedef struct{
	
	REGISTER *ChipSelect;			// point to write register
	REGISTER_BIT ChipSelect_bit;	// pin number of write register
}_23LC1024_CHIP_SELECT_t;

// set operating mode and set entire RAM to RAM_init_value
void init_23LC1024_RAM(uint8_t OP_MODE, uint8_t RAM_init_value);

// copy data into buffer
// Force write will force transfer of buffer at end of function
void Write_16bit_to_23LC1024_Buffer(uint16_t Data, bool ForceWrite);					// for expediency
void Write_Array_to_23LC1024_Buffer(uint8_t *Data,uint8_t DataLen, bool ForceWrite);	// general purpose

// copy buffer to chip
void Write_Buffer_To_23LC1024(void);

// can be used to force transfer; the function is called internally whenever the buffer is edited
void _23LC1024CheckTransferRequirement(bool ForceWrite);

// point to chip select latch register 
void Set_23LC1024_CS(REGISTER *LAT_REG, REGISTER_BIT PinNumber);
// read data into ram buffer
uint8_t *Read_23LC1024(_23LC1024_21bit_Address_t Address, uint8_t Len) ;

// current address value
_23LC1024_21bit_Address_t getCurrent23LC1024Address(void);

// current size of int array
_23LC1024_RAM_BUFFER_SIZE_TYPE _23LC1024getCurrentBufferSize(void);

// access int array
uint8_t *_23LC1024GrabDataPointer(void);

// set buffer size to zero
void Clear_23LC1024_Buffer(void);

void _23LC1024_convert24bitAddr(uint8_t *LowAddr,uint8_t *MidAddr,uint8_t *HighAddr);

void Clear_23LC1024(uint8_t RAM_init_value);

// overload for 'Clear_23LC1024' - PDO callback requires argument as array
void Clear_23LC1024_From_PDO(uint8_t *RAM_init_value, uint8_t *DLC, uint16_t *NodeID);
#endif //_23LC1024_DRIVER_H