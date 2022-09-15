
#include "_23LC1024 Driver.h"
#include "SPI_Interface.h"
// points to chip select register and pin

// holds data before transfer to chip
static _23LC1024_MEM_BUFFER_t RamBuffer;

static uint8_t SPI_DeviceID ;

// set Mode and initialise RAM to select value (i.e. '0xff')
void init_23LC1024_RAM(uint8_t OP_MODE, uint8_t RAM_init_value)
{
    spi_comms_setup(EEPROM_SPI_SETTINGS);
	// set operating mode
    SET_CS_LOW(SPI_DeviceID);

    
	SPI_WRITE(_23LC1024_WRITE_MODE_REGISTER);
	
	SPI_WRITE(OP_MODE);
	
    SET_CS_HIGH(SPI_DeviceID);
    Clear_23LC1024(RAM_init_value);

}

void _23LC1024CheckTransferRequirement(bool ForceWrite)
{
	// current size will rollover to zero when the buffer is full
	if((ForceWrite==true) || (RamBuffer.currentSize == (_23LC1024_RAM_BUFFER_SIZE_TYPE)_23LC1024_RAM_BUFFER_SIZE))
	{
		Write_Buffer_To_23LC1024();
		
		RamBuffer.currentAddress.Addr += RamBuffer.currentSize;
		RamBuffer.currentSize=0;
	}
}
void Write_16bit_to_23LC1024_Buffer(uint16_t Data, bool ForceWrite)
{
	RamBuffer.bf[RamBuffer.currentSize++] = (Data>>8)&0xff;
	RamBuffer.bf[RamBuffer.currentSize++] = (Data>>0)&0xff;
	
	_23LC1024CheckTransferRequirement(ForceWrite);	
}
void Write_Array_to_23LC1024_Buffer(uint8_t *Data,uint8_t DataLen, bool ForceWrite)
{
	for(uint8_t index=0; index< DataLen; ++index)
	{
		RamBuffer.bf[RamBuffer.currentSize++] = Data[index];
		
		// we only want to write if we've filled up the buffer
		_23LC1024CheckTransferRequirement(false);
	}
	_23LC1024CheckTransferRequirement(ForceWrite);

}
void _23LC1024_convert24bitAddr(uint8_t *LowAddr,uint8_t *MidAddr,uint8_t *HighAddr)
{
    *LowAddr = RamBuffer.currentAddress.Addr & 0xff;
    *MidAddr = (RamBuffer.currentAddress.Addr>>8) & 0xff;
    *HighAddr = (RamBuffer.currentAddress.Addr>>16) & 0xff;
}
static void WriteAddress(void)
{
    uint8_t LowAddr;
    uint8_t MidAddr;
    uint8_t HighAddr;
    
    _23LC1024_convert24bitAddr(&LowAddr,&MidAddr,&HighAddr);
    SPI_WRITE(HighAddr);
	SPI_WRITE(MidAddr);
	SPI_WRITE(LowAddr);
	
}
void Write_Buffer_To_23LC1024(void)
{
    spi_comms_setup(EEPROM_SPI_SETTINGS);

    SET_CS_LOW(SPI_DeviceID);
	
	SPI_WRITE(_23LC1024_WRITE_COMMAND);
	
	WriteAddress();
	
	for(_23LC1024_RAM_BUFFER_SIZE_TYPE index=0; index!=RamBuffer.currentSize; index++)
	{
		SPI_WRITE(RamBuffer.bf[index]);
	}
	SET_CS_HIGH(SPI_DeviceID);

}
// copy data into ram buffer and return pointer
uint8_t *Read_23LC1024(_23LC1024_21bit_Address_t Address, uint8_t Len) 
{
	RamBuffer.currentAddress.Addr = Address;
    spi_comms_setup(EEPROM_SPI_SETTINGS);
    SET_CS_LOW(SPI_DeviceID);
	
	SPI_WRITE(_23LC1024_READ_COMMAND);
	
	WriteAddress();
	
	for(_23LC1024_RAM_BUFFER_SIZE_TYPE index=0; index!=Len; index++)
	{
		RamBuffer.bf[index] = READ_SPI();
	}
    SET_CS_HIGH(SPI_DeviceID);
	
	return RamBuffer.bf;
}
_23LC1024_21bit_Address_t getCurrent23LC1024Address(void)
{
    return RamBuffer.currentAddress.Addr;
}
_23LC1024_RAM_BUFFER_SIZE_TYPE _23LC1024getCurrentBufferSize(void)
{
    return RamBuffer.currentSize;
}
uint8_t *_23LC1024GrabDataPointer(void)
{
    return RamBuffer.bf;
}
void Clear_23LC1024_Buffer(void)
{
    RamBuffer.currentSize = 0;
}
void Clear_23LC1024_From_PDO(uint8_t *PDO_Msg, uint8_t *DLC, uint16_t *NodeID)
{
    Clear_23LC1024(0xff);
}
void Clear_23LC1024(uint8_t RAM_init_value)
{
    spi_comms_setup(EEPROM_SPI_SETTINGS);
    RamBuffer.currentAddress.Addr=0;
    SET_CS_LOW(SPI_DeviceID);

    SPI_WRITE(_23LC1024_WRITE_COMMAND);
	// start at address 0
	SPI_WRITE(0);
	SPI_WRITE(0);
	SPI_WRITE(0);
    
	// clear memory
	for(uint24_t index=0; index!=0x1ffff; index++)
	{
		SPI_WRITE(0xff);
	}
    
    SET_CS_HIGH(SPI_DeviceID);

}