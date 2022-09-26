
#include "ADXL345 Driver.h"


extern void FormatSensorMetaData(  uint8_t direct_CAN_Transmission,
                            uint8_t Data_MostSig2bits,              //upper 2 bits of 10 bit value; i.e. ADRESH
                            uint8_t Data_LSB,                       //lower 8 bits of 10 bit value; i.e. ADRESL
                            uint8_t Sensor_Type,                    //0-7 bit shifted << 5; forms bits 7:5 of MSB
                            uint8_t sensor_id_with_offset) ;         //0-7 bit shifted << 3; forms bits 4:2 of MSB
                                                                    //Sensor_type, Sensor_id, Data_MS2b will from 1 byte 
static uint8_t SPI_deviceID = 0xff;

void setup_ACCL(void)
{
    if(SPI_deviceID == 0xff)
        SPI_deviceID = Add_CS_LATCH(&LATA,6);
    
    spi_comms_setup(ACC_SPI_SETTINGS);
    __delay_ms(1);                                //CS de-assertion minimum between SPI communications = 150 nS (Page 17 ADXL345 data sheet)
    uint8_t write = 0x00;                         //t_delay and t_quiet minimum time from data sheet is 5 nS
    uint8_t read = 0x80;
    uint8_t multiByte_transfer = 0x40;
    uint8_t device_ID = 0;

    SET_CS_LOW(SPI_deviceID);
    NOP();
    SPI_WRITE(read | 0x00);
    device_ID = READ_SPI();
    NOP();
    SET_CS_HIGH(SPI_deviceID);
    
    if (device_ID != 0xE5)
    {

        uint8_t accl_amt_data[8] = {0,0,0,0,0,0,0,0};
        // the function argument indicates and array of 8 bytes "uint8_t Data_to_Write[8]"
        // this is actually a complete lie!
        // the 8 is written to indicate the intended size (but it is not enforced by the compiler)
        // so you could also do this 
        //
        //      uint8_t accl_amt_data[100];
        //      Edit_COD(ACCL_AMNT_TRANSFER_FLAG_INDEX, accl_amt_data, 255) ;
        //
        // the array is "promoted" to a pointer (lookup "type promotion in c/c++")
        // that points to the address of the first element of the array; i.e. &Data_to_Write[0] 
        // we could also write the argument as "uint8_t *Data_to_Write" and it would work identically
        // so you can set your array size here to 1, since we pass a parameter "data_length" that defines its size
        //
        //      uint8_t accl_amt_data[] = {0};
        //      Edit_COD(ACCL_AMNT_TRANSFER_FLAG_INDEX, accl_amt_data, 1) ;
        //
        
        // i changed the final parameter to 1 because i saw the bug in the function "Write_to_COD" which is now fixed (data_length was ingored)
        Edit_COD(ACCL_AMNT_TRANSFER_FLAG_INDEX, accl_amt_data, 1) ;
        return ;
    }
    
    uint8_t   Accelerometer_Sample_Rate    =   check_current_value_of_8bit_OD_entry(ACCL_SAMPLE_TRANSFER_FLAG_INDEX,false);     
    uint8_t   Accelerometer_Orientation    =   check_current_value_of_8bit_OD_entry(ACCL_ORIENTATION_TRANSFER_FLAG_INDEX,false);
    /*
     * Output Data Rate   |   Rate Code   |   Decimal (User Input)
     *   3200                   1111                15              
     *   1600                   1110                14
     *   800                    1101                13
     *   400                    1100                12
     *   200                    1011                11
     *   100                    1010                10
     *   50                     1001                9
     *   25                     1000                8
     *   12.5                   0111                7
     *   6.25                   0110                6
     *   3.13                   0101                5
     *   1.56                   0100                4
     *   0.78                   0011                3
     *   0.39                   0010                2
     *   0.20                   0001                1
     *   0.10                   0000                0                   (Page 14 ADXL345 Data Sheet)
     * 
     * 
     * Accelerometer Orientation 0 = For centrifuge data collection
     * Accelerometer Orientation 1 = For robot data collection
     */
    
    
    // there is a lot of repitition; please rewrite these to accept variables
    if (Accelerometer_Orientation == 0)
    {
        SET_CS_LOW(SPI_deviceID);
        NOP();
        SPI_WRITE(multiByte_transfer | 0x1E);        //OFSX (Offset x)               
        SPI_WRITE(0b01111111);      //Offset 1g to account for accelerometer placement                  
        SPI_WRITE(0b01111111);      //Offset 1g to account for accelerometer placement                  
        SPI_WRITE(0b01000111);      //Offset 1g to account for accelerometer placement          
        NOP();
        SET_CS_HIGH(SPI_deviceID);
        __delay_us(1);
    }
    
    else
    {
        SET_CS_LOW(SPI_deviceID);
        NOP();
        SPI_WRITE(multiByte_transfer | 0x1E);        //OFSX (Offset x)               
        SPI_WRITE(0b01111111);      //Offset 1g to account for accelerometer placement                 
        SPI_WRITE(0b10111100);      //Offset 1g to account for accelerometer placement                  
        SPI_WRITE(0b01111111);      //Offset 1g to account for accelerometer placement          
        NOP();
        SET_CS_HIGH(SPI_deviceID);
        __delay_us(1);
    }
    
    SET_CS_LOW(SPI_deviceID);
    NOP();
    SPI_WRITE(multiByte_transfer | 0x2C);        //Rate register                
    SPI_WRITE(Accelerometer_Sample_Rate);      //Set rate to 100Hz                      
    SPI_WRITE(0b00001000);      //Power control value  
    SPI_WRITE(0b00000000);      //All interrupts disabled    
    NOP();
    SET_CS_HIGH(SPI_deviceID);
    __delay_us(1);
     
    SET_CS_LOW(SPI_deviceID);
    NOP();
    SPI_WRITE(write | 0x31);        //Data format register       
    SPI_WRITE(0b00000001);      //4-wire SPI mode and 4G range          
    NOP();
    SET_CS_HIGH(SPI_deviceID);
    __delay_us(1);
     
    SET_CS_LOW(SPI_deviceID);   
    NOP();
    SPI_WRITE(write | 0x38);        //FIFO Register      
    SPI_WRITE(0b00000000);        //FIFO Bypass      
    NOP();
    SET_CS_HIGH(SPI_deviceID);
    __delay_ms(10);                 //Longer delay needed after first initialization or will read data registers as 0
    
    return ;
}
void Format_ACCL_Data(uint16_t axis_accl_data,uint8_t direct_CAN_Transmission,uint8_t Sensor_ID_axis)
{   
    axis_accl_data = TwosComplement(axis_accl_data);
    FormatSensorMetaData(direct_CAN_Transmission,(axis_accl_data>>8)&0x3,axis_accl_data&0xff,Vibration_Sensor_type, Sensor_ID_axis);
}
uint16_t TwosComplement(uint16_t Data)
{
    if (Data & (1<<8))
    {
        //2's Complement
        Data = ~Data;
        Data++;
        Data = Data & 0x03ff; 
    }    
    return Data;
}
uint16_t Read_ACCL_Sensor(SensorClass Sensor,uint8_t direct_CAN_Transmission )
{ 
    spi_comms_setup(ACC_SPI_SETTINGS);
    
    uint16_t x_accl_data = 0;
    uint16_t y_accl_data = 0;
    uint16_t z_accl_data = 0;
    
    SET_CS_LOW(SPI_deviceID);
   //NOP();
    SPI_WRITE(0x80 | 0x40 | 0x32);     //Store in variable    SPI_WRITE(write | multiByte_transfer | data_Reg);    

    x_accl_data = READ_SPI();  
    x_accl_data = x_accl_data | ((uint16_t)READ_SPI()<<8);    //x high byte

    y_accl_data = READ_SPI();
    y_accl_data = y_accl_data | ((uint16_t)READ_SPI()<<8);    //y high byte

    z_accl_data = READ_SPI();
    z_accl_data = z_accl_data | ((uint16_t)READ_SPI()<<8);    //z high byte

    //this delay serves no purpose
    //NOP();
    SET_CS_HIGH(SPI_deviceID);
    //this delay serves no purpose
    //__delay_us(1);
    
    Format_ACCL_Data(x_accl_data,direct_CAN_Transmission,X_AXIS);
    Format_ACCL_Data(y_accl_data,direct_CAN_Transmission,Y_AXIS);
    Format_ACCL_Data(z_accl_data,direct_CAN_Transmission,Z_AXIS);           
    
    return 0;
}

