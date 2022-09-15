/* 
 * File:   humidity sensor.h
 * Author: Administrator
 *
 * Created on 1 August, 2022, 3:46 PM
 */

#ifndef BME_280_HUMIDITY_SENSOR_H
#define	BME_280_HUMIDITY_SENSOR_H

#ifdef	__cplusplus
extern "C" {
#endif

int32_t t_fine;
void setup_BME280(void)
{
    return;
   spi_comms_setup(HUMID_SPI_SETTINGS);
    __delay_ms(1);      
    // for commands that will never change it is better to define a macro
    // #define BME280_READ_COMMAND 0x80
    // otherwise we add overhead
    // e.g. for "read | 0xF7" the compiler will do the inclusive OR operation at run time
    // wheras if "BME280_READ_COMMAND|0xf7" will be calculated at compile time
    uint8_t write = 0x7F;                 
    uint8_t read = 0x80;
    uint8_t device_ID = 0x00;
    ChipSelect_HUMID  = 0; //SS' in enable
    NOP();
    SPI_WRITE(read | 0xD0);
    device_ID = READ_SPI();
    NOP();
    ChipSelect_HUMID  = 1; //SS' disable
    __delay_us(1);
    ChipSelect_HUMID  = 0; //SS' in enable
    NOP();                        // Writing in SPI for the BME280 requires the full register address without bit 7 and the writing command. 
                                  // Several pairs can be written without raising CSB. NOT auto-incremented!!
    SPI_WRITE(write & 0xF2);      // 0xF2 (ctrl_hum) - Controls oversampling of humidity data
    SPI_WRITE(0b00000001);
    SPI_WRITE(write & 0xF4);      // 0xF4 (ctrl_meas) - Controls oversampling of temperature/pressure data. Controls sensor mode of the device
    SPI_WRITE(0b00100111);
    SPI_WRITE(write & 0xF5);      // 0xF5 (config) - Sets the rate, filter and interface options of the device
    SPI_WRITE(0b00000000);
    NOP();
    ChipSelect_HUMID  = 1; //SS' disable
    __delay_us(1);
}
uint16_t Read_BME280_Sensor(SensorClass Sensor, uint8_t direct_CAN_Transmission)
{
    spi_comms_setup(HUMID_SPI_SETTINGS);
    __delay_ms(1);  
    
    
    // for commands that will never change it is better to define a macro
    // #define BME280_READ_COMMAND 0x80
    // otherwise we add overhead
    // e.g. for "read | 0xF7" the compiler will do the inclusive OR operation at run time
    // wheras if "BME280_READ_COMMAND|0xf7" will be calculated at compile time
    // the register "f7" should also be made into a macro
    uint8_t read = 0x80;
    
    
    // the initialisation here should be removed
    // The compiler will clear all these register before copying the SSPBUF into them
    // this is needless
    
    // CLRF pressure_MSB, A
    // .... read SPI
    // MOVFF SSPBUF, pressure_MSB
    // this add a redundant operation for each register
    uint8_t pressure_MSB = 0x00;
    uint8_t pressure_LSB = 0x00;
    uint8_t pressure_XLSB = 0x00;
    int32_t pressure = 0x00;
    uint8_t temperature_MSB = 0x00;
    uint8_t temperature_LSB = 0x00;
    uint8_t temperature_XLSB = 0x00;
    int32_t temperature = 0x00;
    uint8_t humidity_MSB = 0x00;
    uint8_t humidity_LSB = 0x00;
    int32_t humidity = 0x00;
    
    ChipSelect_HUMID  = 0; //SS' in enable
    NOP();                      // Reading in SPI for the BME280 the register address is incremented automatically
    SPI_WRITE(read | 0xF7);
    pressure_MSB = READ_SPI();       //Contains the MSB part up[19:12] of the raw pressure measurement output data
    pressure_LSB = READ_SPI();       //Contains the LSB part up[11:4] of the raw pressure measurement output data
    pressure_XLSB = READ_SPI();      //Contains the XLSB part up[3:0] of the raw pressure measurement output data
    temperature_MSB = READ_SPI();    //Contains the MSB part ut[19:12] of the raw temperature measurement output data 
    temperature_LSB = READ_SPI();    //Contains the LSB part ut[11:4] of the raw temperature measurement output data 
    temperature_XLSB = READ_SPI();   //Contains the XLSB part ut[3:0] of the raw temperature measurement output data 
    humidity_MSB = READ_SPI();       //Contains the MSB part uh[15:8] of the raw humidity measurement output data
    humidity_LSB = READ_SPI();       //Contains the LSB part uh[7:0] of the raw humidity measurement output data
    NOP();
    ChipSelect_HUMID  = 1; //SS' disable
    
    temperature = ((int32_t)temperature_MSB<<12) | ((int32_t)temperature_LSB<<4) | ((int32_t)temperature_XLSB);
    pressure = ((int32_t)pressure_MSB<<12) | ((int32_t)pressure_LSB<<4) | ((int32_t)pressure_XLSB);
    humidity = ((int32_t)humidity_MSB<<8) | ((int32_t)humidity_LSB);
    
    int32_t temp_final = BME280_compensate_T_int32(temperature);
    uint32_t press_final = BME280_compensate_P_int32(pressure);
    uint32_t humid_final = BME280_compensate_H_int32(humidity);
    
    convertHexToDecAndTransmit('H', (uint32_t)temp_final, true);
    convertHexToDecAndTransmit('H', press_final, true);
    convertHexToDecAndTransmit('H', humid_final, true);
    
    FormatSensorMetaData(direct_CAN_Transmission,(temp_final>>23)&0x3,temp_final&0xff,Humidity_Sensor_type, HUMIDITY);
    
    return 0;
}

int32_t BME280_compensate_T_int32 (int32_t adc_T)
{
    int32_t var1, var2, T;
    int32_t dig_T1, dig_T2, dig_T3;
    
    // this should be made into a macro
    uint8_t read = 0x80;
    spi_comms_setup(HUMID_SPI_SETTINGS);
    // this delay is not needed
    __delay_ms(1);   
    
    ChipSelect_HUMID  = 0; //SS' in enable
    NOP();
    SPI_WRITE(read | 0x88);      //0x88
    dig_T1 = READ_SPI();
    dig_T1 = dig_T1 | ((int16_t)READ_SPI()<<8);
    dig_T2 = READ_SPI();
    dig_T2 = dig_T2 | ((int16_t)READ_SPI()<<8);
    dig_T3 = READ_SPI();
    dig_T3 = dig_T3 | ((int16_t)READ_SPI()<<8);
    
    // i dont see any value in this delay; we have finished talking to the sensor
    NOP();  
    ChipSelect_HUMID  = 1; //SS' disable
    
    var1 = ((((adc_T>>3) - (dig_T1<<1))) * (dig_T2)) >> 11;
    var2 = (((((adc_T>>4) - (dig_T1)) * ((adc_T>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

uint32_t BME280_compensate_P_int32 (int32_t adc_P)
{
    spi_comms_setup(HUMID_SPI_SETTINGS);
    // this delay is not needed
    __delay_ms(1);   
    
    // this should be made into a macro
    uint8_t read = 0x80;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    uint16_t dig_P1;
    
    ChipSelect_HUMID  = 0; //SS' in enable
    
    // between the previous command and the SPI_Write we will have something like this
    // Movlw 0x80       ; 1 instruction cycle; set WREG = 0x80
    // Iorlw 0x8e       ; 1 instruction cycle; inclusive OR literal with WREG
    // CALL SPI_WRITE   ; 2 instruction cycles; Function call
    // the extra NOP() is adding an extra 1/12000000 seconds (83 ns) to the delay 
    NOP();
    SPI_WRITE(read | 0x8E);      //0x88
    
    // it's probably better to write a new functionc called "uint16_t READ_SPI_WORD"
    // this will reduce the FLASH usage quite a lot
    dig_P1 = READ_SPI();                                //Compensation parameters
    dig_P1 = dig_P1 | ((uint16_t)READ_SPI()<<8);
    dig_P2 = READ_SPI();
    dig_P2 = dig_P2 | ((int16_t)READ_SPI()<<8);
    dig_P3 = READ_SPI();
    dig_P3 = dig_P3 | ((int16_t)READ_SPI()<<8);
    dig_P4 = READ_SPI();
    dig_P4 = dig_P4 | ((int16_t)READ_SPI()<<8);
    dig_P5 = READ_SPI();
    dig_P5 = dig_P5 | ((int16_t)READ_SPI()<<8);
    dig_P6 = READ_SPI();
    dig_P6 = dig_P6 | ((int16_t)READ_SPI()<<8);
    dig_P7 = READ_SPI();
    dig_P7 = dig_P7 | ((int16_t)READ_SPI()<<8);
    dig_P8 = READ_SPI();
    dig_P8 = dig_P8 | ((int16_t)READ_SPI()<<8);
    dig_P9 = READ_SPI();
    dig_P9 = dig_P9 | ((int16_t)READ_SPI()<<8);
    // this NOP has no benefit
    NOP();
    ChipSelect_HUMID  = 1; //SS' disable
    
    int32_t var1,var2;
    uint32_t p;
    
    // this looks very complicated; can you please work on formatting; add some comments; explain the equations
    // please use meaningful variables names
    var1 = (((int32_t)t_fine)>>1)-(int32_t)64000;
    var2 = (((var1>>2)*(var1>>2))>>11)*((int32_t)dig_P6);
    var2 = var2 + ((var1*((int32_t)dig_P5))<<1);
    var2 = (var2>>2)+(((int32_t)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2))>>13))>>3)+((((int32_t)dig_P2)*var1)>>1))>>18;
    var1 = ((((32768+var1))*((int32_t)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }
    p = (((uint32_t)(((int32_t)1048576)-adc_P)-((uint32_t)var2>>12)))*3125;
    
    // uint8_t 
    if (p<0x80000000)
    {
        p = (p<<1)/((uint32_t)var1);
    }
    else
    {
        p = (p/(uint32_t)var1)*2;
    }
    var1 = (((int32_t)dig_P9)*((int32_t)(((p>>3)*(p>>3))>>13)))>>12;
    var2 = (((int32_t)(p>>2))*((int32_t)dig_P8))>>13;
    p = (uint32_t)((int32_t)p+((var1+var2+dig_P7)>>4));
    return p;
}

uint32_t BME280_compensate_H_int32 (int32_t adc_H)
{
    spi_comms_setup(HUMID_SPI_SETTINGS);
    __delay_ms(1);   
    uint8_t read = 0x80;
    int16_t  dig_H2, dig_H4, dig_H4LH5H, dig_H5;
    uint8_t dig_H1, dig_H3;
    int8_t dig_H6;
    int32_t v_x1_u32r;
    
    ChipSelect_HUMID  = 0; //SS' in enable
    NOP();
    SPI_WRITE(read | 0xA1);      
    dig_H1 = READ_SPI();
    NOP();
    ChipSelect_HUMID  = 1; //SS' disable
    
    ChipSelect_HUMID  = 0; //SS' in enable
    NOP();
    SPI_WRITE(read | 0xE1);      
    dig_H2 = READ_SPI();
    dig_H2 = dig_H2 | ((int16_t)READ_SPI()<<8);
    dig_H3 = READ_SPI();
    dig_H4 = READ_SPI();
    dig_H4LH5H = READ_SPI();
    dig_H5 = READ_SPI();
    dig_H6 = (int8_t)(READ_SPI());
    NOP();
    ChipSelect_HUMID  = 1; //SS' disable
    
    dig_H4 = (dig_H4LH5H&0b000111) | (dig_H4<<4);
    dig_H5 = dig_H5 | ((dig_H4LH5H&0b111000)>>3);
    
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14)-(((int32_t)dig_H4)<<20)-(((int32_t)dig_H5)*v_x1_u32r))\
            +((int32_t)16384))>>15)*(((((((v_x1_u32r *((int32_t)dig_H6))>>10)*(((v_x1_u32r\
            *((int32_t)dig_H3))>>11)+((int32_t)32768)))>>10)+((int32_t)2097152))*\
            ((int32_t)dig_H2)+8192)>>14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r>>15)*(v_x1_u32r>>15))>>7)*((int32_t)dig_H1))>>4));
    //v_x1_u32r = (v_x1_u32r < 0 ? 0:v_x1_u32r);
    //v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400:v_x1_u32r);
    if (v_x1_u32r < 0)
        v_x1_u32r = 0;
    else
        v_x1_u32r = v_x1_u32r;
    if (v_x1_u32r > 419430400)
        v_x1_u32r = 419430400;
    else
        v_x1_u32r = v_x1_u32r;
    return (uint32_t)(v_x1_u32r>>12);
}



#ifdef	__cplusplus
}
#endif

#endif	/* HUMIDITY_SENSOR_H */

