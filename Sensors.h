/* 
 * File:   ADC.h
 * Author: Administrator
 *
 * Created on 24 November, 2021, 12:50 PM
 */

#ifndef SENSORS_H
#define	SENSORS_H
#include <math.h>

#include "CustomDataTypes.h"
#define ACQT_2TAD   0b001
#define ACQT_4TAD   0b010
#define ACQT_6TAD   0b011
#define ACQT_8TAD   0b100
#define ACQT_12TAD  0b101
#define ACQT_16TAD  0b110
#define ACQT_20TAD  0b111

#define ADCS_FOSCd2     0b000
#define ADCS_FOSCd8     0b001
#define ADCS_FOSCd32    0b010
#define ADCS_FRC        0b011
#define ADCS_FOSCd4     0b100
#define ADCS_FOSCd16    0b110
#define ADCS_FOSCd64    0b110
#define ADCS_FRC2       0b111
void setup_ADC(void)
{

    VREFCON0bits.FVREN = 1; //fixed voltage reference enabled
    VREFCON0bits.FVRS = 0b11;//4.096V
    /****************************
     set ADCON1; page 295
     ****************************/
    ADCON1bits.TRIGSEL=0b1;
    ADCON1bits.PVCFG=0b10;  ///positive voltage reference bits fixed voltage reference
    ADCON1bits.NVCFG=0b00;
    /****************************
     set ADCON2; page 296
     ***************************/
    ADCON2bits.ADFM=0b1;    ///set high/low byte order
    ADCON2bits.ACQT=ACQT_8TAD; 
    
    #if USE_PLL == 1           //3x PLL
        ADCON2bits.ADCS=ADCS_FOSCd64;  //Fosc/64 
    #else
        ADCON2bits.ADCS=ADCS_FOSCd16;  //Fosc/16 
    #endif
    ADCON0bits.ADON= 1;
}
//avoids division, multiplication, bit shifts and floats for faster sampling
//we don't bother waiting for conversion to complete
//instead we will store the previous result into our data buffer while we wait for conversion
//we never turn off ADC - this is done in the interrupt vector 
void READ_FROM_ADC_QUICK(uint8_t adc_channel,uint8_t Sensor_type, uint8_t Starting_Sensor_ID)
{   
    ADCON0bits.CHS = adc_channel;
    ADCON0bits.ADON = 1;
    ADCON0bits.GO_DONE = 1 ;
    
    uint8_t sensor_id_with_offset = adc_channel-Starting_Sensor_ID+1;

    sensor_id_with_offset<<=2;
    // This is the previous result
    while(ADCON0bits.GO_DONE==1)//in case the previous wasn't completed yet
    {

    }

    FormatSensorMetaData(EEPROM_OUTPUT_MODE,ADRESH, ADRESL, Sensor_type, sensor_id_with_offset);
     ADCON0bits.ADON = 0;
   // oldSensorType = Sensor_type; 

    //oldSensor_id = sensor_id_with_offset; 

    return ;
}void READ_FROM_ADC_QUICK2(uint8_t adc_channel,uint8_t Sensor_type, uint8_t Starting_Sensor_ID)
{
    uint8_t sensor_id_with_offset = adc_channel-Starting_Sensor_ID+1;

    sensor_id_with_offset<<=2;
    
    while(ADCON0bits.GO_DONE==1)//in case the previous wasn't completed yet
    {

    }
    oldADRESH = ADRESH;
    oldADRESL = ADRESL;
    
    ADCON0bits.CHS = adc_channel;
    ADCON0bits.ADON = 1;
    ADCON0bits.GO_DONE = 1 ;

    FormatSensorMetaData(EEPROM_OUTPUT_MODE,oldADRESH, oldADRESL, oldSensorType, oldSensor_id);
    
    oldSensorType = Sensor_type; 

    oldSensor_id = sensor_id_with_offset; 

    return ;
}
uint16_t READ_FROM_ADC(uint8_t adc_channel, uint8_t Sensor_Readings, uint8_t Sensor_type, uint8_t Starting_Sensor_ID,uint8_t direct_CAN_Transmission)
{
    ADCON0bits.CHS= adc_channel;
    
    uint32_t data=0;
    uint8_t sensor_id_with_offset=0;

    for (int counter=0; counter<Sensor_Readings;counter++)
    {
        ADCON0bits.ADON= 1;
        ADCON0bits.GO_DONE=1;

        //do this first since we can't proceed until analysis is done anyway
        sensor_id_with_offset = adc_channel-Starting_Sensor_ID+1;
    
        sensor_id_with_offset<<=2;
        ///wait until "go" == 0 (i.e. complete)
        while(ADCON0bits.GO_DONE==1)
        {

        }
        uint32_t temp = (uint16_t)((ADRESH<<8)|ADRESL);
       /* if(direct_CAN_Transmission==Calculate_Current_as_RMS_Value)
        {
            temp*=temp;
        }*/
            
        data+=temp;

        ADRESH=0x00;
        ADRESL=0x00;
    }
    data/=Sensor_Readings;
    
    uint16_t final_data=0;
    
  /*  this is using a lot of memory and isn't particularly useful
   * if(direct_CAN_Transmission==Calculate_Current_as_RMS_Value)
    {
        final_data = (uint16_t) (sqrt(data));
    }
    else
    {*/
        final_data=data&0xffff;
 //   }

    ADCON0=0x00; ///Turn ADC off
    FormatSensorMetaData(direct_CAN_Transmission,(final_data>>8)&0x3, final_data&0xff,  Sensor_type,  sensor_id_with_offset);
   
    return final_data;
}
void initSensorList(void)
{
    SensorList [TempSensor].Sensor_Type                                 = Temperature_Sensor_type;
    SensorList [TempSensor].number_of_connected_sensors                 = 0;
    SensorList [TempSensor].Starting_Sensor_Number                      = TEMPERATURE_SENSOR_START_PIN;
    SensorList [TempSensor].number_of_connected_sensors_index           = TEMP_AMNT_TRANSFER_FLAG_INDEX;
    SensorList [TempSensor].Sensorfunction                              = &ReadADCSensor;       
    SensorList [TempSensor].SensorReadings                              = check_current_value_of_8bit_OD_entry(NUM_ADCS_TRANSFER_FLAG_INDEX,true);
    
    SensorList [CurrentSensor].Sensor_Type                              = Current_Sensor_type;
    SensorList [CurrentSensor].number_of_connected_sensors              = 0;
    SensorList [CurrentSensor].Starting_Sensor_Number                   = CURRENT_SENSOR_START_PIN;
    SensorList [CurrentSensor].number_of_connected_sensors_index        = CRNT_AMNT_TRANSFER_FLAG_INDEX;
    SensorList [CurrentSensor].Sensorfunction                           = &ReadADCSensor;       
    SensorList [CurrentSensor].SensorReadings                           = SensorList [TempSensor].SensorReadings;
    
    SensorList [Encoder1].Sensor_Type                                    = Encoder_Sensor_type;
    SensorList [Encoder1].number_of_connected_sensors                    = 0;
    SensorList [Encoder1].Starting_Sensor_Number                         = 0;
    SensorList [Encoder1].number_of_connected_sensors_index              = MOTOR_ENCODER_ENABLED_CAN_INDEX;
    SensorList [Encoder1].Sensorfunction                                 = &storeEncoderData; 
    SensorList [Encoder1].SensorReadings                                 = 0;

    SensorList [VibrationSensor].Sensor_Type                            = Vibration_Sensor_type,
    SensorList [VibrationSensor].number_of_connected_sensors            = 0,
    SensorList [VibrationSensor].Starting_Sensor_Number                 = ACCL_SENSOR_START_PIN,
    SensorList [VibrationSensor].number_of_connected_sensors_index      = ACCL_AMNT_TRANSFER_FLAG_INDEX;
    SensorList [VibrationSensor].Sensorfunction                         = &Read_ACCL_Sensor ;

      
    return ;
}
void setupSensorListCODParameters(void)
{
    ADRESH = 0;
    ADRESL = 0;
    MonitoringModeOutput                        = check_current_value_of_8bit_OD_entry(SENSOR_MONITORING_OUTPUT_MODE,true);
    SensorList [TempSensor].SensorReadings      = check_current_value_of_8bit_OD_entry(NUM_ADCS_TRANSFER_FLAG_INDEX,true);
    SensorList [CurrentSensor].SensorReadings   = SensorList [TempSensor].SensorReadings;

    SENSOR_DATA_BUFFER.currentSize=0;

    for(uint8_t index =0; index <NumberOfSensorsInSensorList ; index++)
    {
        SensorList[index].number_of_connected_sensors=check_current_value_of_8bit_OD_entry(

                SensorList[index].number_of_connected_sensors_index,true

                );
    }
}

uint16_t ReadADCSensor(SensorClass Sensor,uint8_t direct_CAN_Transmission)
{
    uint16_t data=0;
    for(uint8_t sensor_id=Sensor.Starting_Sensor_Number;
                sensor_id<Sensor.Starting_Sensor_Number+Sensor.number_of_connected_sensors;
                sensor_id++)
    {

        //read adc sample without any processing
        if(direct_CAN_Transmission == EEPROM_OUTPUT_MODE)
        {
            READ_FROM_ADC_QUICK
                    (
                        sensor_id, 
                        Sensor.Sensor_Type,
                        Sensor.Starting_Sensor_Number
                    );
        }
        //CAN Bus tranmission slows sampling down so much we might as well filter result
        else
        {
            data = READ_FROM_ADC
                    (  
                        sensor_id, 
                        Sensor.SensorReadings,
                        Sensor.Sensor_Type,
                        Sensor.Starting_Sensor_Number,
                        direct_CAN_Transmission
                    );
        }
    }
    return data;    //return value only holds data for the final sensor; used for DC Motor test
}
void SensorHeader(uint8_t SensorListPosition,uint8_t direct_CAN_Transmission)
{
    if(SensorList[SensorListPosition].number_of_connected_sensors==0)
    {
        return;
    }

    SensorList[SensorListPosition].Sensorfunction(SensorList[SensorListPosition], direct_CAN_Transmission);
}
void ConvertADCToCurrent(int32_t *newdata)
{
    int32_t OFFSET_VOLTAGE =0xffff & (check_current_value_of_32bit_OD_entry(CRNT_OFFSET_VOLTAGE_FLAG_INDEX,false)); 

    //divide by 0.4 because of sensor resolution 0f 400mV/A
    OFFSET_VOLTAGE *=10;        //multiply by 10

    OFFSET_VOLTAGE >>=2;        //divide by 4
                                //offset voltage is now Amps offset
    //new data is ADC counts
    *newdata*=10;               // for 4.048 v reference:   4.096/1024          = 4 mV/ADC count
                                //                          0.01 A/ADC Count    = (4mV/ADC Count) / (400mV/A)        
                                //                          10 mA/ADC Count     = 0.01 A/ADC Count x (1000mA / 1A)
    *newdata-=(OFFSET_VOLTAGE); // result in mA

    if(*newdata<0)
    {
        *newdata = -*newdata;
    }   
}
/*******************************
send a message like so
[byte1]		[byte2]		[byte3]		[byte4]		[byte5][byte6][byte7]
[SensorType][SensorID]	[Val_LSB]	[Val_MSB]	[EEPROM ADDR........]
*******************************/
void Transmit_Sensor_Data(uint16_t TX_Data, uint8_t Addr4, uint8_t Addr3, uint8_t Addr2, uint8_t Addr1)
{
    spi_comms_setup(DEFAULT_SPI_SETTINGS);
    ///parse out EEPROM format to transmit format
        ///ADC data is the 10 LSBs
        uint16_t ADC_DATA   =TX_Data&0x3FF;            ///ignore meta-data
        uint8_t Msg_1       =(ADC_DATA>>0)&0xff;           ///we can only send 8 bits at a time on SPI
        uint8_t Msg_2       =(ADC_DATA>>8)&0xff;
        
        ///remaining bits are meta-data
        uint8_t meta_data   =(TX_Data>>10);            ///remove ADC data
        uint8_t sensor_id   = meta_data&0x7;         ///last 3 bits
        uint8_t SENSOR_TYPE =(meta_data>>3)&0x7;

        if(TX_Data == 0xffff)    //i.e. EEPROM empty
        {
            sensor_id = 0xff;
            SENSOR_TYPE = 0xff;
            Msg_1 = 0xff;
            Msg_2 = 0xff;        //otherwise the message can cause aliasing -> 07 07 FF 07
        }
    ///find node id and transform into 11 bit ID
        
        CopyNodeIDToCANController(0x00,0x180, CAN_TXB1CTRL_REG,false);

        uint8_t msg_DLC=8;    
        Fill_TX_REG_of_CAN_CONTROLLER(CAN_TXB1CTRL_REG+5,msg_DLC+1,msg_DLC,SENSOR_TYPE,sensor_id,Msg_1,Msg_2, Addr4,Addr3,Addr2,Addr1);

        transmit_CAN_Message(CAN_RTS_TXB1);

    SSPCON1bits.SSPEN=0;
}

uint16_t CopyTimerToEncoderValue(volatile uint16_t *Timer,uint8_t direct_CAN_Transmission, uint16_t MetaData)
{
    uint16_t OutputData = *Timer;
    *Timer=0;
    
    OutputData+=MetaData;
    
    if(direct_CAN_Transmission==EEPROM_OUTPUT_MODE) 
    {
        Write_To_Ext_EEPROM(OutputData);
    }
    else if(direct_CAN_Transmission==CAN_OUTPUT_MODE_1)  //Transmit raw with Metadata
    {
        Transmit_Sensor_Data(OutputData,0,0,0,0x00);
    }
    else if(direct_CAN_Transmission==CAN_OUTPUT_MODE_2)  //Transmit in Decimal without metadata
    {
        transmit_readable_sensor_data(OutputData, 'E',  Encoder_Sensor_type);
    } 
    return OutputData;
}
uint16_t storeEncoderData(SensorClass Sensor,uint8_t direct_CAN_Transmission)
{
    uint16_t OutputData=CopyTimerToEncoderValue(&TMR1,direct_CAN_Transmission,Encoder_Meta_Data);
    
    if (Sensor.number_of_connected_sensors == 2)
        CopyTimerToEncoderValue(&TMR3,direct_CAN_Transmission,Encoder2_Meta_Data);

    return OutputData;
}
void transmit_monitoring_summary(void)
{
    uint32_t total_time = Timers.OperatingTime_ms-MonitoringModeStartTime;
    //show the user how many samples were taken
    uint32_t total_samples =  SENSOR_DATA_BUFFER.CurrentAddress._24_bit_Address+
                              SENSOR_DATA_BUFFER.currentSize;

    convertHexToDecAndTransmit('S',total_samples/2,true);        


    //show user how long it took
    total_time/=ProgramTimerDivider;
    convertHexToDecAndTransmit('T',total_time,true);
    
    OperatingModeFlags.MonitoringModeActive =0;
}
void convertSensorFrequencyToClockMultiple(void)
{
#define DONT_USE_FLOATS
#ifndef DONT_USE_FLOATS
    // this is using 7% of total memory by iteself!
    float Rate = (float)((1000.0/Sensor_Checking_Interval)/TIMER0_INTERRUPT_TIME_us);
    Rate*=1000;
#else
#define CONVERT_MICRO_TO_SECONDS (1000000UL)
    uint32_t Rate = (uint32_t)((CONVERT_MICRO_TO_SECONDS/Sensor_Checking_Interval)/TIMER0_INTERRUPT_TIME_us);
   
#endif
#undef DONT_USE_FLOATS
    Sensor_Checking_Interval = (uint16_t)Rate;
}
void setup_monitoring_mode(uint8_t Mode)
{
    //configure the accelerometer
    
    OperatingModeFlags.MonitoringModeActive = Mode;
    
    //turn monitoring on
    if(OperatingModeFlags.MonitoringModeActive == 1)
    {
        setup_ACCL();
        Sensor_Checking_Interval    =   check_current_value_of_16bit_OD_entry(SENSOR_LOGGING_RATE_CAN_INDEX,false);
        
        convertSensorFrequencyToClockMultiple();
        
        setupSensorListCODParameters();
        MonitoringModeStartTime = Timers.OperatingTime_ms;
    }
    //turn monitoring off
    else
    {
        //transfer partial page
        if (SENSOR_DATA_BUFFER.currentSize != 0)
            transfer_EEPROM_BUFFER();
        
        transmit_monitoring_summary();
        
        ZeroStorageBuffer();
    }
    TMR1 = 0; //reads encoder1
    TMR3 = 0; //reads encoder2
    Timers.SensorFlag=0;
}
void transmit_readable_sensor_data(uint32_t OutputData, char sensor_identifier, uint8_t Sensor_Type)
{
    int32_t newdata=OutputData&0x3ff; 

    if(Sensor_Type == Current_Sensor_type)
    {
        ConvertADCToCurrent(&newdata);
    }

    convertHexToDecAndTransmit(sensor_identifier,(uint16_t)newdata, True);    
}
void FormatSensorMetaData(  uint8_t direct_CAN_Transmission,
                            uint8_t Data_MostSig2bits,              //upper 2 bits of 10 bit value; i.e. ADRESH
                            uint8_t Data_LSB,                       //lower 8 bits of 10 bit value; i.e. ADRESL
                            uint8_t Sensor_Type,                    //0-7 bit shifted << 5; forms bits 7:5 of MSB
                            uint8_t sensor_id_with_offset)          //0-7 bit shifted << 3; forms bits 4:2 of MSB
                                                                    //Sensor_type, Sensor_id, Data_MS2b will from 1 byte 
{
    
    /**
     * 6 bits of meta data contain the sensor type and sensor id
     * sensor type specifier; 3 bits range [0 - 8]
     * Currently Allocated
     * 00 = temperature
     * 01 = current
     * 02 = acceleration - (Sensor ID - 01=x 02=y 03=z)
     * 
     * there are max 8 sensors of each type (3 bits)
     * 
     * There are a max of 10 bits of data reserved for the sensor
     * 
     * DATA FIELD   |  SENSOR_TYPE  SENSOR_ID   SENSOR_DATA
     * DATA BITS    |   <15:13>     <12:10>     <9:0>
     * 
     **/
    //SENSOR_TYPE
    /*
     *  20 instructions @16MHz = 5us
     */
 
    uint8_t Mem_meta_data=Sensor_Type;                              //Max value is  0b00111000
    Mem_meta_data+=sensor_id_with_offset;                           //Max sensor is 0b11100 -> 0b11111100
    Mem_meta_data+=Data_MostSig2bits;                               //Max value is 0b11111111      
     
    ///either the data can be stored in the external eeprom
    uint16_t FinalOutputData = Data_LSB + ((uint16_t)Mem_meta_data<<8);
    if(direct_CAN_Transmission==EEPROM_OUTPUT_MODE) 
    {
        Write_To_Ext_EEPROM(FinalOutputData);   
        return;
    }
    //or directly transmitted on the can bus
    //we don't care about efficiency from here on
    
    
    if(direct_CAN_Transmission==CAN_OUTPUT_MODE_1)  //Transmit raw with Metadata
    {
        Transmit_Sensor_Data(FinalOutputData,0,0,0,0x00);
    }
    else if(direct_CAN_Transmission==CAN_OUTPUT_MODE_2)  //Transmit in Decimal without metadata
    {
        transmit_readable_sensor_data(FinalOutputData, 'A',  Sensor_Type);
    } 
}
void store_CAN_MSG(uint8_t msg_marker, uint8_t data_value)
{
	uint8_t First_EEPROM_byte =CAN_MSG_Meta_Data + msg_marker;
	
	uint16_t EEPROM_DUMP = (uint16_t)(First_EEPROM_byte<<8) + data_value;
	
	Write_To_Ext_EEPROM(EEPROM_DUMP);
}

void log_CAN_MESSAGE_TO_EEPROM( uint8_t Data[8],uint8_t DLC, uint16_t Node_ID)
{
	uint8_t SOM_MARKER = 0b100000;
    uint8_t Node_ID_L=Node_ID&0xff;
    uint8_t Node_ID_H=(Node_ID&0xff00)>>8;
    
    
    //store 03 08 MSG.ID_H <empty>
    //      03 00 MDG.ID_L <empty>
    //      03 00 MSG.DATA[0] <empty>
    //      .   .   .
    //      03 00 MSG.Data[x] <emtpy>
	store_CAN_MSG(SOM_MARKER, Node_ID_H);
    
	store_CAN_MSG(00, Node_ID_L);
    
	for (uint8_t counter=0; counter< DLC; counter++)
	{
		uint8_t EOM_MARKER =0b000000;
		
		if(counter == DLC-1)
		{
			EOM_MARKER=0b000100;
		}
		store_CAN_MSG(EOM_MARKER, Data[counter]);
	} 
    spi_comms_setup(DEFAULT_SPI_SETTINGS);
}
void setup_ACCL(void)
{
    spi_comms_setup(ACC_SPI_SETTINGS);
    __delay_ms(1);                                //CS de-assertion minimum between SPI communications = 150 nS (Page 17 ADXL345 data sheet)
    uint8_t write = 0x00;                         //t_delay and t_quiet minimum time from data sheet is 5 nS
    uint8_t read = 0x80;
    uint8_t multiByte_transfer = 0x40;
    uint8_t device_ID = 0;

    ChipSelect_ACCL  = 0; //SS' in enable
    NOP();
    SPI_WRITE(read | 0x00);
    device_ID = READ_SPI();
    NOP();
    ChipSelect_ACCL  = 1; //SS' disable
    
    if (device_ID != 0xE5)
    {
        SensorList [VibrationSensor].number_of_connected_sensors = 0;
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
        return;
    }
    
    uint8_t   Accelerometer_Sample_Rate    =   check_current_value_of_8bit_OD_entry(ACCL_SAMPLE_TRANSFER_FLAG_INDEX,false);   
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
     */
    
    ChipSelect_ACCL  = 0; //SS' in enable   
    NOP();
    SPI_WRITE(multiByte_transfer | 0x1E);        //OFSX (Offset x)               
    SPI_WRITE(0b00000000);      //No offset required                    
    SPI_WRITE(0b01000000);      //Offset 1g to account for accelerometer placement                  
    SPI_WRITE(0b01000000);      //Offset 1g to account for accelerometer placement          
    NOP();
    ChipSelect_ACCL  = 1; //SS' disable 
    __delay_us(1);
    
    ChipSelect_ACCL  = 0; //SS' in enable
    NOP();
    SPI_WRITE(multiByte_transfer | 0x2C);        //Rate register                
    SPI_WRITE(Accelerometer_Sample_Rate);      //Set rate to 100Hz                      
    SPI_WRITE(0b00001000);      //Power control value  
    SPI_WRITE(0b00000000);      //All interrupts disabled    
    NOP();
    ChipSelect_ACCL  = 1; //SS' disable 
    __delay_us(1);
     
    ChipSelect_ACCL  = 0; //SS' in enable
    NOP();
    SPI_WRITE(write | 0x31);        //Data format register       
    SPI_WRITE(0b00000001);      //4-wire SPI mode and 4G range          
    NOP();
    ChipSelect_ACCL  = 1; //SS' disable  
    __delay_us(1);
     
    ChipSelect_ACCL  = 0; //SS' in enable      
    NOP();
    SPI_WRITE(write | 0x38);        //FIFO Register      
    SPI_WRITE(0b00000000);        //FIFO Bypass      
    NOP();
    ChipSelect_ACCL  = 1; //SS' disable
    __delay_ms(10);                 //Longer delay needed after first initialization or will read data registers as 0
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
void Format_ACCL_Data(uint16_t axis_accl_data,uint8_t direct_CAN_Transmission,uint8_t Sensor_ID_axis)
{   
    axis_accl_data = TwosComplement(axis_accl_data);
    FormatSensorMetaData(direct_CAN_Transmission,(axis_accl_data>>8)&0x3,axis_accl_data&0xff,Vibration_Sensor_type, Sensor_ID_axis);
}

uint16_t Read_ACCL_Sensor(SensorClass Sensor,uint8_t direct_CAN_Transmission )
{ 
    spi_comms_setup(ACC_SPI_SETTINGS);
    
    uint16_t x_accl_data = 0;
    uint16_t y_accl_data = 0;
    uint16_t z_accl_data = 0;
    
    ChipSelect_ACCL = 0; //SS' in enable 
    NOP();    
    SPI_WRITE(0x80 | 0x40 | 0x32);     //Store in variable    SPI_WRITE(write | multiByte_transfer | data_Reg);    

    /*
     * your code here was fine but the compiler is not doing a great job
     * 
     * normally it should inline the function "READ_SPI" but it does not
     * 
     * so it ends up saving around a 100 instructions to use the macro
     */
#ifdef oldACCL
    x_accl_data = READ_SPI();  
    x_accl_data = x_accl_data | ((uint16_t)READ_SPI()<<8);    //x high byte

    y_accl_data = READ_SPI();
    y_accl_data = y_accl_data | ((uint16_t)READ_SPI()<<8);    //y high byte

    z_accl_data = READ_SPI();
    z_accl_data = z_accl_data | ((uint16_t)READ_SPI()<<8);    //z high byte
#else
// can be used without restiction if speed is required
    

    READ_16_BIT_SPI(x_accl_data);
    READ_16_BIT_SPI(y_accl_data);
    READ_16_BIT_SPI(z_accl_data);

#endif
    //this delay serves no purpose
    NOP();
    ChipSelect_ACCL  = 1; //SS' disable
    //this delay serves no purpose
    __delay_us(1);
    
    Format_ACCL_Data(x_accl_data,direct_CAN_Transmission,X_AXIS);
    Format_ACCL_Data(y_accl_data,direct_CAN_Transmission,Y_AXIS);
    Format_ACCL_Data(z_accl_data,direct_CAN_Transmission,Z_AXIS);           
    
    return 0;
}


#endif	/* SENSORS_H */

