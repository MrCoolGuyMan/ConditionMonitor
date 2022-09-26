
#include "Sensors.h"
/* 
 * File:   ADC.h
 * Author: Administrator
 *
 * Created on 24 November, 2021, 12:50 PM
 */

    static uint24_t MonitoringModeStartTime;

    static uint16_t Sensor_Checking_Interval=Sampling_period_TMR_Interrupts;
    //holds list of sensors to be measured during monitoring process
    static SensorClass SensorList[NumberOfSensorsInSensorList];
    static uint8_t MonitoringModeActive;  
    
    extern TimingFlags Timers;
 
         
uint8_t checkMonitoringActive(void)
{
    return MonitoringModeActive;
}
bool CheckIfSensorIntervalTimeHasArrived(void)
{
    if(Timers.SensorFlag>=Sensor_Checking_Interval)  //user definable period; 1ms by default
    {
        return true;
    }
    return false;
}
/****************************
record the value from each type of sensor in turn
****************************/
void Begin_sensor_monitoring(bool realTimeTransfer)
{
    for(uint8_t index =0; index <NumberOfSensorsInSensorList ; index++)
    {
        SensorHeader(index,EEPROM_OUTPUT_MODE);
    }
    if(realTimeTransfer == false)
        _23LC1024CheckTransferRequirement(true);

}
void AddSensor(uint8_t Type, uint8_t startingID, uint24_t COD_INDEX_NUM_CONNECTED, Sensorfunction_t Callback,Sensorsetupfunction_t SetupFunc)
{
    uint8_t TypeAsMetaValue = (Type << 5)&0xff;
    SensorList [Type].Sensor_Type                                 = TypeAsMetaValue;
    SensorList [Type].number_of_connected_sensors                 = 0;
    SensorList [Type].Starting_Sensor_Number                      = startingID;
    SensorList [Type].number_of_connected_sensors_index           = COD_INDEX_NUM_CONNECTED;
    SensorList [Type].Sensorfunction                              = Callback;       
    SensorList [Type].SetupFunction                               = SetupFunc;
}   

void setupSensorListCODParameters(void)
{
    ADRESH = 0;
    ADRESL = 0;

    /*
     call the function to initialise the sensor
     */
    for(uint8_t index=0; index<NumberOfSensorsInSensorList; index++)
    {
        if(SensorList [index].SetupFunction != NULL)
        {
            SensorList [index].SetupFunction();
        }
    }
    /*
     Call the function to execute the sensor reading
     */
    for(uint8_t index =0; index <NumberOfSensorsInSensorList ; index++)
    {
        SensorList[index].number_of_connected_sensors=check_current_value_of_8bit_OD_entry(

                SensorList[index].number_of_connected_sensors_index,true

                );
       
        //encoder 1 and 2 are read in through the same can index
        if(index == Encoder2)
        {
            if(SensorList[index].number_of_connected_sensors != 2)
            {
                SensorList[index].number_of_connected_sensors = 0;
            }
        }
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
        uint8_t sensorDataFormatted = READ_FROM_ADC_QUICK
                (
                    sensor_id, 
                    Sensor.Sensor_Type,
                    Sensor.Starting_Sensor_Number
                );
        FormatSensorMetaData(direct_CAN_Transmission,ADRESH, ADRESL, Sensor.Sensor_Type, sensorDataFormatted);
    }
    return data;    //return value only holds data for the final sensor; used for DC Motor test
}
void SensorHeader(uint8_t SensorListPosition,uint8_t direct_CAN_Transmission)
{
    if(SensorList[SensorListPosition].number_of_connected_sensors==0)
    {
        return;
    }

    if(SensorList[SensorListPosition].Sensorfunction!=NULL)
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
 * This function is for debugging 
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
        Write_16bit_to_23LC1024_Buffer(OutputData,false);
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
uint16_t StoreEncoder1(SensorClass Sensor,uint8_t direct_CAN_Transmission)
{
    uint16_t OutputData=CopyTimerToEncoderValue(&TMR1,direct_CAN_Transmission,Encoder_Meta_Data);
   
    return OutputData;
}
uint16_t StoreEncoder2(SensorClass Sensor,uint8_t direct_CAN_Transmission)
{
    uint16_t OutputData=CopyTimerToEncoderValue(&TMR3,direct_CAN_Transmission,Encoder2_Meta_Data);

    return OutputData;
}

void transmit_monitoring_summary(void)
{
    uint32_t total_time = Timers.OperatingTime_ms-MonitoringModeStartTime;
    //show the user how many samples were taken
    uint32_t total_samples =  getCurrent23LC1024Address();

    convertHexToDecAndTransmit('S',total_samples/2,true);        

    //show user how long it took
    total_time/=ProgramTimerDivider;
    convertHexToDecAndTransmit('T',total_time,true);
   
}
void convertSensorFrequencyToClockMultiple(void)
{
    #define CONVERT_MICRO_TO_SECONDS (1000000UL)
    uint32_t Rate = (uint32_t)((CONVERT_MICRO_TO_SECONDS/Sensor_Checking_Interval)/TIMER0_INTERRUPT_TIME_us);
   
    Sensor_Checking_Interval = (uint16_t)Rate;
}

void setup_monitoring_mode(uint8_t Mode)
{
    //configure the accelerometer

    MonitoringModeActive = Mode;

    //turn monitoring on
    if(MonitoringModeActive == 1)
    {
        Sensor_Checking_Interval    =  check_current_value_of_16bit_OD_entry(SENSOR_LOGGING_RATE_CAN_INDEX,false);
        
        convertSensorFrequencyToClockMultiple();

        setupSensorListCODParameters();
        MonitoringModeStartTime = Timers.OperatingTime_ms;
    }
    //turn monitoring off
    else
    {
        transmit_monitoring_summary();
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

    convertHexToDecAndTransmit(sensor_identifier,(uint16_t)newdata, true);    
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
        Write_16bit_to_23LC1024_Buffer(FinalOutputData,false); 
        return;
    }
    //or directly transmitted on the can bus
    //we don't care about efficiency from here on
    
    
    if(direct_CAN_Transmission==CAN_OUTPUT_MODE_1)          //Transmit raw with Metadata as seperate bytes
    {
        Transmit_Sensor_Data(FinalOutputData,0,0,0,0x00);
    }
    else if(direct_CAN_Transmission==CAN_OUTPUT_MODE_2)     //Transmit in Decimal without metadata
    {
        transmit_readable_sensor_data(FinalOutputData, 'A',  Sensor_Type);
    } 
}
void store_CAN_MSG(uint8_t msg_marker, uint8_t data_value)
{
	uint8_t First_EEPROM_byte =CAN_MSG_Meta_Data + msg_marker;
	
	uint16_t EEPROM_DUMP = (uint16_t)(First_EEPROM_byte<<8) + data_value;
	
    Write_16bit_to_23LC1024_Buffer(EEPROM_DUMP,false);

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


void TransmitRawSensorData(uint8_t *TX_Data)
{
    spi_comms_setup(DEFAULT_SPI_SETTINGS);
    
    CopyNodeIDToCANController(0x00,0x180, CAN_TXB1CTRL_REG,false);

    Fill_TX_REG_of_CAN_CONTROLLER(CAN_TXB1CTRL_REG+5,9,8,TX_Data[0],TX_Data[1],TX_Data[2],TX_Data[3],TX_Data[4],TX_Data[5],TX_Data[6],TX_Data[7]);

    transmit_CAN_Message(CAN_RTS_TXB1);

    SSPCON1bits.SSPEN=0;
}
void transferEntireRam(uint8_t Data[8], uint8_t *DLC, uint16_t *NodeID)
{
    uint24_t Address = 0;
    while(1)
    {
        uint8_t *READ_Data = Read_23LC1024(Address,8); 

        TransmitRawSensorData(READ_Data);
        
        if(READ_Data[0] == 0xff)
        {
            break;
        }
        Address+=8;
    } 
    
    return ;
}



