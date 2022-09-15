/* 
 * File:   CAN Object Dictionary.h
 * Author: Administrator
 *
 * Created on 18 January, 2021, 2:14 PM
 */

#ifndef CAN_OBJECT_DICTIONARY_H
#define	CAN_OBJECT_DICTIONARY_H

#define DEVICE_TYPE                     0x100000 
#define DEVICE_NAME_INDEX               0x100800
#define HARDWARE_VERSION_INDEX          0x100900
#define VERSION_NUMBER_INDEX            0x100A00
#define FACTORY_RESET_CAN_INDEX         0x101101 
#define PRODUCT_CODE_INDEX              0x101802 
#define SERIAL_NUMBER_INDEX             0x101804
#define PROGRAM_CONTROL_NUM_ENTRIES     0x1F5100
#define PROGRAM_CONTROL_CAN_INDEX       0x1F5101
//sensors
#define SENSOR_NOT_IN_OBJECT_DICTIONARY 0xFFFFFF    //used for sensors not needing to be logged in EEPROM
#define TEMP_SENS_TRANSFER_FLAG_INDEX   0x200000
#define TEMP_AMNT_TRANSFER_FLAG_INDEX   0x200001   
#define NUM_ADCS_TRANSFER_FLAG_INDEX    0x200002 
#define CRNT_SENS_TRANSFER_FLAG_INDEX   0x200100
#define CRNT_AMNT_TRANSFER_FLAG_INDEX   0x200101   
#define CRNT_OFFSET_VOLTAGE_FLAG_INDEX  0x200103 
#define ACCL_SENS_TRANSFER_FLAG_INDEX   0x200400
#define ACCL_AMNT_TRANSFER_FLAG_INDEX   0x200401
#define ACCL_SAMPLE_TRANSFER_FLAG_INDEX 0x200402
//not used yet
#define HALL_EFFECT_TRANSER_FLAG_INDEX  0x200500
#define HALL_EFFECT_AMNT_FLAG_INDEX     0x200501
#define HALL_EFFECT_ADCS_FLAG_INDEX     0x200502 

//Operating Modes TRANSFER_SENSOR_DATA_FROM_EEP_INDEX
#define ENTER_SENSOR_MONITORING_MODE    0x200300
#define SENSOR_MONITORING_OUTPUT_MODE   0x200301
#define ENTER_CAN_BUS_MONITORING_MODE   0x200500
#define SENSOR_LOGGING_RATE_CAN_INDEX   0x201000
//eeprom
#define TRANSFER_SENSOR_DATA_FROM_EEP   0x200600
#define TRANSFER_BYTE_DATA_FROM_EEP     0x200601
#define TOTAL_EEPROM_MEM_USAGE_INDEX    0x200602
#define CLEAR_EXTERNAL_EEPROM_INDEX     0x200603  

#define NODE_ID_CAN_INDEX               0x2F0001


//tests
#define MAXON_GRIPPER_TEST_CAN_INDEX    0x300000
#define MOTOR_MAX_DUTY_CYCLE_CAN_INDEX  0x300001
#define MOTOR_DIRECTION_CAN_INDEX       0x300002
#define MOTOR_STATUS_CAN_INDEX          0x300003
#define MOTOR_ENCODER_ENABLED_CAN_INDEX 0x300004
//convenience macros

#define TRANSFER_SENSOR_DATA_FROM_EEP_UPPER         (TRANSFER_SENSOR_DATA_FROM_EEP&0xff0000)>>16
#define TRANSFER_SENSOR_DATA_FROM_EEP_LOWER         (TRANSFER_SENSOR_DATA_FROM_EEP&0xff00)>>8
#define TRANSFER_SENSOR_DATA_FROM_EEP_SUB           (TRANSFER_SENSOR_DATA_FROM_EEP&0xff)
#define TRANSFER_SENSOR_DATA_FROM_EEP_INDEX         TRANSFER_SENSOR_DATA_FROM_EEP_LOWER,TRANSFER_SENSOR_DATA_FROM_EEP_UPPER,TRANSFER_SENSOR_DATA_FROM_EEP_SUB
  
///populate the C_OD_ENTRY struct from EEPROM; 
///keep track of current EEPROM address
///write default values to EEPROM during factory reset
void  CHECK_COD_ENTRY(  CAN_INDEX_TYPE CAN_INDEX,CAN_INDEX_TYPE REFERENCE_CAN_INDEX,
                        uint8_t *Mem_Address, uint8_t default_value[],
                        uint8_t mem_usage,uint32_t max_value,uint32_t min_value,
                        bool Factory_Reset_Enabled,struct C_OD_ENTRY *OBJECT_DATA )
{
 
	if ((CAN_INDEX==REFERENCE_CAN_INDEX)||(Factory_Reset_Enabled==1))
	{
		setup_COD_data(*Mem_Address, default_value,mem_usage,max_value,min_value,OBJECT_DATA);
	}
    ///increments the EEPROM address
	*Mem_Address+=mem_usage;
	
	if(Factory_Reset_Enabled==1)///write default value to EEPROM
    {
        Write_Data_to_INTERNAL_EEPROM(OBJECT_DATA->Start_Address,OBJECT_DATA->default_value,OBJECT_DATA->length);
    }
	
	return ;
}
///populate C_OD_ENTRY struct values
void  setup_COD_data(uint8_t Mem_Address, uint8_t default_value[],uint8_t mem_usage,uint32_t max_value,uint32_t min_value,struct C_OD_ENTRY *OBJECT_DATA)
{
    OBJECT_DATA->Start_Address = Mem_Address;
    for(uint8_t i = mem_usage; i!=0; i--)   
    {
        OBJECT_DATA->default_value[i-1] = default_value[i-1] ;
    }
    OBJECT_DATA->length     =   mem_usage;
    OBJECT_DATA->max_value  =   max_value;
    OBJECT_DATA->min_value  =   min_value;
    OBJECT_DATA->read_index =   0;

    return ;
}
///converts a can object dictionary entry index to an EEPROM memory address
struct C_OD_ENTRY FIND_CAN_OBJECT(CAN_INDEX_TYPE CAN_INDEX,bool Factory_Reset_Enabled)
{
    struct C_OD_ENTRY OBJECT_DATA;
    OBJECT_DATA.length=0;               //indicates a failure

    uint8_t total_mem_use=0;            //256 bytes of memory 
    uint8_t default_value[10];
    uint8_t default_value_index = 0;


    /**
     * device type
     */

            default_value_index=0; 
            default_value[default_value_index] = '4';default_value_index++;
            default_value[default_value_index] = '5';default_value_index++;
            default_value[default_value_index] = '5';default_value_index++;
            default_value[default_value_index] = '0';default_value_index++;
            CHECK_COD_ENTRY(CAN_INDEX,DEVICE_TYPE,&total_mem_use, default_value,default_value_index,0xffffffff,0,Factory_Reset_Enabled,&OBJECT_DATA);

    /**
     * device type
     */

            default_value_index=0; 
            default_value[default_value_index] = 'M';default_value_index++;
            default_value[default_value_index] = 'O';default_value_index++;
            default_value[default_value_index] = 'N';default_value_index++;
            default_value[default_value_index] = 'I';default_value_index++;
            default_value[default_value_index] = 'T';default_value_index++;
            default_value[default_value_index] = 'O';default_value_index++;
            default_value[default_value_index] = 'R';default_value_index++;   
            CHECK_COD_ENTRY(CAN_INDEX,DEVICE_NAME_INDEX,&total_mem_use, default_value,default_value_index,0xffffffff,0,Factory_Reset_Enabled,&OBJECT_DATA);

    /**
     * Product Code
     */     

            default_value_index=0; 
            default_value[default_value_index] = 'P';default_value_index++;
            default_value[default_value_index] = 'P';default_value_index++;
            default_value[default_value_index] = '0';default_value_index++;
            default_value[default_value_index] = '1';default_value_index++;
            CHECK_COD_ENTRY(CAN_INDEX,PRODUCT_CODE_INDEX,&total_mem_use, default_value,default_value_index,0xffffffff,0,Factory_Reset_Enabled,&OBJECT_DATA);
        
     /**
     * serial number
     */   
            default_value_index=0; 
            default_value[default_value_index] = 0;
            CHECK_COD_ENTRY(CAN_INDEX,SERIAL_NUMBER_INDEX,&total_mem_use, default_value,1,0xff,0,Factory_Reset_Enabled,&OBJECT_DATA);

    /*
     Program Control
     *
            default_value_index=0; 
            default_value[default_value_index] = 0;
            CHECK_COD_ENTRY(CAN_INDEX,PROGRAM_CONTROL_NUM_ENTRIES,&total_mem_use, default_value,1,0xff,0,Factory_Reset_Enabled,&OBJECT_DATA);   
     */ 
    /**
     * Hardware version
     */     
            default_value_index=0; 
            default_value[default_value_index] = 'V';default_value_index++;
            default_value[default_value_index] = '1';default_value_index++;
            default_value[default_value_index] = '.';default_value_index++;
            default_value[default_value_index] = '0';default_value_index++;

            CHECK_COD_ENTRY(CAN_INDEX,HARDWARE_VERSION_INDEX,&total_mem_use, default_value,default_value_index,1,0,Factory_Reset_Enabled,&OBJECT_DATA);

     /**
     * Node ID
     **/
            #define NODE_ID_DEFAULT 0x7f
            #define NODE_ID_MEM_USAGE 1
            #define NODE_ID_MAX 0x7f
            #define NODE_ID_MIN 1
            default_value[0] = NODE_ID_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,NODE_ID_CAN_INDEX,&total_mem_use, default_value,NODE_ID_MEM_USAGE,NODE_ID_MAX,NODE_ID_MIN,Factory_Reset_Enabled,&OBJECT_DATA);      

    /**
     * Temperature Sensors
     **/
		///sensor transfer enable flag
            #define TEMP_SENS_FLAG_DEFAULT 0
            #define TEMP_SENS_FLAG_MEM_USAGE 1
            #define TEMP_SENS_FLAG_MAX 1
            #define TEMP_SENS_FLAG_MIN 0
            default_value[0] = TEMP_SENS_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,TEMP_SENS_TRANSFER_FLAG_INDEX,&total_mem_use,default_value,TEMP_SENS_FLAG_MEM_USAGE,TEMP_SENS_FLAG_MAX,TEMP_SENS_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);

		///number of temperature sensors

            #define TEMP_AMNT_FLAG_DEFAULT 0
            #define TEMP_AMNT_FLAG_MEM_USAGE 1
            #define TEMP_AMNT_FLAG_MAX 8
            #define TEMP_AMNT_FLAG_MIN 0
            default_value[0] = TEMP_AMNT_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,TEMP_AMNT_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,TEMP_AMNT_FLAG_MEM_USAGE,TEMP_AMNT_FLAG_MAX,TEMP_AMNT_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);

		///number of temperature sensor readings to average over

            #define TEMP_ADCS_FLAG_DEFAULT 20
            #define TEMP_ADCS_FLAG_MEM_USAGE 1
            #define TEMP_ADCS_FLAG_MAX 100
            #define TEMP_ADCS_FLAG_MIN 1
            default_value[0] = TEMP_ADCS_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,NUM_ADCS_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,TEMP_ADCS_FLAG_MEM_USAGE,TEMP_ADCS_FLAG_MAX,TEMP_ADCS_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);

    /**
     * Current Sensors
     **/
		///sensor transfer enable flag
            #define CRNT_SENS_FLAG_DEFAULT 0
            #define CRNT_SENS_FLAG_MEM_USAGE 1
            #define CRNT_SENS_FLAG_MAX 1
            #define CRNT_SENS_FLAG_MIN 0 
            default_value[0] = CRNT_SENS_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,CRNT_SENS_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,CRNT_SENS_FLAG_MEM_USAGE,CRNT_SENS_FLAG_MAX,CRNT_SENS_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);

		///number of sensors

            #define CRNT_AMNT_FLAG_DEFAULT 1
            #define CRNT_AMNT_FLAG_MEM_USAGE 1
            #define CRNT_AMNT_FLAG_MAX 8
            #define CRNT_AMNT_FLAG_MIN 0 
            default_value[0] = CRNT_AMNT_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,CRNT_AMNT_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,CRNT_AMNT_FLAG_MEM_USAGE,CRNT_AMNT_FLAG_MAX,CRNT_AMNT_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);
            
       //offset voltage for current sensor
            default_value[0] = (2500>>0)&0xff;
            default_value[1] = (2500>>8)&0xff;
            CHECK_COD_ENTRY(CAN_INDEX,CRNT_OFFSET_VOLTAGE_FLAG_INDEX,&total_mem_use, default_value,2,5000,0,Factory_Reset_Enabled,&OBJECT_DATA);
       
     /**
     * Accelerometer Sensors
     **/
		///sensor transfer enable flag
            #define ACCL_SENS_FLAG_DEFAULT 0
            #define ACCL_SENS_FLAG_MEM_USAGE 1
            #define ACCL_SENS_FLAG_MAX 1
            #define ACCL_SENS_FLAG_MIN 0 
            default_value[0] = ACCL_SENS_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,ACCL_SENS_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,ACCL_SENS_FLAG_MEM_USAGE,ACCL_SENS_FLAG_MAX,ACCL_SENS_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);

		///number of sensors

            #define ACCL_AMNT_FLAG_DEFAULT 1
            #define ACCL_AMNT_FLAG_MEM_USAGE 1
            #define ACCL_AMNT_FLAG_MAX 1
            #define ACCL_AMNT_FLAG_MIN 0 
            default_value[0] = ACCL_AMNT_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,ACCL_AMNT_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,ACCL_AMNT_FLAG_MEM_USAGE,ACCL_AMNT_FLAG_MAX,CRNT_AMNT_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);
         
        //sample rate of accelerometer
            #define ACCL_SAMPLE_FLAG_DEAFULT 15
            #define ACCL_SAMPLE_FLAG_MEM_USAGE 1
            #define ACCL_SAMPLE_FLAG_MAX 0x0f
            #define ACCL_SAMPLE_FLAG_MIN 1

            default_value[0] = ACCL_SAMPLE_FLAG_DEAFULT;
            CHECK_COD_ENTRY(CAN_INDEX,ACCL_SAMPLE_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,ACCL_SAMPLE_FLAG_MEM_USAGE,ACCL_SAMPLE_FLAG_MAX,ACCL_SAMPLE_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);
         
    /***
     * Transmit Sensor Data From EEPROM
     ***/

            #define GENERIC_ON_OFF_FLAG_DEFAULT 0
            #define GENERIC_ON_OFF_FLAG_MEM_USAGE   1
            #define GENERIC_ON_OFF_FLAG_MAX 1
            #define GENERIC_ON_OFF_FLAG_MIN 0 
            default_value[0] = GENERIC_ON_OFF_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,TRANSFER_SENSOR_DATA_FROM_EEP,&total_mem_use, default_value,GENERIC_ON_OFF_FLAG_MEM_USAGE,GENERIC_ON_OFF_FLAG_MAX,GENERIC_ON_OFF_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);

    /***
     * Constant reading of sensor data
     ***/
            default_value[0] = GENERIC_ON_OFF_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,ENTER_SENSOR_MONITORING_MODE,&total_mem_use, default_value,GENERIC_ON_OFF_FLAG_MEM_USAGE,GENERIC_ON_OFF_FLAG_MAX,GENERIC_ON_OFF_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);
        
            default_value[0] = EEPROM_OUTPUT_MODE;
            CHECK_COD_ENTRY(CAN_INDEX,SENSOR_MONITORING_OUTPUT_MODE,&total_mem_use, default_value,GENERIC_ON_OFF_FLAG_MEM_USAGE,2,GENERIC_ON_OFF_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);
    
     /***
     * logging rate of sensor data
     ***/
            //default_value[0] = Sensor_Checking_Interval;   //every 10 ms
            default_value[0] = (Sensor_Checking_Interval)&0xff; 
            default_value[1] = (Sensor_Checking_Interval>>8)&0xff; 
            CHECK_COD_ENTRY(CAN_INDEX,SENSOR_LOGGING_RATE_CAN_INDEX,&total_mem_use, default_value,2,65535,0,Factory_Reset_Enabled,&OBJECT_DATA);

     /***
     * Constant reading of CAN BUS data
     ***/
            default_value[0] = GENERIC_ON_OFF_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,ENTER_CAN_BUS_MONITORING_MODE,&total_mem_use, default_value,GENERIC_ON_OFF_FLAG_MEM_USAGE,GENERIC_ON_OFF_FLAG_MAX,GENERIC_ON_OFF_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);

    /****
     *Factory Reset
     */

            default_value[0] = GENERIC_ON_OFF_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,FACTORY_RESET_CAN_INDEX,&total_mem_use, default_value,1,1,0x00,Factory_Reset_Enabled,&OBJECT_DATA);
    /****
     *clear external eeprom
     */
            default_value[0] = GENERIC_ON_OFF_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,CLEAR_EXTERNAL_EEPROM_INDEX,&total_mem_use, default_value,1,1,0x00,Factory_Reset_Enabled,&OBJECT_DATA);
    /****
     *software version number
     */
            #define VERSION_NUMBER_MAX 0xffffffff
            #define VERSION_NUMBER_MIN 0
            default_value_index=0; 
            default_value[default_value_index] = 'V';default_value_index++;
            default_value[default_value_index] = '1';default_value_index++;
            default_value[default_value_index] = '.';default_value_index++;
            default_value[default_value_index] = '1';default_value_index++;
            default_value[default_value_index] = '0';default_value_index++;
            CHECK_COD_ENTRY(CAN_INDEX,VERSION_NUMBER_INDEX,&total_mem_use, default_value,default_value_index,VERSION_NUMBER_MAX,VERSION_NUMBER_MIN,Factory_Reset_Enabled,&OBJECT_DATA);
            
    /*
     * Active gripper motor test
     */
            #define MAXON_TEST_MEM_USAGE 1
            #define MAXON_TEST_DEFAULT_VALUE MOTOR_TEST_DISABLED
            #define MAXON_TEST_MAX_VALUE 1
            #define MAXON_TEST_MIN_VALUE 0
            default_value[0] = MAXON_TEST_DEFAULT_VALUE;
            CHECK_COD_ENTRY(CAN_INDEX,MAXON_GRIPPER_TEST_CAN_INDEX,&total_mem_use, default_value,MAXON_TEST_MEM_USAGE,MAXON_TEST_MAX_VALUE,MAXON_TEST_MIN_VALUE,Factory_Reset_Enabled,&OBJECT_DATA);
 
    /*
     Motor Max Duty cycle
     */
            default_value[0] = 255;
            CHECK_COD_ENTRY(CAN_INDEX,MOTOR_MAX_DUTY_CYCLE_CAN_INDEX,&total_mem_use, default_value,MAXON_TEST_MEM_USAGE,255,MAXON_TEST_MIN_VALUE,Factory_Reset_Enabled,&OBJECT_DATA);
 
     /*
     Motor Direction
     */
            default_value[0] = 0;
            CHECK_COD_ENTRY(CAN_INDEX,MOTOR_DIRECTION_CAN_INDEX,&total_mem_use, default_value,MAXON_TEST_MEM_USAGE,MAXON_TEST_MAX_VALUE,MAXON_TEST_MIN_VALUE,Factory_Reset_Enabled,&OBJECT_DATA);
     /*
     Motor Status Update Rate
     */
            default_value[0] = (Motor_Status_Update_Rate)&0xff; 
            default_value[1] = (Motor_Status_Update_Rate>>8)&0xff; 
            CHECK_COD_ENTRY(CAN_INDEX,MOTOR_STATUS_CAN_INDEX,&total_mem_use, default_value,2,65535,0,Factory_Reset_Enabled,&OBJECT_DATA);
     
    /*
     Motor Encoder Enabled
     */
            default_value[0] = 1; 
            CHECK_COD_ENTRY(CAN_INDEX,MOTOR_ENCODER_ENABLED_CAN_INDEX,&total_mem_use, default_value,1,2,0,Factory_Reset_Enabled,&OBJECT_DATA);
     
    /***
     * THIS MUST BE LAST!!!!!!!!!!!!!!!! 
     * total EEPROM Bytes USED
     ***/
            default_value[0] = total_mem_use + 1 ;
            CHECK_COD_ENTRY(CAN_INDEX,TOTAL_EEPROM_MEM_USAGE_INDEX,&total_mem_use, default_value,1,255,0x00,Factory_Reset_Enabled,&OBJECT_DATA);
    
            
    return OBJECT_DATA;
}
//Allows writing either a specified value "Data_To_Write" or a default value Data_To_Write=NULL
//using the CAN Address
//this is the one folks should use from external files
ERROR_CODE Edit_COD(CAN_INDEX_TYPE CAN_INDEX,uint8_t Data_to_Write[8], uint8_t data_length) 
{
    struct C_OD_ENTRY Object_Data=FIND_CAN_OBJECT(CAN_INDEX,0);
    
    uint8_t error_code = ERROR_CODE_OKIE_DOKES ;
    if(Data_to_Write == NULL)
    {
        Write_Data_to_INTERNAL_EEPROM(Object_Data.Start_Address,Object_Data.default_value,Object_Data.length);
    }
    else
    {
        error_code=Write_to_COD(Object_Data, Data_to_Write, data_length);
    }
    return error_code;    
}
//Implements the Write Request from above
//you can call this directly if you already have "Object_Data"
//if you want to write the default value you need to use Edit_COD
//or call function like this Write_to_Code (Object,Object.default_value,Object.length)
ERROR_CODE Write_to_COD(struct C_OD_ENTRY Object_Data, uint8_t Data_to_Write[8], uint8_t data_length)
{
    uint32_t Value_To_Write = 0;
    //little endian madness
    //01 00 03 02 -> 03 02 00 01
    
    uint8_t shift_vals[] = {0,8,24,16};
    for(uint8_t index=0; index< data_length; index++)
    {
        Value_To_Write +=(uint32_t) Data_to_Write[index]<<(shift_vals[index]);
    }
   /* Value_To_Write = Data_to_Write[0];
    Value_To_Write +=(uint32_t) Data_to_Write[1]<<8;
    Value_To_Write +=(uint32_t) Data_to_Write[2]<<24;
    Value_To_Write +=(uint32_t) Data_to_Write[3]<<16;*/
    
    if((Value_To_Write < Object_Data.min_value)
    || (Value_To_Write > Object_Data.max_value))
    {
        return ERROR_CODE_NOT_OKIE_DOKES; //return Error
    }

    Write_Data_to_INTERNAL_EEPROM(Object_Data.Start_Address,Data_to_Write,Object_Data.length);

    return ERROR_CODE_OKIE_DOKES;
}
/************************
read EEPROM address coded as Object-Dictionary address and return value
 * 
 * check_current_value_of_8bit_OD_entry exists to save having to mask with 0xff, as most parameters are 8-bit
************************/
uint8_t check_current_value_of_8bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds)
{
    return (0xff & check_current_value_of_32bit_OD_entry(CAN_INDEX,check_bounds));
}
uint16_t check_current_value_of_16bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds)
{
    return (0xffff & check_current_value_of_32bit_OD_entry(CAN_INDEX,check_bounds));
}
uint32_t check_current_value_of_32bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds)
{
    struct C_OD_ENTRY Object_Data=FIND_CAN_OBJECT(CAN_INDEX,0);
    if(Object_Data.length!=0)   //confirms that index exists
    {
        uint32_t Data_value=0;

        ///little endian nonsense
        for(uint8_t i=0; i <Object_Data.length && i<2; i++)
        {
            Data_value +=   (uint32_t)(Read_From_INTERNAL_EEPROM(Object_Data.Start_Address+i)<<(8*i));
        }
        for(uint8_t i=0; i <Object_Data.length-2 && i<2; i++)
        {
            Data_value +=   (uint32_t)(Read_From_INTERNAL_EEPROM(Object_Data.Start_Address+i+2)<<(8*(3-i)));
        }
        if(check_bounds==true)///check if min_value<Data_value<Max_value -> will find corrupted data; or fresh install 
        {
            if((Data_value>Object_Data.max_value)||(Data_value<Object_Data.min_value))
            {
                //overwrite invalid data with default value
                 Write_Data_to_INTERNAL_EEPROM(Object_Data.Start_Address,Object_Data.default_value,Object_Data.length);

                 Data_value=Read_From_INTERNAL_EEPROM(Object_Data.Start_Address);
            }  
        }
        return Data_value;
    }
    return ERROR_CODE_NOT_OKIE_DOKES;    //indicates a goof
}
#endif	/* CAN_OBJECT_DICTIONARY_H */

