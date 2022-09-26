#include "ObjectDictionary/CAN Object Dictionary.h"
#include "ObjectDictionary.h"

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
        
        ///orientation rate of accelerometer (gravitational offset)

            #define ACCL_ORIENTATION_FLAG_DEFAULT 0
            #define ACCL_ORIENTATION_FLAG_MEM_USAGE 1
            #define ACCL_ORIENTATION_FLAG_MAX 1
            #define ACCL_ORIENTATION_FLAG_MIN 0 
            default_value[0] = ACCL_ORIENTATION_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,ACCL_ORIENTATION_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,ACCL_ORIENTATION_FLAG_MEM_USAGE,ACCL_ORIENTATION_FLAG_MAX,ACCL_ORIENTATION_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);                 

 /**
     * STEP MOTOR
     **/
		///sensor transfer enable flag
            #define STEP_MOTOR_FLAG_DEFAULT 0
            #define STEP_MOTOR_FLAG_MEM_USAGE 1
            #define STEP_MOTOR_FLAG_MAX 1
            #define STEP_MOTOR_FLAG_MIN 0 
            default_value[0] = STEP_MOTOR_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,STEP_MOTOR_TRANSFER_FLAG_INDEX,&total_mem_use, default_value,STEP_MOTOR_FLAG_MEM_USAGE,STEP_MOTOR_FLAG_MAX,STEP_MOTOR_FLAG_MIN,Factory_Reset_Enabled,&OBJECT_DATA);

            #define GENERIC_ON_OFF_FLAG_DEFAULT 0
            #define GENERIC_ON_OFF_FLAG_MEM_USAGE   1
            #define GENERIC_ON_OFF_FLAG_MAX 1
            #define GENERIC_ON_OFF_FLAG_MIN 0 

  /***
     * logging rate of sensor data
     ***/

            default_value[0] = (Sampling_period_TMR_Interrupts)&0xff; 
            default_value[1] = (Sampling_period_TMR_Interrupts>>8)&0xff; 
            CHECK_COD_ENTRY(CAN_INDEX,SENSOR_LOGGING_RATE_CAN_INDEX,&total_mem_use, default_value,2,65535,1000,Factory_Reset_Enabled,&OBJECT_DATA);

    /****
     *Factory Reset
     */

            default_value[0] = GENERIC_ON_OFF_FLAG_DEFAULT;
            CHECK_COD_ENTRY(CAN_INDEX,FACTORY_RESET_CAN_INDEX,&total_mem_use, default_value,1,1,0x00,Factory_Reset_Enabled,&OBJECT_DATA);
 
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

            #define MAXON_TEST_MEM_USAGE 1
            #define MAXON_TEST_DEFAULT_VALUE MOTOR_TEST_DISABLED
            #define MAXON_TEST_MAX_VALUE 1
            #define MAXON_TEST_MIN_VALUE 0

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
     Motor Encoder Enabled
     */
            default_value[0] = 1; 
            CHECK_COD_ENTRY(CAN_INDEX,MOTOR_ENCODER_ENABLED_CAN_INDEX,&total_mem_use, default_value,1,2,0,Factory_Reset_Enabled,&OBJECT_DATA);
            
     /*
      baud rate
     */
            default_value[1] = 0; 
            default_value[0] = 0x7d; 
            CHECK_COD_ENTRY(CAN_INDEX,CAN_BAUD_CAN_INDEX,&total_mem_use, default_value,2,500,125,Factory_Reset_Enabled,&OBJECT_DATA);
  
    /***
     * THIS MUST BE LAST!!!!!!!!!!!!!!!! 
     * total EEPROM Bytes USED
     ***/
            default_value[0] = total_mem_use + 1 ;
            CHECK_COD_ENTRY(CAN_INDEX,TOTAL_EEPROM_MEM_USAGE_INDEX,&total_mem_use, default_value,1,255,0x00,Factory_Reset_Enabled,&OBJECT_DATA);
    
            
    return OBJECT_DATA;
}