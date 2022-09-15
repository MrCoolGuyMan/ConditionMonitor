/* 
 * File:   CustomDataTypes.h
 * Author: Administrator
 *
 * Created on 2 December, 2021, 4:10 PM
 */

#ifndef CUSTOMDATATYPES_H
#define	CUSTOMDATATYPES_H
#define bool uint8_t
#define True 1
#define true 1
#define False 0
#define false 0
typedef uint8_t tristate;
typedef uint24_t CAN_INDEX_TYPE ;
typedef uint8_t ERROR_CODE;
    /*
     * for holding a single message service-data-object 
     */
        struct SDO_TX_CAN{
           uint16_t Message_ID;
           uint8_t MessageDLC;      //data length code
           uint8_t CCD;
           uint8_t Index_low;
           uint8_t Index_high;
           uint8_t Index_sub;
           uint8_t Data_bytes[4];
        };
    
    /*
     * for holding a multi-message service-data-object 
     */
        struct MULTI_READ_SDO{
            uint16_t Message_ID;
            uint8_t MessageDLC;     //each message is always 8 bytes
            uint8_t valid_length;   //but not all bytes are to be read
            uint8_t Data_bytes[7];
        };
    /*
     * flags and timers
     */
        typedef struct
        {
            bool CAN_MonitoringModeActive;
            tristate MotorTestEnabled;      //1 = Active, 0 = Halting, 2 = Inactive
            bool MonitoringModeActive;
        }ProgramFlags;
        
        typedef struct
        {
            uint16_t SensorFlag;            //counts milliseconds since last sensor reading
            uint16_t HeartBeat_FLAG;        //counts milliseconds since last heartbeat message
            uint24_t OperatingTime_ms;      //counts milliseconds since power on
            uint16_t EncoderWriteFlag;      //counts milliseconds since last encoder value was transmitted
            uint16_t MotorAccelerateflag;    //counts milliseconds since motor acceleration was last altered

        }TimingFlags;


    //L6202 chip requires 2 direction pins and a PWM pin
        typedef struct 
        {
            //both direction pins must be connected to the same port
            volatile uint8_t *directionPinLatch;    //Points to a PORT's Latch register
            uint8_t direction1PinNumber;            //these represent the pin numbers on the above latch

            volatile uint8_t *enablePinLatch;       //Points to a PORT's Latch register
            uint8_t enablePinNumber;

        }MotorConnections;

        typedef struct 
        {
            uint8_t MaxDutycyle;                //0-255
            uint8_t CurrentDutyCycle;           //0-255
            uint8_t Direction;                  //0-1 clockwise or counter clockwise
            uint16_t StatusUpdateRate;          //how often sensor data is transmitted during DC motor tests
            bool encoderConnected;              //is encoder attached?
        }Motor_Parameters;
        
        typedef struct
        {
                                                //used during DC Motor Test to record sensor ranges
            uint24_t Min_Encoder_Reading;
            uint24_t Max_Encoder_Reading;
            uint24_t Min_Current_Reading;
            uint24_t Max_Current_Reading;

        }MotorRecords;
        
        typedef struct
        {
            Motor_Parameters Settings;
            MotorConnections Connections;
            MotorRecords Records;

        }MotorClass;
    //CAN-Object Dictionary
        struct C_OD_ENTRY
        {
            uint32_t max_value;
            uint32_t min_value; 
            uint8_t default_value[10];
            uint8_t read_index;
            uint8_t Start_Address;
            uint8_t length;
       };

    //for sensors that are constantly read during the Monitoring mode operation
        typedef struct SensorType
        {
            uint8_t Sensor_Type;
            uint8_t number_of_connected_sensors;
            uint8_t Starting_Sensor_Number;
            uint8_t SensorReadings;
            CAN_INDEX_TYPE number_of_connected_sensors_index;
            uint16_t (*Sensorfunction)(struct SensorType,uint8_t);  //this is called a function pointer
                                                                     //the function is the one that will read the sensor value during the monitoring process
                                                                     //it needs to accept an EEPROM address for storing the sensor data into EEPROM
        }SensorClass;
        
        typedef union
        {
            uint24_t _24_bit_Address;

            struct {
                uint8_t Addr_3;  
                uint8_t Addr_2;    
                uint8_t Addr_1; 
            }bytes;

        }MemoryAddress;
    //EEPROM chip takes 6 ms to complete a write, the write size can be 1 - 256 bytes
    //so we will write the max number of bytes at once for max sampling rate
    //we need to balance this against the time it takes to dump a large amount of data at once
    //until then we store the data in a software buffer
    //the buffer will be dumped whenever currentSize rolls over, or when monitoring mode is turned off
        #define BUFFER_TRANSFER_SIZE 256
         typedef struct 
         {
             uint8_t StorageArray[BUFFER_TRANSFER_SIZE];
             uint8_t currentSize;
             MemoryAddress CurrentAddress;
         }LOG_DATA_BUFFER;


#endif	/* CUSTOMDATATYPES_H */

