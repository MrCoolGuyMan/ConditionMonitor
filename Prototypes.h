/* 
 * File:   Prototypes.h
 * Author: Administrator
 *
 * Created on 3 December, 2021, 10:11 AM
 */

#ifndef PROTOTYPES_H
#define	PROTOTYPES_H
#define private   
#define public
/*
 *  General Process Control
 */
private void CheckFlags(void);
private void setup_IO_Ports(void);
void ToggleLatch(volatile uint8_t *LatchReg, uint8_t PinNum);
//Setup timer 0 for executing tasks at multiples of 5ms; 
//timer1 is setup for reading motor encoder
//INT2 pin is used for CAN Controller RX interrupt
void Interrupt_setup(void);

//Set default values into internal EEPROM
//reset external eeprom chip
void factory_reset(void);

//Set "PinNum" bit of "LatchReg" to 1
/*
 * Example          volatile  uint8_t *ptr = &LATA;
 *                  TurnLatchOn(ptr,3);                
 * equivalent to 
 *                  LATAbits.LA3 = 1;
 */
void TurnLatchOn(volatile uint8_t *LatchReg, uint8_t PinNum);

//Set "PinNum" bit of "LatchReg" to 0
/*
 * Example          volatile  uint8_t *ptr = &LATA; 
 *                  TurnLatchOn(ptr,3);                
 * equivalent to 
 *                  LATAbits.LA3 = 0;
 */
void TurnLatchOff(volatile uint8_t *LatchReg, uint8_t PinNum);


/*
 * Sensors
 */    
void READ_FROM_ADC_QUICK(uint8_t adc_channel,uint8_t Sensor_type, uint8_t Starting_Sensor_ID);

//continually poll sensors
void Begin_sensor_monitoring(void);

void transmit_monitoring_summary(void);
//set "Sensor montoring" to false
void Turn_off_Sensor_Monitoring(void);

//send TX_Data, along with associated eeprom address onto CAN Bus
// the eeprom address is to match the message up with the read request
void Transmit_Sensor_Data(uint16_t TX_Data, uint8_t Addr4, uint8_t Addr3, uint8_t Addr2, uint8_t Addr1);

//transmit a value is binary coded decimal
//i.e. 0xA = 0d10 is transmitted as 0x:00 00 00 10 
void convertHexToDecAndTransmit(uint8_t identifier,uint32_t InputValue, bool TransmitOnCompletion);
///Accelerometer
void setup_ACCL(void);
uint16_t Read_ACCL_Sensor(SensorClass Sensor,uint8_t direct_CAN_Transmission );
void Format_ACCL_Data(uint16_t axis_accl_data, uint8_t direct_CAN_Transmission,uint8_t Sensor_ID_axis);

uint16_t ReadADCSensor(SensorClass Sensor,uint8_t direct_CAN_Transmission);
//sensor class functions
//populate SensorClass Array
void initSensorList(void);

//reads values from COD that could change between tests
void setupSensorListCODParameters(void);

//checks if a sensor has at least one connection, then call function pointed to by SensoClass function pointer
void SensorHeader(uint8_t SensorListPosition,uint8_t direct_CAN_Transmission);
uint16_t TwosComplement(uint16_t Data);
//copy current encoder reading to memory
uint16_t storeEncoderData(SensorClass Sensor,uint8_t direct_CAN_Transmission);
/*   
 * Format sensor data for storage in external EEPROM chip
* 6 bits of meta data contain the sensor type and sensor id
* 4 types; 2 bits
* 00 = temperature
* 01 = current
* 02 = acceleration - (Sensor ID - 01=x 02=y 03=z)
* 
* there are max 10 sensors of each type (4 bits)
* 
* DATA FIELD   |  SENSOR_TYPE  SENSOR_ID   SENSOR_DATA
* DATA BITS    |   <15:14>     <13:10>     <9:0>
* 
*/
void FormatSensorMetaData(uint8_t direct_CAN_Transmission, uint8_t Data_MostSig2bits,uint8_t Data_LSB, uint8_t Sensor_Type, uint8_t sensor_id_with_offset); 

void setup_monitoring_mode(uint8_t Mode);
/*
 *  Analog to Digital Converter 
 */

void setup_ADC(void);
//Convert 10-bit ADC value into mA
void ConvertADCToCurrent(int32_t *newdata);
//convert from ADC to human readable value
void transmit_readable_sensor_data(uint32_t OutputData, char sensor_identifier, uint8_t Sensor_Type);



/*
 * CAN BUS Communication
 */

//load RX buffer from CAN controller and parse out
void Read_CAN_Message(uint8_t Starting_Register);

//find out which RX buffer has a new message stored
void check_RX(void);

/*
 * Service Data Object Protocol 
 */
void send_SDO_Response(struct SDO_TX_CAN Msg);

//determine if an SDO message is for single/multi read or for a write operation and pass data into appropriate functions
void Deal_With_SDO(uint8_t Data[8]);

//write specified value into specified CAN-OD index
void HANDLE_SDO_WRITE_REQUEST(struct SDO_TX_CAN *Response,struct C_OD_ENTRY Object_Data,uint8_t Msg[8]);

//read specified value from specified CAN-OD index
void HANDLE_SDO_READ_REQUEST(struct SDO_TX_CAN *Response,struct C_OD_ENTRY Object_Data);

//read specified value from specified CAN-OD index transferred over multiple 8-byte messages
void deal_with_multi_read_SDO(uint8_t Data);

//populate MULTI_READ_SDO struct for current SDO message
void initialise_multi_read_response(struct MULTI_READ_SDO *Response,uint8_t Data);

//after write request, check if any function needs to be processed
void Process_SDO_REQUEST(uint24_t Index,uint8_t Data[8]);

/*
 * Internal EEPROM
 * 
 * 256 bytes of eeprom, used to store CAN object Dictionary
 */

//read single byte of eeprom data
uint8_t Read_From_INTERNAL_EEPROM(uint8_t Address);

//write upto 8 bytes of data to eeprom
void Write_Data_to_INTERNAL_EEPROM(uint8_t Address, uint8_t Data[8], uint8_t Total_Bytes);


/*
 * CAN-Object Dictionary
 */
//all values are stored as 32 bit unsigned integers
//bounds check will ensure that the value is inside the range min<= x <= max (otherwise it is set to default)
uint32_t check_current_value_of_32bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds);

//identical to above function but includes a 16-bit mask (0xffff)
uint16_t check_current_value_of_16bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds);

//identical to above function but includes an 8-bit mask (0xff)
uint8_t  check_current_value_of_8bit_OD_entry(CAN_INDEX_TYPE CAN_INDEX, bool check_bounds);

//sets up a CAN-Object by copying it into internal EEPROM
void setup_COD_data(uint8_t Mem_Address, uint8_t default_value[],uint8_t mem_usage,uint32_t max_value,
                    uint32_t min_value,struct C_OD_ENTRY *OBJECT_DATA );

///populate the C_OD_ENTRY struct from EEPROM; 
///keep track of current EEPROM address
///write default values to EEPROM during factory reset
void CHECK_COD_ENTRY(CAN_INDEX_TYPE CAN_INDEX,CAN_INDEX_TYPE REFERENCE_CAN_INDEX,uint8_t *Mem_Address, 
                    uint8_t default_value[],uint8_t mem_usage,uint32_t max_value,uint32_t min_value,
                    bool Factory_Reset_Enabled,struct C_OD_ENTRY *OBJECT_DATA);

///converts a can object dictionary entry index to an EEPROM memory address (START address inside of C_OD_ENTRY struct)
//returns a struct containing the data inside that directory
struct C_OD_ENTRY FIND_CAN_OBJECT(CAN_INDEX_TYPE CAN_INDEX, bool Factory_Reset_Enabled );

//change COD entry value using can index
ERROR_CODE Edit_COD(CAN_INDEX_TYPE CAN_INDEX,uint8_t Data_to_Write[8], uint8_t data_length);

//Copies DATA_TO_WRITE into CAN-Object Dictionary using eeprom address
ERROR_CODE Write_to_COD(struct C_OD_ENTRY Object_Data, uint8_t Data_to_Write[8], uint8_t data_length);

/**
 * 6 bits of meta data contain the sensor type and sensor id
 * sensor type specifier; 3 bits range [0 - 8]
 * Currently Allocated
 * 00 = temperature
 * 01 = current
 * 02 = acceleration - (Sensor ID - 01=x 02=y 03=z)
 * 03 = CAN Log
 * there are max 8 sensors of each type (3 bits)
 * 
 * There are a max of 10 bits of data reserved for the sensor
 * 
 * DATA FIELD   |  SENSOR_TYPE  SENSOR_ID   SENSOR_DATA
 * DATA BITS    |   <15:13>     <12:10>     <9:0>
 * 
 **/

//Increments the EEPROM Address Pointer by 2 bytes, turns off sensor monitoring after final address
void Increment_Ext_EEPROM_ADDRESS(void);

//Reset the External EEPROM's Memory
void Reset_EEPROM(void);

//Send Write Enable Command to EEPROM
void WREN_Ext_EERPOM(void);

//wait for WriteInProgress bit to clear
void waitForEEPROMReady(void);

//Read 2 bytes from External EEPROM
uint16_t Read_From_Ext_EEPROM(MemoryAddress Address);

//copy EEPROM_BUFFER to external chip
void transfer_EEPROM_BUFFER(void);

//initialize memory
void ZeroStorageBuffer(void);
//Write 2 bytes from External EEPROM (Buffer)
void Write_To_Ext_EEPROM(uint16_t Data_MSB);//, uint8_t Data_LSB);
void start_transfer(void);
//write less than 256 bytes to eeprom
void transfer_partial_page(void);
//write 256 bytes to eeprom, optimised for speed
void transfer_Entire_Page(void);
//stores a full CAN message into the external eeprom
void log_CAN_MESSAGE_TO_EEPROM( uint8_t Data[8],uint8_t DLC, uint16_t Node_ID);

//the PC will send an eeprom address to read, the MCU will return the data at that address
void handle_Ext_EEPROM_READ_REQUEST(MemoryAddress EEPROM_Address_Read_Request);
/*
 * SPI Bus
 */
//Setup SPI; CKE and CKP are configurable per device
void spi_comms_setup(uint8_t CKE,uint8_t CKP,uint8_t SSPM);

//reads 1 byte from SPI bus, implemented as SPI_WRITE(0xff) for convenience
char READ_SPI(void);

//Writes 1 byte to SPI bus
void SPI_WRITE(unsigned char msg);

//Writes variable bytes to SPI Bus
void SPI_MULTI_WRITE(uint8_t num_msgs, ...);

/*
 * Motor Control - brushed DC using L6202 IC
 * 
 * interfaces uses 2 pins to specify direction and one PWM
 */
//comparator 1 is used for overcurrent protection
void disableCM1(void);

void enableCM1(void);

//returns struct containing default DC Motor Settings
Motor_Parameters InitMotorParameters(void);

//Accelerates or decelerates a dc motor to either the min or max duty cycles
uint8_t AccelerateMotor(uint8_t *motor_duty_cycle, uint8_t EndDutyCycle);

//checks the motor status, decides whether to accelerate/decelerate/stop/ignore
void checkMotor(void);

//transmits the encoder reading on the CAN bus in hex-coded decimal
void SendEncoderData(void);

//brakes the motor, writes 255 to DC Motor Test CAN INDEX
void turnoffMotorControl(void);

//Enables motor and sets motor enabled flag
void startMotor(void);

//setup Timer 1 for PWM output to control motor
void setupPWM(uint8_t startDutyCycle, uint8_t PR_Counter);

//record the min and max encoder readings
void compare_minmax(uint24_t value, uint24_t *Min, uint24_t *Max);
/*
 * MCP2515 CAN CONTROLLER
 */

void write_command_to_CAN_CONTROLLER(unsigned char Message_type,uint8_t Address,uint8_t data, uint8_t final_CS_Value);

//send repeated read command to MCP2515
void Repeated_Read_from_CAN_CONTROLLER(uint8_t Start_address);

//read data at specified register address
unsigned char read_from_CAN_CONTROLLER(uint8_t Address);

uint8_t Reset_Interrupt_flags_On_CAN_CONTROLLER(void);

//set configuration registers of MCP2515
void Reset_CAN_CONTROLLER(void);

//setup TX Buffer 0 to send heartbeat command Node = NodeID + 0x700; message = 05
void setup_heart_beat_command(void);

//send request to send for TX buffer 0; sets up SPI
void send_heartbeat_message(void);

//send request to send command for specified TX buffer
void transmit_CAN_Message(uint8_t RTS_REGISTER);

//load specified TX buffer with variable message bytes
void Fill_TX_REG_of_CAN_CONTROLLER(uint8_t starting_REG, uint8_t num_msgs, ...);

//offset node id with NODE_ID_OFFSET and parse into upper/lower bytes 
//if find_Node_Id is true, ID is read from EEPROM, else Node_ID value is used
void CopyNodeIDToCANController(uint8_t Node_ID,uint16_t Node_ID_offset, uint8_t CAN_TX_REG, bool find_Node_Id);
void Parse_Node_ID(uint8_t Node_ID,uint16_t Node_ID_offset, uint8_t *Node_ID_H, uint8_t *Node_ID_L);


#endif	/* PROTOTYPES_H */

