/* 
 * File:   newmain.c
 * Author: Track
 *
 * Created on 23 November 2020, 2:00 PM
 */
#define GREEN_LED       LATCbits.LATC6
#define TOGGLE_GREEN    asm("BTG LATC, 6");

#define RED_LED         LATCbits.LATC2
#define RED_LED_ON      RED_LED=1;
#define RED_LED_OFF     RED_LED=0;
#define TOGGLE_RED   asm("BTG LATC, 2");

#include "ConditionMonitorConfigFile.h"
#include <stdint.h>                 //uint8_t etc.
#include "CustomDataTypes.h"
#include <xc.h>

#include "Global Variables.h"
#include "Prototypes.h"
#include "IO_Ports.h"
#include <stdarg.h>                 ///variable argument functions - > see can controller Fill_TX_REG_of_CAN_CONTROLLER          
#include "SPI_Interface.h"


#include "/ObjectDictionary/CAN Object Dictionary.h"
#include "/MCP2515 CAN Controller/MCP2515.h"
#include "/SDO/SDO Protocol.h"
#include "CANMessageDecoder.h"
#include "PDO/PDO Protocol.h"
#include "NMT/NMT Protocol.h"
#include "SDO Application.h"


#include "Sensors.h"

#include "DCMotorControl.h"
#include "StepperMotorControl.h"
#include "Interrupts.h"
#include "InternalEEPROM.h"
#include "/23LC1024 SPI RAM/_23LC1024RAM Definitions.h"
#include "/23LC1024 SPI RAM/_23LC1024 Driver.h"
#include "/ADXL345 Acc/ADXL345 Driver.h"
#include "/Humidty Sensor/BME280.h"
void setup_heart_beat_command(void);
void factory_reset(void);
#define DONT_RESPOND_TO_CAN_MESSAGE 0xff
void initClock(void)
{
        /*****
     OSCCON -page 33
	 set internal clock rate then wait for stabilization 
     ***/

        OSCCONbits.SCS = 0b10;  //internal
        OSCCON|=INTERNAL_OSC_SETTING;

        OSCCON2bits.INTSRC = 1;

        OSCCON2bits.PLLEN = 1;
        OSCTUNE = OSCTUNE_PLL_SETTING;

        while(OSCCONbits.HFIOFS==0)
        {

        }
}

void ReadSensorSingleShot(uint8_t Message[8],uint8_t *DLC, uint16_t *NodeIDWithFunctionCode)
{
    // data[0] is the read instruction
    // datr[1] is the sensor type
    setupSensorListCODParameters();
    SensorHeader(Message[1],CAN_OUTPUT_MODE_1);
    *DLC = 0;
    *NodeIDWithFunctionCode = DONT_RESPOND_TO_CAN_MESSAGE;
}
void PDO_ToggleMotor(uint8_t *Message,uint8_t *DLC, uint16_t *NodeIDWithFunctionCode)
{
    OperatingModeFlags.MotorTestEnabled = Message[1];
    
    *NodeIDWithFunctionCode -= 0x20;
    Message[7] = PDO_COMMAND_COMPLETE;
    RED_LED_OFF;
}
void PDO_ToggleSensorMonitoringMode(uint8_t *Message,uint8_t *DLC, uint16_t *NodeIDWithFunctionCode)
{
    setup_monitoring_mode(Message[1]);
    
    *DLC = 0;
    *NodeIDWithFunctionCode = DONT_RESPOND_TO_CAN_MESSAGE;
}

void setupCANCallbacks(void)
{
    // basic message decoding 
    SetCANCallback(&decodePDO,PDO_1_CALLBACK);
    
    SetCANCallback(&Deal_With_SDO,SDO_CALLBACK);
    
    SetCANCallback(&handleNMT,NMT_CALLBACK);
    
    
    // Process Data Object Protocol 
    SetPDOCallback(&transferEntireRam,              PDO_RAM_TRANSFER);
    SetPDOCallback(&Clear_23LC1024_From_PDO,        PDO_CLEAR_MEMORY);    
    SetPDOCallback(&PDO_ToggleSensorMonitoringMode, PDO_ENTER_SENSOR_MONITORING_MODE);
    SetPDOCallback(&ReadSensorSingleShot,           PDO_TRANSFER_SENSOR_READING);
    SetPDOCallback(&PDO_ToggleMotor,                PDO_TOGGLE_MOTOR);    
    
}

void initSensorList(void)
{
    AddSensor(TempSensor,       TEMPERATURE_SENSOR_START_PIN,   TEMP_AMNT_TRANSFER_FLAG_INDEX,      &ReadADCSensor,     &setup_ADC);   
    AddSensor(CurrentSensor,    CURRENT_SENSOR_START_PIN,       CRNT_AMNT_TRANSFER_FLAG_INDEX,      &ReadADCSensor,     &setup_ADC);   
    AddSensor(Encoder1,         0,                              MOTOR_ENCODER_ENABLED_CAN_INDEX,    &StoreEncoder1,     NULL); 
    AddSensor(Encoder2,         0,                              MOTOR_ENCODER_ENABLED_CAN_INDEX,    &StoreEncoder2,     NULL); 
    AddSensor(VibrationSensor,  0,                              ACCL_AMNT_TRANSFER_FLAG_INDEX,      &Read_ACCL_Sensor,  &setup_ACCL); 
    AddSensor(HumiditySensor,   0,                              HUMID_SENS_TRANSFER_FLAG_INDEX,     NULL,NULL); 
    return ;
}
void respondToCommand(CAN_RX_Buffer *Msg)
{
    if((Msg->MessageID == DONT_RESPOND_TO_CAN_MESSAGE)||(Msg->DLC == 0))
    {
        return;
    }
    CopyNodeIDToCANController(0x00,Msg->MessageID, CAN_TXB1CTRL_REG,false);
    
    Fill_TX_REG_of_CAN_CONTROLLER(  CAN_TXB1CTRL_REG+5,
                                    9,
                                    Msg->DLC,
                                    Msg->Data[0],
                                    Msg->Data[1],
                                    Msg->Data[2],
                                    Msg->Data[3],
                                    Msg->Data[4],
                                    Msg->Data[5],
                                    Msg->Data[6],
                                    Msg->Data[7]
                                );

    transmit_CAN_Message(CAN_RTS_TXB1);
}
void setup(void)
{
    // initialise the internal clock; the MCP2515 must be configured to enabled the CLK_OUT pin
    initClock();

	/****
	basic pin configuration
	*****/
	   setup_IO_Ports();
	   INTCON2bits.RBPU=1;
	   

       setup_ADC();
       Add_CS_LATCH(&LATA,4);   // 23LC1024 - RAM
       
	   spi_comms_setup(DEFAULT_SPI_SETTINGS);

	///check if the internal EEPROM is storing a node id already
		Assigned_Node_ID=check_current_value_of_8bit_OD_entry(NODE_ID_CAN_INDEX,false);
        
        //this will happen after a firmware update
        if(Assigned_Node_ID==0x00)
        {
            factory_reset();
            RESET();
        }
        
	/****
	CAN Controller configuration
	*****/
		
		Reset_CAN_CONTROLLER();   
    
        __delay_ms(1);
        init_CAN_CONTROLLER();
        
        setup_heart_beat_command();
        
        
    // change to external clock
    #if USE_INTERNAL_OSC == 0
        OSCCON2bits.PRISD= 1;
        OSCCONbits.SCS = 0b0;  //external clock
    #endif       
        init_23LC1024_RAM(_23LC1024_SEQ_MODE,0xff);
    //setup timer0,1 and INT2
        Interrupt_setup();
    
        initSensorList();
        
        setupCANCallbacks();
    /****
	 *if the logger was reset during monitoring mode, this will still be active
     *turn off monitoring and sensor transfer
     *************/
        turnoffMotorControl();
}
int main()
{
    setup();
    //infinite loop

    while(1)
    {
        if(checkMonitoringActive() == 1)
        {
            if(CheckIfSensorIntervalTimeHasArrived() == true) 
            {
                Timers.SensorFlag=0;
                #define REAL_TIME_MODE_TE
                #ifdef REAL_TIME_MODE_TEST
                
                Begin_sensor_monitoring(true);
                
                uint8_t data_len = _23LC1024getCurrentBufferSize();
                
                if(data_len >=8)
                {
                    uint8_t *TX_Data = _23LC1024GrabDataPointer();
                    for(uint8_t counter = 0 ; counter < data_len; counter+=8)
                    {
                        TransmitRawSensorData(&TX_Data[counter]);
                    }
                    Clear_23LC1024_Buffer();
                }
                #else
                Begin_sensor_monitoring(false);
                #endif
            }
        }
        else
        {
            ///send heartbeat every 500 ms 
            if(Timers.HeartBeat_FLAG>=Heart_Beat_TMR_Interrupts)
            {
                Timers.HeartBeat_FLAG=0;
                send_heartbeat_message();
                TOGGLE_GREEN;
            }
            if(OperatingModeFlags.MotorTestEnabled  != MOTOR_TEST_DISABLED)
            {
               checkMotor();
            }

        }
        //check if any messages are received
        while(CAN_INTERRUPT_PIN==0)///keep checking until all messages have been read
        {
           CAN_RX_Buffer NewMessage = check_RX();

            /*        if(OperatingModeFlags.CAN_MonitoringModeActive==1)
                    {
                        SET_CS_HIGH(SPI_deviceID);
                        //TODO
                      //  SSPEN = 0;
                      //  log_CAN_MESSAGE_TO_EEPROM (Data, DLC & 0b00001111,  Node_ID);

                    }*/           
           decodeMessage(NewMessage.Data,&NewMessage.DLC,&NewMessage.MessageID);

           respondToCommand(&NewMessage);
        }   
        

        if(OperatingModeFlags.MotorTestEnabled == MOTOR_TEST_EMERGENCY_HALT)
        {
            turnoffMotorControl();
        }
    }
        
    return (EXIT_SUCCESS);
}


void factory_reset(void)
{
    INTCONbits.GIE=0;///disable global interrupts
    FIND_CAN_OBJECT(0,1);

    INTCONbits.GIE=1;///enable global interrupts
}

//takes a value like 0x0FC3 (0d4035))
//and converts it to hex-coded decimal
//  0x00 0x00 0x40 0x35
void convertHexToDecAndTransmit(uint8_t identifier,uint32_t InputValue, bool TransmitOnCompletion)
{
    spi_comms_setup(DEFAULT_SPI_SETTINGS);

    CopyNodeIDToCANController(Assigned_Node_ID,0x00, CAN_TXB1CTRL_REG,true);
    
    uint32_t hex = 0;
    uint32_t i = 1;

    while(InputValue!=0)
    { 
        uint32_t digit = InputValue%10;
        InputValue=InputValue/10;
        hex+=digit*i;
        i*=16;
    }
    uint8_t MSG_DLC = 5;
    uint8_t first  = ((hex &0xff000000 )>>24);
    uint8_t second = ((hex &0xff0000   )>>16);
    uint8_t third  = ((hex &0xff00     )>>8);
    uint8_t fourth =  (hex &0xff       );

    Fill_TX_REG_of_CAN_CONTROLLER(CAN_TXB1CTRL_REG+5,MSG_DLC+1,MSG_DLC,identifier,first,second,third,fourth);

    if(TransmitOnCompletion == true)
    {
        transmit_CAN_Message(CAN_RTS_TXB1);
    }
    SSPEN = 0;
}
void compare_minmax(uint24_t value, uint24_t *Min, uint24_t *Max)
{
    if(*Min > value)
    {
        *Min = value;
    }
    if(*Max < value)
    {
        *Max = value;
    }
}
///send out message "0x05" with ID = " 0x700 + Node ID"
///stores the values in TX0
void setup_heart_beat_command(void)
{

    /**********
     * permanently stores heart beat data inside of the TXB0 register     
     *****************/ 
    uint8_t HeartBeat_DLC=0b00000001;///set RTR =0; 1 byte
    uint8_t HeartBeat_Msg=0b00000101;///5
    /*********
     * Extended ID uses 18 bits in 3 registers
     * TXBxSIDL 2LS bits are the MSB of Ext. ID
     * TXBnEID8 - High order
     * TXBnEID0 - Low order   
     * 
     * heart beat uses Node ID +0x700   
     ***********/
    CopyNodeIDToCANController(Assigned_Node_ID,NMT_NODE_MONITOR, CAN_TXB0CTRL_REG,false);
    
    Fill_TX_REG_of_CAN_CONTROLLER(CAN_TXB0CTRL_REG+5,2,HeartBeat_DLC,HeartBeat_Msg);
  
}