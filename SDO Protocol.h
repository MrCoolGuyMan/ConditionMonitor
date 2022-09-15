/* 
 * File:   SDO Protocol.h
 * Author: Administrator
 *
 * Created on 7 December, 2021, 11:14 AM
 */

#ifndef SDO_PROTOCOL_H
#define	SDO_PROTOCOL_H

#include "CustomDataTypes.h"



/****************************
respond to service data object message
****************************/
void send_SDO_Response(struct SDO_TX_CAN Msg)
{
    CopyNodeIDToCANController(0x00,Msg.Message_ID, CAN_TXB1CTRL_REG,false);
    
    Fill_TX_REG_of_CAN_CONTROLLER(  CAN_TXB1CTRL_REG+5,
                                    9,
                                    Msg.MessageDLC,
                                    Msg.CCD,
                                    Msg.Index_low,
                                    Msg.Index_high,
                                    Msg.Index_sub,
                                    Msg.Data_bytes[0],
                                    Msg.Data_bytes[1],
                                    Msg.Data_bytes[2],
                                    Msg.Data_bytes[3]);

    transmit_CAN_Message(CAN_RTS_TXB1);
}

void HANDLE_SDO_READ_REQUEST(struct SDO_TX_CAN *Response,struct C_OD_ENTRY Object_Data)
{
    /*
     * determine if this is a single or multi response protocol
     */
    
    //the standard SDO response can only contain 4 data bytes
    if(Object_Data.length<=4)
    {
        Response->CCD=0x53-Object_Data.length*4;    //encodes how many bytes to NOT read

        if(Object_Data.Start_Address!=0xff)
        {
            for(uint8_t counter=0;counter<Object_Data.length;counter++)
            {
                Response->Data_bytes[counter]=Read_From_INTERNAL_EEPROM(Object_Data.Start_Address+counter); 
            }
        } 
    }
    else    //use the extended response
    {
        Response->CCD=0x41;                         //code word for extended response
        Response->Data_bytes[0]=Object_Data.length; //only one data byte is sent, which contains the total entry size in bytes
        
        Last_accessed_object=Object_Data;           //store the data in buffer
    }
}

void Process_SDO_REQUEST(uint24_t Index,uint8_t SDOMsg[8])
{
    /*
     * SDOMsg[0] = CCD
     * SDOMsg[1:3] = CAN INDEX
     * SDOMsg[4:7] = Data Bytes (Little endian)  
    */
    
    switch(Index)
    {
        case FACTORY_RESET_CAN_INDEX:
            factory_reset(); 
            break;
            
        case CLEAR_EXTERNAL_EEPROM_INDEX:
            Reset_EEPROM();
            SENSOR_DATA_BUFFER.CurrentAddress._24_bit_Address=0;
            break;
            
        case NODE_ID_CAN_INDEX:
            Assigned_Node_ID=SDOMsg[4];
            setup_heart_beat_command();  
            Reset_CAN_CONTROLLER();   
            break;
		 
        case TEMP_SENS_TRANSFER_FLAG_INDEX:
            setupSensorListCODParameters();
            SensorHeader(TempSensor,MonitoringModeOutput);
            break;
            
        case CRNT_SENS_TRANSFER_FLAG_INDEX:
            setupSensorListCODParameters();
            SensorHeader(CurrentSensor,MonitoringModeOutput);
            break;
            
        case ACCL_SENS_TRANSFER_FLAG_INDEX:
            setup_ACCL();
            setupSensorListCODParameters();
            SensorHeader(VibrationSensor,MonitoringModeOutput);
            break;
            
        case ENTER_SENSOR_MONITORING_MODE: 
            setup_monitoring_mode( SDOMsg[4]);

            break;
            
        case ENTER_CAN_BUS_MONITORING_MODE:   
            OperatingModeFlags.CAN_MonitoringModeActive  =   SDOMsg[4];
            break; 
            
        case MAXON_GRIPPER_TEST_CAN_INDEX:   
            setupSensorListCODParameters();
            OperatingModeFlags.MotorTestEnabled = SDOMsg[4];
            break; 
        
        case TRANSFER_SENSOR_DATA_FROM_EEP:
            
            break;
    };
}
void HANDLE_SDO_WRITE_REQUEST(struct SDO_TX_CAN *Response,struct C_OD_ENTRY Object_Data,uint8_t Msg[8])
{
    Response->CCD=0x60; //code for successful write

    //SDO message bytes 4-7 contain data to be written
    uint8_t Data_to_Write[4]={Msg[4],Msg[5],Msg[6],Msg[7]};
    uint8_t Write_Status = Write_to_COD(Object_Data,Data_to_Write,4);
    
    if(Write_Status!=0)
    {
        ///80h indicates an error code 
        //here the error indicates that the user tried to write a value outside of the min<=x<=max range
        Response->CCD = 0x80;            //ERROR #2
        Response->Data_bytes[0]=0x02;    
    }
}
//respond to a service-data object protocol request
void Deal_With_SDO(uint8_t Msg[8])
{
    struct SDO_TX_CAN Response;
	
	///setup as per SDO protocol
		Response.Message_ID=0x580+Assigned_Node_ID;
		Response.MessageDLC=0x8;
		Response.Index_high=Msg[2];
		Response.Index_low =Msg[1];
		Response.Index_sub =Msg[3];
	
	///initialize other bytes
		Response.Data_bytes[0]=0x00;
		Response.Data_bytes[1]=0x00;
		Response.Data_bytes[2]=0x00;
		Response.Data_bytes[3]=0x00; 
    
        uint24_t Index = (uint24_t)(Response.Index_low<<8)+((uint24_t)Response.Index_high<<16)+Response.Index_sub;
    struct C_OD_ENTRY Object_Data=FIND_CAN_OBJECT(Index,0);
    
    uint8_t check_high_byte=Msg[0]&0xf0;
    
    if(Object_Data.length!=0)   //if the object isn't found inside the Can-object dictionary, it will have a length of zero
    {
        if(check_high_byte==0x40)//read request
        {
            HANDLE_SDO_READ_REQUEST(&Response,Object_Data);
        }
        if(check_high_byte==0x20)//write request
        {
            HANDLE_SDO_WRITE_REQUEST(&Response, Object_Data,Msg);
        }
    }
    else    ///the Can Index was not found inside the Can Object dictionary
    {
        ///80h indicates an error code
        Response.CCD = 0x80;            //ERROR #1
        Response.Data_bytes[0]=0x01;    
    }
    send_SDO_Response(Response);
    
    //after a write request we check if any process needs to be executed
    if(check_high_byte==0x20)//write request
    {
        if(Response.CCD != 0x80)//write did not result in an error
        {
            Process_SDO_REQUEST(Index,Msg);
        }
    }
}
/*
 * for SDO response of more than 4 bytes
 * 
 * this operates over a handshake protocol
 * 
 * 600+ID 8 60 00 00 00 00 00 00 00 
 * 580+ID 8 00 DATA BYTES
 * 600+ID 8 70 00 00 00 00 00 00 00
 * 580+ID 8 10 DATA BYTES
 * 
 * The master alternates between 60 and 70h
 * the slave alternates between 0 and 1h for the upper nibble of the first data byte
 * the lower nibble encodes how many of the data bytes should be read
 * this is calculated as 2* (7-(READ_SIZE)) + 1
 * this is in the standard, i have no idea why you would do it this way
 * seems dumb to me
 */
void initialise_multi_read_response(struct MULTI_READ_SDO *Response,uint8_t Data)
{
    ///setup as per SDO protocol
	Response->Message_ID=0x580+Assigned_Node_ID;
	Response->MessageDLC=0x8;
      
    //we aren't barbarians 
    for(uint8_t i = 0; i<7 ; i++)
    {
        Response->Data_bytes[i]=0;
    } 
    
    ///handshake alternates between 00 and 0x10;   
    Response->valid_length = 0x00;
    if(Data==0x70)
    {
        Response->valid_length = 0x10;
    }
    
    //first byte also indicates the number of valid bytes in the message
    uint8_t Message_length=Last_accessed_object.length -Last_accessed_object.read_index;
    if(Message_length <7)
    {
        Response->valid_length +=2* (7-(Message_length)) + 1;
    }
    else
    {
        Message_length=7;
    }
    //remaining 7 bytes are the message
   
    for(uint8_t i = 0; i<Message_length ; i++)
    {
        Response->Data_bytes[i]=Last_accessed_object.default_value[Last_accessed_object.read_index];
        Last_accessed_object.read_index++;
    } 
}

void deal_with_multi_read_SDO(uint8_t Data)
{     
    struct MULTI_READ_SDO Response;
    
    initialise_multi_read_response(&Response, Data);
    
    CopyNodeIDToCANController(0x00,Response.Message_ID, CAN_TXB1CTRL_REG,false);
    
    Fill_TX_REG_of_CAN_CONTROLLER(CAN_TXB1CTRL_REG+5,9,Response.MessageDLC,Response.valid_length,Response.Data_bytes[0],Response.Data_bytes[1],Response.Data_bytes[2],Response.Data_bytes[3],Response.Data_bytes[4],Response.Data_bytes[5],Response.Data_bytes[6]);

    transmit_CAN_Message(CAN_RTS_TXB1);
   
}

#endif	/* SDO_PROTOCOL_H */

