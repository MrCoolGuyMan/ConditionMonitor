/* 
 * File:   CAN Controller.h
 * Author: Administrator
 *
 * Created on 18 January, 2021, 3:05 PM
 */
#include "MCP2515.h"

static uint8_t SPI_deviceID = 0xff;
extern uint8_t Assigned_Node_ID;
// the transmit request bit is cleared after transmission completes
void CheckIfTransmitReady(uint8_t CAN_TXBxCTRL_REG)
{
    spi_comms_setup(DEFAULT_SPI_SETTINGS); 
    
    uint8_t RegisterValue = 1;
    uint8_t timeout = 0;
    while(RegisterValue != 0)
    {
        RegisterValue = read_from_CAN_CONTROLLER(CAN_TXBxCTRL_REG);
        
        RegisterValue &= CAN_TXBxCTRL_REG_TXREQ_BIT;
        
        if(--timeout == 0)
        {
            break;
        }
        
    }
    return ;
}

///stores a variable number of bytes inside of the TX registers specified by starting reg
void Fill_TX_REG_of_CAN_CONTROLLER(uint8_t starting_REG, uint8_t num_msgs, ...)
{
    ///FYI - "Warning: (1496)" is bugged
    CheckIfTransmitReady(starting_REG & 0xf0);    // each of the three transmit buffers exists in the space 0x30-0x3D, 0x40-0x4D, 0x50 - 0x5D
                                                  // 0x30,0x40,0x50 are the main control registers
    va_list args;
    va_start(args,num_msgs);
    
    write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,starting_REG, (uint8_t)va_arg(args,uint8_t),NOT_CS_LOW);///CAN_TXB1CTRL_REG + 1 = SIDH
    
    for(uint8_t counter=1; counter<num_msgs;counter++)
    {
        SPI_WRITE(va_arg(args,uint8_t));
    }    
    va_end(args);

    SET_CS_HIGH(SPI_deviceID);
}
void CopyNodeIDToCANController(uint8_t Node_ID,uint16_t Node_ID_offset, uint8_t CAN_TX_REG, bool find_Node_Id)
{

    uint8_t Node_ID_L=0;
    uint8_t Node_ID_H=0;
    Parse_Node_ID(Node_ID,Node_ID_offset,&Node_ID_H,&Node_ID_L);  
    
    Fill_TX_REG_of_CAN_CONTROLLER(CAN_TX_REG+1,2,Node_ID_H,Node_ID_L);
}
///converts 16 bit node id into 2 8 bit IDs for storage in CAN controller TX buffer
void Parse_Node_ID(uint8_t Node_ID,uint16_t Node_ID_offset, uint8_t *Node_ID_H, uint8_t *Node_ID_L)
{
    uint16_t NODE=((Node_ID+Node_ID_offset)); 
    *Node_ID_L=(NODE<<5) & 0xe0;
    *Node_ID_H=(NODE>>3) & 0xff;
}
void write_command_to_CAN_CONTROLLER(unsigned char Message_type, 
                                uint8_t Address,
                                uint8_t data, uint8_t final_CS_value)
{
    SET_CS_LOW(SPI_deviceID);   
    SPI_MULTI_WRITE(3,Message_type,Address,data);
    
    if(final_CS_value == 1)
        SET_CS_HIGH(SPI_deviceID);///set as low if you want to continue sending messages
}
///read data from registers starting from "Start_address"
void Repeated_Read_from_CAN_CONTROLLER(uint8_t Start_address)
{
    SET_CS_LOW(SPI_deviceID);
    SPI_MULTI_WRITE(2,CAN_READ_COM,Start_address);
    return;
}
///read data from register "address"
///uses repeated read function, with n=1
unsigned char read_from_CAN_CONTROLLER(uint8_t Address)
{
    Repeated_Read_from_CAN_CONTROLLER(Address);

    unsigned char Message=READ_SPI();
    SET_CS_HIGH(SPI_deviceID);
    
    return Message;
}
uint8_t Reset_Interrupt_flags_On_CAN_CONTROLLER(void)
{
    spi_comms_setup(DEFAULT_SPI_SETTINGS); 
    
     ///reset RX flag
    uint8_t DISABLE_PINS    = 0b00000000;
    uint8_t Bit_Mod_Mask    = 0b11111111; 

    write_command_to_CAN_CONTROLLER(CAN_BIT_MODIFY,CAN_INTF_REG, Bit_Mod_Mask,NOT_CS_LOW);
    SPI_WRITE(DISABLE_PINS);
    SET_CS_HIGH(SPI_deviceID);
         
    return 0;
}
void init_CAN_CONTROLLER(uint16_t speedSetting_kbit)
{
    if(SPI_deviceID==0xff)
        SPI_deviceID = Add_CS_LATCH(&LATB,4);
    
    /***********
     *Enable RX interrupts
     * CANINTE - page 52
     * BFPCTRL - page 29; disabled RX' pins
     ***********/
    write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,CAN_INTE_REG,     0x03,NOT_CS_HIGH);
    write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,CAN_BFPCTRL_REG,  0x00,NOT_CS_HIGH);

    /*********
     * bit Timing
     * Baud rate = 125kHz
     * MCU clock = 16MHz
     * set the BRP - baud rate prescaler to 2
     * 
     * Sync Seg =1TQ
     * Prop Seg =3TQ
     * PS1      =5TQ
     * PS2      =7TQ
     * CNF1 - address 2A
        * SJW <7:6>  
        * BRP <5:0> control baud rate prescaler     
     * CNF2 - address 29
        * BTLMODE  SAM  PHSEG1<2:0>    PRSEG<2:0>  
        * PRSEG<2:0> Propagation Segment TQ length + 1
        * PHSEG1<2:0> Phase 1 Segment TQ length +1
        * SAM bit controls number of sample points
        *      0  = 1
        *      1  = 3 
        * BTLMODE controls how the length of PS2 is determined
        *      0 = PS2 length is greater than PS1 + IPT - info processing time
        *        = 2 x TQ
        *      1 = PS2 has length set in CNF3.PHSEG2

     * CFN3 - address 28
        * PHSEG<2:0>Phase 2 Segment TQ length
     ******/
    
    // supports 125, 250 and 500 kbits/s
    // default settings for 250
    uint8_t Set_CNF1 =  SJW_4TQ | MCP2515_BAUDRATE_PRESCALER_1;
    uint8_t Set_CNF2 =  BLT_MODE_1|SAM_x1|SET_PHASE_SEG1(4)|SET_PROPAGATION_SEGMENT(5);
    uint8_t Set_CNF3 =  SET_PHASE_SEG2(6);     

    if(speedSetting_kbit == 125){
        Set_CNF1    |=  MCP2515_BAUDRATE_PRESCALER_3 ;    
    }                                            
  
    else if(speedSetting_kbit ==500){
      
        Set_CNF2        =BLT_MODE_1|SAM_x1|SET_PHASE_SEG1(2)|SET_PROPAGATION_SEGMENT(2);
        Set_CNF3        =SET_PHASE_SEG2(3);                                             
    }


        write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,CAN_CNF1_REG, Set_CNF1,NOT_CS_HIGH);

        write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,CAN_CNF2_REG, Set_CNF2,NOT_CS_HIGH);

        write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,CAN_CNF3_REG, Set_CNF3,NOT_CS_HIGH);
    /*********
     * TXRTSCRTL - disable request to send pins
     * --see page 19     
     * value will be set to UUxxx000
     ******/
        uint8_t DISABLE_PINS    = 0b00000000;
        uint8_t Bit_Mod_Mask    = 0b00000111; 
        write_command_to_CAN_CONTROLLER(CAN_BIT_MODIFY,CAN_TXRTSCRTL_REG, Bit_Mod_Mask,NOT_CS_LOW);
        SPI_WRITE(DISABLE_PINS);
        SET_CS_HIGH(SPI_deviceID);
    /*********
     * rxfnsidl filter - page 34 
     * allow messages
     * 0x600 + node ID
     * Node ID 
     * 00 - (i.e. reset)
     ******/

        #define FILTER_CAN_MESSAGES
        #ifdef FILTER_CAN_MESSAGES
            uint8_t Node_ID_L=0;
            uint8_t Node_ID_H=0;
            
            Parse_Node_ID(Assigned_Node_ID,0x600,&Node_ID_H,&Node_ID_L);
            
            //filter for SDO Protocol
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF0SIDH_FILTER,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF0SIDL_FILTER,Node_ID_L,NOT_CS_HIGH);///sidl

            //filter for PDO protocol
            Parse_Node_ID(Assigned_Node_ID,0x200,&Node_ID_H,&Node_ID_L);
            
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF1SIDH_FILTER,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF1SIDL_FILTER,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF2SIDH_FILTER,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF2SIDL_FILTER,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF3SIDH_FILTER,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF3SIDL_FILTER,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF4SIDH_FILTER,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF4SIDL_FILTER,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF5SIDH_FILTER,0,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXF5SIDL_FILTER,0,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXM0SIDH_MASK,0xff,NOT_CS_HIGH);///sidh-mask
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXM0SIDL_MASK,0xff,NOT_CS_HIGH);///sidl-mask
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXM1SIDH_MASK,0xff,NOT_CS_HIGH);///sidh-mask
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,RXM1SIDH_MASK,0xff,NOT_CS_HIGH);///sidl-mask
            
#define RX0_ROLLS_OVER_INTO_RX1 (0b1<<1)
            write_command_to_CAN_CONTROLLER(CAN_BIT_MODIFY,CAN_RXB0CTRL_REG, RX_MASKS_ONLY_SID|RX0_ROLLS_OVER_INTO_RX1,NOT_CS_LOW);
           // SPI_WRITE(0b001000);
            SET_CS_HIGH(SPI_deviceID);
            write_command_to_CAN_CONTROLLER(CAN_BIT_MODIFY,CAN_RXB1CTRL_REG, RX_MASKS_ONLY_SID,NOT_CS_LOW);
          //  SPI_WRITE(0b001000);
            SET_CS_HIGH(SPI_deviceID);
        #endif
    /*********
     * Can control register - page 60 
     * value will be set to 000000100 
     ******/
        uint8_t Bit_mod_CANCTRL =0b00000000 | FCLK_OUT_SELECTION;//normal operation mode; clock out enabled with no pre-scaler
        Bit_Mod_Mask    =0b11111111; 
        write_command_to_CAN_CONTROLLER(CAN_BIT_MODIFY,CAN_CANCTRL_REG, Bit_Mod_Mask,NOT_CS_LOW);
        SPI_WRITE(Bit_mod_CANCTRL);
        SET_CS_HIGH(SPI_deviceID);
        
}

///send request to send message
///data is already loaded into appropriate TX Registers
void transmit_CAN_Message(uint8_t RTS_REGISTER)
{   
    SET_CS_LOW(SPI_deviceID);///sets MCP2515 to begin read
    
    SPI_WRITE(RTS_REGISTER);
    
    SET_CS_HIGH(SPI_deviceID);///sets MCP2515 to end read
}
void Reset_CAN_CONTROLLER(void)
{

    SET_CS_LOW(SPI_deviceID);    
    
    SPI_WRITE(CAN_RESET);
    
    SET_CS_HIGH(SPI_deviceID);
    
}
void send_heartbeat_message(void)
{
    spi_comms_setup(DEFAULT_SPI_SETTINGS);
     /*******
      * Set Node ID in TXBnSIDL - page 20 (address 0x32)
      * Set number of bytes and RTR status in TXB0DLC (address 0x35)
      * Set Heartbeat "05" byte to TXB0D0 (address 0x36)
      * Fire RTS for TXB0
     *******/

    transmit_CAN_Message(CAN_RTS_TXB0);
    SSPCON1bits.SSPEN=0;
}

CAN_RX_Buffer Read_CAN_Message(uint8_t Starting_Register)
{
    CAN_RX_Buffer Read_Message;
    
    Repeated_Read_from_CAN_CONTROLLER(Starting_Register);
    uint8_t             SIDH=READ_SPI();       //standard identifier High
    uint8_t             SIDL=READ_SPI();       //standard identifier Low
    volatile uint8_t    EIDH=READ_SPI();       //extended identifier High  -> not used 
    volatile uint8_t    EIDL=READ_SPI();       //extended identifier Low   -> not used
    uint8_t             DLC=READ_SPI();        //Data Length Code
    
    //SIDL contains
    /***
     * page 30
     * 
     * bits 7-5 SID <2:0> Node ID 3 LSBs
     * bit 4 SRR Standard remote frame transfer bit
     *          1 = RTR
     *          0 = standard
     * 
     * bit 3 IDE - extended identifier bit flag
     *          1=extended frame
     *          0= standard frame
     * 
     * bit 2 -unimplemented
     * 
     * bit1-0 extended ID MSBs
     ****/
    uint8_t Frame_Type = SIDL & 0b00010000;
    
    if(Frame_Type==0)///standard frame
    {
        /*
         * RXBnDLC - page 31
         * bit 7 - U
         * bit 6 - RTR (Extended frame)
         * bit 5-4 - reserved 
         * bit 3-0 - data length code
         */
        Read_Message.DLC=0;
        uint8_t data_bytes=DLC & 0b00001111;
        
        
        while(data_bytes>0)
        { 
            Read_Message.Data[Read_Message.DLC++] = READ_SPI();

            data_bytes--;
        }

        Read_Message.MessageID = (uint16_t)( ((SIDH<<3))|(SIDL>>5));
        
    }
    SET_CS_HIGH(SPI_deviceID);
    
    
    return Read_Message;
}

CAN_RX_Buffer check_RX(void)
{
    CAN_RX_Buffer Read_Message;
    spi_comms_setup(DEFAULT_SPI_SETTINGS);
    
    /*****
     *request RX status from MCP2515
     * 
     * bits 7-6 code the source
     * 7|6
     * 0|0 - no message
     * 0|1 - RXB0
     * 1|0 - RXB1
     * 1|1 - RXB0-1
     * 
     * bits 4-3 code the message type
     * 00 - standard data frame
     * 01 - standard remote frame
     * 10 - extended frame
     * 11 - extended remote frame
     * 
     * the message is repeated twice
     * CANINTE.RXnIE must be cleared
     * 
     ***/
    
    SET_CS_LOW(SPI_deviceID);
    SPI_WRITE(CAN_RX_STATUS);
    uint8_t msg1=READ_SPI();
    SET_CS_HIGH(SPI_deviceID);

    uint8_t message_buffer_location = msg1 & 0b11000000;

    if(message_buffer_location>0)
    {
        if((message_buffer_location==0b01000000)||(message_buffer_location==0b11000000))
        {
            ///buffer1
            Read_Message = Read_CAN_Message(0x61);
        } 
        else if(message_buffer_location>=0b10000000)
        {
            ///buffer2
            Read_Message = Read_CAN_Message(0x71);
        }
    }
    
    Reset_Interrupt_flags_On_CAN_CONTROLLER();
    
    SSPEN=0;
    
    return Read_Message;
}