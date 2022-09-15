/* 
 * File:   CAN Controller.h
 * Author: Administrator
 *
 * Created on 18 January, 2021, 3:05 PM
 */

#ifndef CAN_CONTROLLER_H
#define	CAN_CONTROLLER_H


    /**********************************************
     * MCP2515 expects the first byte after lowering the cs' to be
     * the instruction/command byte
     * then send the address and at least 1 byte of data
     * MCP2515 Commands; source page 66
     * RESET            1100 0000
     * READ             0000 0011
     * READ RX BUFFER   1001 0nm0   nm = address pointer
     * WRITE            0000 0010
     * LOAD TX BUFFER   0100 0abc   abc = address pointer
     * RTS              1000 0nnn   n00 = RTS for TXB2
     *                              0n0 = RTS for TXB1
     *                              00n = RTS for TXB0
     * READ STATUS      1010 0000
     * RX STATUS        1011 0000
     * BIT MODIFY       0000 0101
     * 
     * 
     * Address table is on page 63
     **********************************************/
///Commands
#define CAN_RESET           0b11000000
#define CAN_READ_COM        0b00000011
#define CAN_WRITE_COM       0b00000010
#define CAN_RTS_TXB2        0b10000100 
#define CAN_RTS_TXB1        0b10000010 
#define CAN_RTS_TXB0        0b10000001 
#define CAN_READ_STATUS     0b10100000
#define CAN_RX_STATUS       0b10110000
#define CAN_READ_RXB0_SIDH  0b10010000
#define CANREAD_RXB1_SIDH   0b10010100
#define CAN_READ_RXB0_D0    0b10010010
#define CAN_READ_RXB1_D0    0b10010110
#define CAN_BIT_MODIFY      0b00000101
///Registers
#define CAN_INTF_REG        0x2C
#define CAN_INTE_REG        0x2b
#define CAN_BFPCTRL_REG     0x0c
#define CAN_TXRTSCRTL_REG   0x0d
#define CAN_CNF1_REG        0x2a
#define CAN_CNF2_REG        0x29
#define CAN_CNF3_REG        0x28
#define CAN_RXB0CTRL_REG    0x60
#define CAN_RXB1CTRL_REG    0x70
#define CAN_CANCTRL_REG     0x0f
#define CAN_TXB0CTRL_REG    0x30    ///stores heartbeat
#define CAN_TXB1CTRL_REG    0x40
#define CAN_TXB2CTRL_REG    0x50
// bit is cleared after message has finished sending
#define CAN_TXBxCTRL_REG_TXREQ_BIT (1<<3)
//config registers
//CNF1 
#define SJW_4TQ         (0b11<<6)
#define SJW_3TQ         (0b10<<6)
#define SJW_2TQ         (0b01<<6)
#define SJW_1TQ         (0b00<<6)

#define BAUDRATE_PRESCALER  (1)
//CNF2 
#define BLT_MODE_1      (1<<7)      //sets length of Phase segment 2 as determined by PHSEG2 2:0 bits in CFN3
#define BLT_MODE_0      (0<<7)      //sets length of Phase segment 2 as greater of PS1 and IPT (2 TQ)

#define SAM_x3          (1<<6)      //sample 3 times
#define SAM_x1          (0<<6)      //sample once

#define SET_PHASE_SEG1(val)             ((val-1)<<3)    //sets phase segment 1 value            
#define SET_PROPAGATION_SEGMENT(val)    ((val-1)<<0)    //sets propagation segment value   


//CFN3
#define START_OF_FRAME_SIGNAL_BIT_ON    (1<<7)          //CLKOUT pin will signal SOF

#define WAKIF_ON    (1<<6)                              //wake up filter enabled
#define SET_PHASE_SEG2(val)             ((val-1)<<0)    //sets phase segment 2 value        

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

    ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
}
void CopyNodeIDToCANController(uint8_t Node_ID,uint16_t Node_ID_offset, uint8_t CAN_TX_REG, bool find_Node_Id)
{
    if(find_Node_Id==true)
    {
        Node_ID=check_current_value_of_8bit_OD_entry(NODE_ID_CAN_INDEX,1);
    }
    uint8_t Node_ID_L=0;
    uint8_t Node_ID_H=0;
    Parse_Node_ID(Node_ID,Node_ID_offset,&Node_ID_H,&Node_ID_L);  
    
    Fill_TX_REG_of_CAN_CONTROLLER(CAN_TX_REG+1,2,Node_ID_H,Node_ID_L);
}
///converts 16 bit node id into 2 8 bit IDs for storage in CAN controller TX buffer
void Parse_Node_ID(uint8_t Node_ID,uint16_t Node_ID_offset, uint8_t *Node_ID_H, uint8_t *Node_ID_L)
{
    uint16_t NODE=((Node_ID+Node_ID_offset)); 
    *Node_ID_L=(NODE<<5) & 0b11100000;
    *Node_ID_H=(NODE>>3)& 0b11111111;
}
void write_command_to_CAN_CONTROLLER(unsigned char Message_type, 
                                uint8_t Address,
                                uint8_t data, uint8_t final_CS_value)
{
    ChipSelect_CAN_CONTROLLER=NOT_CS_LOW;
    SPI_MULTI_WRITE(3,Message_type,Address,data);
    ChipSelect_CAN_CONTROLLER=final_CS_value;///set as low if you want to continue sending messages
}
///read data from registers starting from "Start_address"
void Repeated_Read_from_CAN_CONTROLLER(uint8_t Start_address)
{
    ChipSelect_CAN_CONTROLLER=NOT_CS_LOW;
    SPI_MULTI_WRITE(2,CAN_READ_COM,Start_address);
    return;
}
///read data from register "address"
///uses repeated read function, with n=1
unsigned char read_from_CAN_CONTROLLER(uint8_t Address)
{
    Repeated_Read_from_CAN_CONTROLLER(Address);

    unsigned char Message=READ_SPI();
    ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
    
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
    ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
         
    return 0;
}
void Reset_CAN_CONTROLLER(void)
{
    ChipSelect_CAN_CONTROLLER=NOT_CS_LOW;///sets MCP2515 to begin read
    SPI_WRITE(CAN_RESET);
    ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
    __delay_ms(1);
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
   
        uint8_t Set_CNF1        =SJW_4TQ | BAUDRATE_PRESCALER ;                                 //0b11000001;//0xc1
        uint8_t Set_CNF2        =BLT_MODE_1|SAM_x1|SET_PHASE_SEG1(4)|SET_PROPAGATION_SEGMENT(5);//0b10101010;//aa
        uint8_t Set_CNF3        =SET_PHASE_SEG2(6);                                             //0b00000101;//0x5

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
        ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
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
            
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x00,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x01,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x04,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x05,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x08,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x09,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x010,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x011,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x014,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x015,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x018,Node_ID_H,NOT_CS_HIGH);///sidh
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x019,Node_ID_L,NOT_CS_HIGH);///sidl

            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x20,0xff,NOT_CS_HIGH);///sidh-mask
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x21,0xff,NOT_CS_HIGH);///sidl-mask
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x24,0xff,NOT_CS_HIGH);///sidh-mask
            write_command_to_CAN_CONTROLLER(CAN_WRITE_COM,0x25,0xff,NOT_CS_HIGH);///sidl-mask

            write_command_to_CAN_CONTROLLER(CAN_BIT_MODIFY,CAN_RXB0CTRL_REG, 0b001000,NOT_CS_LOW);
            SPI_WRITE(0b001000);
            ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
            write_command_to_CAN_CONTROLLER(CAN_BIT_MODIFY,CAN_RXB1CTRL_REG, 0b001000,NOT_CS_LOW);
            SPI_WRITE(0b001000);
            ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
        #endif
    /*********
     * Can control register - page 60 
     * value will be set to 000000100 
     ******/
        uint8_t Bit_mod_CANCTRL =0b00000000;//normal operation mode; clock out enabled with no pre-scaler
        Bit_Mod_Mask    =0b11111111; 
        write_command_to_CAN_CONTROLLER(CAN_BIT_MODIFY,CAN_CANCTRL_REG, Bit_Mod_Mask,NOT_CS_LOW);
        SPI_WRITE(Bit_mod_CANCTRL);
        ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
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
    CopyNodeIDToCANController(Assigned_Node_ID,0x700, CAN_TXB0CTRL_REG,false);
    
    Fill_TX_REG_of_CAN_CONTROLLER(CAN_TXB0CTRL_REG+5,2,HeartBeat_DLC,HeartBeat_Msg);
  
}
///send request to send message
///data is already loaded into appropriate TX Registers
void transmit_CAN_Message(uint8_t RTS_REGISTER)
{   
    ChipSelect_CAN_CONTROLLER=NOT_CS_LOW;///sets MCP2515 to begin read
    
    SPI_WRITE(RTS_REGISTER);
    
    ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;///sets MCP2515 to end read
    
   // __delay_ms(1);  //time for message to be sent
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

void Read_CAN_Message(uint8_t Starting_Register)
{
    
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
        uint8_t data_bytes=DLC & 0b00001111;
        uint8_t counter=0;
        uint8_t Data[8];

        while(data_bytes>0)
        { 
            Data[counter]=READ_SPI();

            data_bytes--;
            counter++;
        }
        ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
        uint16_t Node_ID =(uint16_t)( ((SIDH<<3))|(SIDL>>5));
       
            
        if(Node_ID==0x00)
        {
            if(Data[0]==0x81)///reset command 
            {
                if(Data[1]==Assigned_Node_ID)
                {
                    Reset();
                    return;
                }
            }
        }
        if(Node_ID==Assigned_Node_ID+0x600)///SDO
        {      
            ///read from external EEPROM
            if((Data[1]==TRANSFER_SENSOR_DATA_FROM_EEP_LOWER)&&(Data[2]==TRANSFER_SENSOR_DATA_FROM_EEP_UPPER)&&(Data[3]==0x1))
            {
                MemoryAddress EEPROM_Address_Read_Request = {.bytes.Addr_3 = Data[4],.bytes.Addr_2 = Data[5],.bytes.Addr_1 = Data[6] };
                handle_Ext_EEPROM_READ_REQUEST(EEPROM_Address_Read_Request);
            }
            else if ((Data[0]==0x60)||(Data[0]==0x70))
            {
                deal_with_multi_read_SDO(Data[0]);
            }
            ///read from internal EEPROM
            else
            {           
                Deal_With_SDO(Data);
            }           
        }
        else
        {
            if(OperatingModeFlags.CAN_MonitoringModeActive==1)
            {
                ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
                
                SSPEN = 0;
                log_CAN_MESSAGE_TO_EEPROM (Data, DLC & 0b00001111,  Node_ID);
                return;
            }
        }
    }
    ChipSelect_CAN_CONTROLLER=NOT_CS_HIGH;
}

void check_RX(void)
{
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
    
    ChipSelect_CAN_CONTROLLER=0;///sets MCP2515 to begin read
    SPI_WRITE(CAN_RX_STATUS);
    uint8_t msg1=READ_SPI();
    ChipSelect_CAN_CONTROLLER=1;

    uint8_t message_buffer_location = msg1 & 0b11000000;

    if(message_buffer_location>0)
    {
        if((message_buffer_location==0b01000000)||(message_buffer_location==0b11000000))
        {
            ///buffer1
            Read_CAN_Message(0x61);
        } 
        if(message_buffer_location>=0b10000000)
        {
            ///buffer2
            Read_CAN_Message(0x71);
        }
    }
    
    Reset_Interrupt_flags_On_CAN_CONTROLLER();
    
    SSPEN=0;
}
#endif	/* CAN_CONTROLLER_H */

