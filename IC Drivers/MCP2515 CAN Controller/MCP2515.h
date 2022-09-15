/* 
 * File:   CAN Controller.h
 * Author: Administrator
 *
 * Created on 18 January, 2021, 3:05 PM
 */

#ifndef CAN_CONTROLLER_H
#define	CAN_CONTROLLER_H

#include "SPI_Interface.h"
#include <stdint.h>
#include <stdbool.h>

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


#define BAUDRATE_PRESCALER  (3) // This was set to 1 on the old clock
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


//clock prescalers
#define FCLK_OUT_ENABLE     (0b1<<2)
#define FCLK_OUT_1_TO_1     (0b00)
#define FCLK_OUT_2_TO_1     (0b01)
#define FCLK_OUT_4_TO_1     (0b10)
#define FCLK_OUT_8_TO_1     (0b11)
#if USE_INTERNAL_OSC == 0
        #define FCLK_OUT_SELECTION  (FCLK_OUT_ENABLE | FCLK_OUT_1_TO_1)
    #else
        #define FCLK_OUT_SELECTION  (0)
#endif


// filters

    #define RXF0SIDH_FILTER (0x00)
    #define RXF0SIDL_FILTER (0x01)

    #define RXF1SIDH_FILTER (0x04)
    #define RXF1SIDL_FILTER (0x05)


    #define RXF2SIDH_FILTER (0x08)
    #define RXF2SIDL_FILTER (0x09)

    #define RXF3SIDH_FILTER (0x10)
    #define RXF3SIDL_FILTER (0x11)

    #define RXF4SIDH_FILTER (0x14)
    #define RXF4SIDL_FILTER (0x15)
    #define RXF5SIDH_FILTER (0x18)
    #define RXF5SIDL_FILTER (0x19)

    #define RXM0SIDH_MASK   (0x20)
    #define RXM0SIDL_MASK   (0x21)         
    #define RXM1SIDH_MASK   (0x24)
    #define RXM1SIDL_MASK   (0x25)

// filter settings for RX controler buffer (0x60, 0x70)
#define RX_MASKS_OFF        (0b11<<5)
#define RX_MASKS_ONLY_SID   (0b10<<5)
#define RTR_RX_ON           (1<<3)

typedef struct
{
    uint16_t MessageID;
    uint8_t DLC;
    uint8_t Data[8];
}CAN_RX_Buffer;
// the transmit request bit is cleared after transmission completes
void CheckIfTransmitReady(uint8_t CAN_TXBxCTRL_REG);
///stores a variable number of bytes inside of the TX registers specified by starting reg
void Fill_TX_REG_of_CAN_CONTROLLER(uint8_t starting_REG, uint8_t num_msgs, ...);
void CopyNodeIDToCANController(uint8_t Node_ID,uint16_t Node_ID_offset, uint8_t CAN_TX_REG, bool find_Node_Id);
///converts 16 bit node id into 2 8 bit IDs for storage in CAN controller TX buffer
void Parse_Node_ID(uint8_t Node_ID,uint16_t Node_ID_offset, uint8_t *Node_ID_H, uint8_t *Node_ID_L);
void write_command_to_CAN_CONTROLLER(unsigned char Message_type, 
                                uint8_t Address,
                                uint8_t data, uint8_t final_CS_value);
///read data from registers starting from "Start_address"
void Repeated_Read_from_CAN_CONTROLLER(uint8_t Start_address);
///read data from register "address"
///uses repeated read function, with n=1
unsigned char read_from_CAN_CONTROLLER(uint8_t Address);
uint8_t Reset_Interrupt_flags_On_CAN_CONTROLLER(void);
void Reset_CAN_CONTROLLER(void);
void init_CAN_CONTROLLER(void);
///send request to send message
///data is already loaded into appropriate TX Registers
void transmit_CAN_Message(uint8_t RTS_REGISTER);
void send_heartbeat_message(void);

CAN_RX_Buffer Read_CAN_Message(uint8_t Starting_Register);
CAN_RX_Buffer check_RX(void);
#endif	/* CAN_CONTROLLER_H */

