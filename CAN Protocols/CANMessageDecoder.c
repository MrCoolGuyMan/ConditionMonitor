#include "CANMessageDecoder.h"

static CAN_Callback_t CANCallbacks[TOTAL_CAN_CALLBACKS];

void SetCANCallback(CAN_Callback_t NewCallback, uint8_t callbackSpecifier)
{
    CANCallbacks[callbackSpecifier] = NewCallback;
}

void decodeMessage(uint8_t *Data, uint8_t *DLC, uint16_t *NodeIDWithFunctionCode)
{
    uint16_t FunctionCode = (*NodeIDWithFunctionCode & (0x780));

    switch(FunctionCode)
    {
        case NMT_NODE_CONTROL:
            CANCallbacks[NMT_CALLBACK](Data,DLC,NodeIDWithFunctionCode);
            break;
            
        case NMT_NODE_MONITOR:
            break;
            
        case SYNC_COMMAND:
            break;
            
        case TIMESTAMP:
            break;
            
        case PDO_TX_1:
            break;

        case PDO_TX_2:
            break;
            
        case PDO_TX_3:
            break;
            
        case PDO_TX_4:
            break;
            
        case PDO_RX_1:
            CANCallbacks[PDO_1_CALLBACK](Data,DLC,NodeIDWithFunctionCode);
            break;
            
        case PDO_RX_2:
            break;
            
        case PDO_RX_3:
            break;
            
        case PDO_RX_4:
            break;
            
        case SDO_RX:
            CANCallbacks[SDO_CALLBACK](Data,DLC,NodeIDWithFunctionCode);
            break;
            
        default:
            break;
    }
}