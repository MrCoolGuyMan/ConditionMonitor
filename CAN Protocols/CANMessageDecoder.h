
#ifndef CAN_MSG_DECODER_h
#define CAN_MSG_DECODER_h
#include <stdint.h>
#include "Function Codes/FunctionCodes.h"
//#include "xc.h"
//#include "PDO/PDO Protocol.h"
//#include "SDO/SDO Protocol.h"

enum CANCallbackPositions{
    PDO_1_CALLBACK = 0,
    SDO_CALLBACK,
    NMT_CALLBACK,
    TOTAL_CAN_CALLBACKS
};
typedef void (*CAN_Callback_t)(uint8_t *Data, uint8_t *DLC, uint16_t *NodeIDWithFunctionCode);

void decodeMessage(uint8_t *Data, uint8_t *DLC, uint16_t *NodeIDWithFunctionCode);

void SetCANCallback(CAN_Callback_t NewCallback, uint8_t callbackSpecifier);
#endif