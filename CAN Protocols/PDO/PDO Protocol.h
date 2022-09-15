#ifndef PDO_PROTOCOL_h
#define PDO_PROTOCOL_h
#include <stdint.h>
#include "PDO Instruction Set.h"

typedef struct 
{
    uint8_t Data[8];
    uint8_t DLC;
    uint16_t MsgID;      // includes function code
}PDO_RX_MSG;

typedef void (*PDO_CALLBACK_t)(uint8_t Message[8],uint8_t *DLC, uint16_t *NodeIDWithFunctionCode);

void decodePDO (uint8_t *data,uint8_t *DLC, uint16_t *NodeIDWithFunctionCode);

void SetPDOCallback(PDO_CALLBACK_t NewCallback, uint8_t CallbackID);
#endif