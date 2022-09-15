#include "PDO Protocol.h"
static PDO_CALLBACK_t PDO_CALLBACK_LIST[PDO_TOTAL_INSTRUCTIONS];

void SetPDOCallback(PDO_CALLBACK_t NewCallback, uint8_t CallbackID)
{    
    PDO_CALLBACK_LIST[CallbackID] = NewCallback;
}
void decodePDO (uint8_t *data,uint8_t *DLC, uint16_t *NodeIDWithFunctionCode)
{
    PDO_CALLBACK_LIST[data[0]](data,DLC,NodeIDWithFunctionCode);
}