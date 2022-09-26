#include "NMT Protocol.h"

void handleNMT(uint8_t *Data, uint8_t *DLC, uint16_t *NodeID)
{
    if(Data[0]==GOTO_RESET_NODE)///reset command 
    {
        Reset();
        return;
    }
}