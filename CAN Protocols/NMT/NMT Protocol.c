
#include "NMT Protocol.h"
extern uint8_t Assigned_Node_ID;

void handleNMT(uint8_t *Data, uint8_t *DLC, uint16_t *NodeID)
{
    if(Data[0]==GOTO_RESET_NODE)///reset command 
    {
        //if(Data[1]==Assigned_Node_ID)
        {
            Reset();
            return;
        }
    }
}