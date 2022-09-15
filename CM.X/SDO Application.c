
#include "SDO Application.h"

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

        case NODE_ID_CAN_INDEX:
            Assigned_Node_ID=SDOMsg[4];
            setup_heart_beat_command();  
            Reset_CAN_CONTROLLER();   
            break;
    };
}