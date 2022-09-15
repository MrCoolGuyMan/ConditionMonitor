
#ifndef NMT_PROTOCOL_h
#define NMT_PROTOCOL_h
#include <stdint.h>
#include "Function Codes/FunctionCodes.h"
#include <xc.h>

void handleNMT(uint8_t *Data, uint8_t *DLC, uint16_t *NodeID);
#endif