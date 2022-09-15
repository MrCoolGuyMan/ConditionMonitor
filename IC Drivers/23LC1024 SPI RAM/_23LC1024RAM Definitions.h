
#ifndef _23LC1024_DEFINES_H
#define _23LC1024_DEFINES_H
#include <stdint.h>
// Application and architecture Specific 
// Make a copy of this file; do not edit
// be default this is setup for an 8 bit MCU transferring 256 bits of data
	#define _23LC1024_RAM_BUFFER_SIZE (256)
	
	// change "uint8_t" to match the buffer transfer size; below works because RAMBuffer.Current size is always incremented before comparison
	typedef uint8_t 			_23LC1024_RAM_BUFFER_SIZE_TYPE	;
	
	// pic 18 allows 24 bit int types
	typedef uint24_t 			_23LC1024_21bit_Address_t;
// must define "REGISTER" (i.e. voltaile uintx_t ) and "REGISTER_BIT" (uintx_t etc)
// must match register size; otherwise compilation error will occur
	typedef uint8_t 			REGISTER_BIT ;
	typedef volatile uint8_t 	REGISTER ;

// see "_23LC1024_BitAddress_t" typedef; read compiler documentation; structs may pack from front to back or back to front
	#define STRUCT_HIGH_END_PACK
	
	
#endif  //_23LC1024_DEFINES_H