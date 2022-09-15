/* 
 * File:   IO_Ports.h
 * Author: Administrator
 *
 * Created on 2 August, 2022, 3:36 PM
 */

#ifndef IO_PORTS_H
#define	IO_PORTS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <xc.h>
void setup_IO_Ports(void);


#define IO_PORT_PIN_0   (1<<0)
#define IO_PORT_PIN_1   (1<<1)
#define IO_PORT_PIN_2   (1<<2)
#define IO_PORT_PIN_3   (1<<3)
#define IO_PORT_PIN_4   (1<<4)
#define IO_PORT_PIN_5   (1<<5)
#define IO_PORT_PIN_6   (1<<6)
#define IO_PORT_PIN_7   (1<<7)

void TurnLatchOff(volatile uint8_t *LatchReg, uint8_t PinNum);
void TurnLatchOn(volatile uint8_t *LatchReg, uint8_t PinNum);
void ToggleLatch(volatile uint8_t *LatchReg, uint8_t PinNum);
#ifdef	__cplusplus
}
#endif

#endif	/* IO_PORTS_H */

