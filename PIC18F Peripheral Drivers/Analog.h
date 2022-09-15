/* 
 * File:   Analog.h
 * Author: Administrator
 *
 * Created on 5 August, 2022, 1:05 PM
 */


#ifndef ANALOG_H
#define	ANALOG_H

#ifdef	__cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <xc.h>
    

#define ACQT_2TAD   0b001
#define ACQT_4TAD   0b010
#define ACQT_6TAD   0b011
#define ACQT_8TAD   0b100
#define ACQT_12TAD  0b101
#define ACQT_16TAD  0b110
#define ACQT_20TAD  0b111

#define ADCS_FOSCd2     0b000
#define ADCS_FOSCd8     0b001
#define ADCS_FOSCd32    0b010
#define ADCS_FRC        0b011
#define ADCS_FOSCd4     0b100
#define ADCS_FOSCd16    0b110
#define ADCS_FOSCd64    0b110
#define ADCS_FRC2       0b111
void setup_ADC(void);
//avoids division, multiplication, bit shifts and floats for faster sampling
//we don't bother waiting for conversion to complete
//instead we will store the previous result into our data buffer while we wait for conversion
//we never turn off ADC - this is done in the interrupt vector 
uint8_t READ_FROM_ADC_QUICK(uint8_t adc_channel,uint8_t Sensor_type, uint8_t Starting_Sensor_ID);
uint16_t READ_FROM_ADC(uint8_t adc_channel, uint8_t Sensor_Readings, uint8_t Sensor_type, uint8_t Starting_Sensor_ID,uint8_t direct_CAN_Transmission);

#ifdef	__cplusplus
}
#endif

#endif	/* ANALOG_H */

