/* 
 * File:   Prototypes.h
 * Author: Administrator
 *
 * Created on 3 December, 2021, 10:11 AM
 */

#ifndef PROTOTYPES_H
#define	PROTOTYPES_H

/*
 *  General Process Control
 */

int32_t BME280_compensate_T_int32 (int32_t adc_T);
uint32_t BME280_compensate_P_int32 (int32_t adc_P);
uint32_t BME280_compensate_H_int32 (int32_t adc_H);

void Turn_Step_Motor(void);

#endif	/* PROTOTYPES_H */

