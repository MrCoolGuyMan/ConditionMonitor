/* 
 * File:   StepperMotorControl.h
 * Author: Administrator
 *
 * Created on 2 August, 2022, 3:34 PM
 */

#ifndef STEPPERMOTORCONTROL_H
#define	STEPPERMOTORCONTROL_H

#ifdef	__cplusplus
extern "C" {
#endif


void Turn_Step_Motor(void)
{
    TRISA = 0x00;
    uint8_t delay=100;
    
    for(int i=0; i<12; i++){
        LATA = 0b00000001;
        __delay_ms(25);
        LATA = 0b00000100;
        __delay_ms(25);
        LATA = 0b00000010;
        __delay_ms(25);
        LATA = 0b00001000;
        __delay_ms(25);
    }
    
    /**
    if(stepmotorcount == 1){
    
    for(int i=0; i<12; i++){
        LATA = 0b00000001;
        __delay_ms(25);
        LATA = 0b00000100;
        __delay_ms(25);
        LATA = 0b00000010;
        __delay_ms(25);
        LATA = 0b00001000;
        __delay_ms(25);
    }
        LATA = 0b00000001;
        __delay_ms(25);
        LATA = 0b00000100;
        __delay_ms(25); 
    }
        
    if (stepmotorcount == 2){
        
        for(int i=0; i<12; i++){
        LATA = 0b00000010;
        __delay_ms(25);
        LATA = 0b00001000;
        __delay_ms(25);
        LATA = 0b00000001;
        __delay_ms(25);
        LATA = 0b00000100;
        __delay_ms(25);
    }
        LATA = 0b00000010;
        __delay_ms(25);
        LATA = 0b00001000;
        __delay_ms(25); 
        stepmotorcount = 0;
    }
    
    stepmotorcount++;**/

    return; 
}



#ifdef	__cplusplus
}
#endif

#endif	/* STEPPERMOTORCONTROL_H */

