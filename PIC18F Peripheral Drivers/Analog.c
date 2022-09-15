#include "Analog.h"
void setup_ADC(void)
{

    VREFCON0bits.FVREN = 1; //fixed voltage reference enabled
    VREFCON0bits.FVRS = 0b11;//4.096V
    /****************************
     set ADCON1; page 295
     ****************************/
    ADCON1bits.TRIGSEL=0b1;
    ADCON1bits.PVCFG=0b10;  ///positive voltage reference bits fixed voltage reference
    ADCON1bits.NVCFG=0b00;
    /****************************
     set ADCON2; page 296
     ***************************/
    ADCON2bits.ADFM=0b1;    ///set high/low byte order
    ADCON2bits.ACQT=ACQT_8TAD; 
    
    #if USE_PLL == 1           //3x PLL
        ADCON2bits.ADCS=ADCS_FOSCd64;  //Fosc/64 
    #else
        ADCON2bits.ADCS=ADCS_FOSCd16;  //Fosc/16 
    #endif
    ADCON0bits.ADON= 1;
    
    return;
}
//avoids division, multiplication, bit shifts and floats for faster sampling
//we don't bother waiting for conversion to complete
//instead we will store the previous result into our data buffer while we wait for conversion
//we never turn off ADC - this is done in the interrupt vector 
uint8_t READ_FROM_ADC_QUICK(uint8_t adc_channel,uint8_t Sensor_type, uint8_t Starting_Sensor_ID)
{   
    ADCON0bits.CHS = adc_channel;
    ADCON0bits.ADON = 1;
    ADCON0bits.GO_DONE = 1 ;
    
    uint8_t sensor_id_with_offset = adc_channel-Starting_Sensor_ID+1;

    sensor_id_with_offset<<=2;
    // This is the previous result
    while(ADCON0bits.GO_DONE==1)//in case the previous wasn't completed yet
    {

    }

    
    ADCON0bits.ADON = 0;

    return sensor_id_with_offset;
}
uint16_t READ_FROM_ADC(uint8_t adc_channel, uint8_t Sensor_Readings, uint8_t Sensor_type, uint8_t Starting_Sensor_ID,uint8_t direct_CAN_Transmission)
{
    ADCON0bits.CHS= adc_channel;
    
    uint32_t data=0;
    uint8_t sensor_id_with_offset=0;

    for (int counter=0; counter<1;counter++)
    {
        ADCON0bits.ADON= 1;
        ADCON0bits.GO_DONE=1;

        //do this first since we can't proceed until analysis is done anyway
        sensor_id_with_offset = adc_channel-Starting_Sensor_ID+1;
    
        sensor_id_with_offset<<=2;
        ///wait until "go" == 0 (i.e. complete)
        while(ADCON0bits.GO_DONE==1)
        {

        }
        uint32_t temp = (uint16_t)((ADRESH<<8)|ADRESL);
       /* if(direct_CAN_Transmission==Calculate_Current_as_RMS_Value)
        {
            temp*=temp;
        }*/
            
        data+=temp;

        ADRESH=0x00;
        ADRESL=0x00;
    }
    data/=Sensor_Readings;
    
    uint16_t final_data=0;
    
  /*  this is using a lot of memory and isn't particularly useful
   * if(direct_CAN_Transmission==Calculate_Current_as_RMS_Value)
    {
        final_data = (uint16_t) (sqrt(data));
    }
    else
    {*/
        final_data=data&0xffff;
 //   }

    ADCON0=0x00; ///Turn ADC off
//    FormatSensorMetaData(direct_CAN_Transmission,(final_data>>8)&0x3, final_data&0xff,  Sensor_type,  sensor_id_with_offset);
   
    return final_data;
}

