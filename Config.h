/* 
 * File:   Config.h
 * Author: Administrator
 *
 * Created on 23 November, 2021, 11:52 AM
 */

#ifndef CONFIG_H
#define	CONFIG_H

#define CURRENT_SENSOR_START_PIN 0
#define TEMPERATURE_SENSOR_START_PIN 20
#define ACCL_SENSOR_START_PIN 40
#define CAN_INTERRUPT_PIN PORTBbits.RB2
//NumberOfSensorsInSensorList must be the final value
enum SensorListPositions {TempSensor=0,CurrentSensor=1,VibrationSensor=2,Encoder1=3,/*Encoder2 is number 4*/ NumberOfSensorsInSensorList};
//bit shifted because of eeprom formatting
#define MetaDataOffsetAmount            5
#define Temperature_Sensor_type         (TempSensor<<MetaDataOffsetAmount)         //0
#define Current_Sensor_type             (CurrentSensor<<MetaDataOffsetAmount)      //1
#define Vibration_Sensor_type           (VibrationSensor<<MetaDataOffsetAmount)    //2
#define CAN_MESSAGE_as_sensor_type      7
#define Encoder_Sensor_type             Encoder1
#define Encoder_Meta_Data               (unsigned) (Encoder1 << (MetaDataOffsetAmount+8)) 
#define Encoder2_Meta_Data              (unsigned) ((Encoder1+1) << (MetaDataOffsetAmount+8)) 
#define CAN_MSG_Meta_Data               (CAN_MESSAGE_as_sensor_type << MetaDataOffsetAmount)
//SensorData bit shifted for Meta Data Formatting
#define X_AXIS 1<<2
#define Y_AXIS 2<<2
#define Z_AXIS 3<<2

#define MOTOR_TEST_ENABLED 1
#define MOTOR_TEST_HALTING 0
#define MOTOR_TEST_DISABLED 2
#define MOTOR_TEST_EMERGENCY_HALT 3
#define EEPROM_OUTPUT_MODE  0
#define CAN_OUTPUT_MODE_1 1
#define CAN_OUTPUT_MODE_2 2
//Timers
#define USE_PLL 1
#define PLL_MUL 3
#if USE_PLL == 1
    #define _XTAL_FREQ 16000000*PLL_MUL      ///needed for delays
#else
    #define _XTAL_FREQ 16000000
#endif

#define _FREQ_MHZ               (_XTAL_FREQ/1000000)
#define INSTRUCTION_FREQUENCY   (_FREQ_MHZ/4)
//Timer0
#define TIMER0_INTERRUPT_TIME_us        50
#define ProgramTimerDivider             1000/TIMER0_INTERRUPT_TIME_us
#define TIMER0_COUNTS_REQUIRED          (TIMER0_INTERRUPT_TIME_us*INSTRUCTION_FREQUENCY)
//takes 7 instructions to reset timer0 counter
//so we need to account for this
#define TIMER0_RESET_INSTRUCTION_CYCLES     7
#define INTERNAL_INTERRUPT_LATENCY_CYCLES   3
#define TIMER0_TOTAL_LATENCY_INSTRUCTION_CYCLES (TIMER0_RESET_INSTRUCTION_CYCLES+INTERNAL_INTERRUPT_LATENCY_CYCLES)
#define TIMER0_START_COUNT              (uint16_t)(65536 - TIMER0_COUNTS_REQUIRED + TIMER0_TOTAL_LATENCY_INSTRUCTION_CYCLES)

//thresholds are based on how many times the timer must interrupt before the specific period is over
#define Heart_Beat_Period_us            500000
#define Heart_Beat_TMR_Interrupts      (Heart_Beat_Period_us/TIMER0_INTERRUPT_TIME_us)

#define Motor_Acc_Period_us             10000
#define Motor_Acc_TMR_Interrupts        (Motor_Acc_Period_us/TIMER0_INTERRUPT_TIME_us)
//#define Motor_Status_Update_Rate        (500000/TIMER0_INTERRUPT_TIME_us)
#define Motor_Status_Update_Rate        0
#define motor_duty_cycle_delta          1

#define Sampling_period_TMR_Interrupts  (1000)//TIMER0_INTERRUPT_TIME_us)

#define Calculate_Current_as_RMS_Value 2

#define USE_INTERNAL_OSC 
// CONFIG1L

#pragma config PLLSEL = PLL3X   // PLL Selection (4x clock multiplier)
#if USE_PLL == 0
    #pragma config CFGPLLEN = OFF   // PLL Enable Configuration bit (PLL Disabled (firmware controlled))
#else 
    #pragma config CFGPLLEN = ON
#endif

#pragma config CPUDIV = NOCLKDIV// CPU System Clock Postscaler (CPU uses system clock (no divide))
#pragma config LS48MHZ = SYS24X4// Low Speed USB mode with 48 MHz system clock (System clock at 24 MHz, USB clock divider is set to 4)

// CONFIG1H
#ifdef USE_INTERNAL_OSC
    #pragma config FOSC =  1000   // Oscillator Selection (Internal oscillator)
    #pragma config PCLKEN = OFF     // Primary Oscillator Shutdown (Primary oscillator enabled)
#else
    #pragma config FOSC = ECLIO     // 1000//RCIO//  // Oscillator Selection (Internal oscillator)
    #pragma config PCLKEN = ON     // Primary Oscillator Shutdown (Primary oscillator enabled)
#endif
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config nPWRTEN = OFF    // Power-up Timer Enable (Power up timer disabled)
#pragma config BOREN = OFF  // Brown-out Reset Enable (BOR enabled in hardware (SBOREN is ignored))
#pragma config BORV = 190       // Brown-out Reset Voltage (BOR set to 1.9V nominal)
#pragma config nLPBOR = ON     // Low-Power Brown-out Reset (Low-Power Brown-out Reset disabled)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (WDT disabled in hardware (SWDTEN ignored))
#pragma config WDTPS = 32768    // Watchdog Timer Postscaler (1:32768)

// CONFIG3H
#pragma config CCP2MX = RC1     // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config T3CMX = RB5      // Timer3 Clock Input MUX bit (T3CKI function is on RC0)
#pragma config SDOMX = RB3      // SDO Output MUX bit (SDO function is on RB3)
#pragma config MCLRE = ON       // Master Clear Reset Pin Enable (MCLR pin enabled; RE3 input disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset (Stack full/underflow will cause Reset)
#pragma config LVP = OFF//ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port Enable (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled)

// CONFIG5L
#pragma config CP0 = OFF        // Block 0 Code Protect (Block 0 is not code-protected)
#pragma config CP1 = OFF        // Block 1 Code Protect (Block 1 is not code-protected)
#pragma config CP2 = OFF        // Block 2 Code Protect (Block 2 is not code-protected)
#pragma config CP3 = OFF        // Block 3 Code Protect (Block 3 is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protect (Boot block is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protect (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Block 0 Write Protect (Block 0 (0800-1FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Block 1 Write Protect (Block 1 (2000-3FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Block 2 Write Protect (Block 2 (04000-5FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Block 3 Write Protect (Block 3 (06000-7FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Registers Write Protect (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protect (Boot block (0000-7FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protect (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Block 0 Table Read Protect (Block 0 is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Block 1 Table Read Protect (Block 1 is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Block 2 Table Read Protect (Block 2 is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Block 3 Table Read Protect (Block 3 is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protect (Boot block is not protected from table reads executed in other blocks)

#pragma warning disable 1496
//Error Codes
#define ERROR_CODE_OKIE_DOKES 0         //i.e. no error code
#define ERROR_CODE_NOT_OKIE_DOKES 255
#endif	/* CONFIG_H */

