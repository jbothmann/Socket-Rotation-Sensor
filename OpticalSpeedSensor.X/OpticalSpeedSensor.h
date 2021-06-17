// 09/286/2020
// OpticalSpeedSensor.h
// Optical speed sensor for ratchet bolt run down testing

//---------------------Specifications
//  PIC16F1825, 14 pin TSSOP package
//  Fosc, Clock frequency = 16MHz, (4Mhz internal clock)
//  Tcy, (Instruction cycle time = 4 x (1/Fosc) = 250ns)
//  Vcc = 5.0VDC
//

//---------------------Revisions
// REV 1.0 07/18/2017   Number of bytes received and transmitted is 18



// G L O B A L  V A R I A B L E S ----------------------------------------------
//
unsigned char bRisingEdges;         // Number of rising edges on the optical sensor
unsigned short sCurrentState;       //
unsigned short sLastState;          //
unsigned char bFlags_1;             // Status flags
unsigned char bFlags_2;             // Status flags
signed short swCurrentADCVal;       // Current ADC sample
unsigned char bADCHigh;             // ADC high byte
unsigned char bADCLow;              // ADC low byte
unsigned long sGenTimer1;          // General use timer 1
unsigned long sGenTimer2;          // General use timer 2




// D E F I N I T I O N S -------------------------------------------------------

#define _XTAL_FREQ  16000000      // Fosc =  16MHz, instr time = 16MHz/4 = 250ns.
                                    // This is also used by the __delay_ms(xx) and __delay_us(xx) functions


// Timer1 Value for 1ms time out
#define TIMER1HVAL  0xFC            // Setup Timer1 to overflow in 1ms and interrupt
#define TIMER1LVAL  0x18            // Time unit = 1MHz (1us)
                                    // 1ms / 1us = 1000
                                    // 65536 - 1000 = 64536(FC18h)
                                    // Load Timer, TMR1H = 0xFC and TMR1L = 0x18
                                    // TMR1H = 0xFC;
                                    // TMR1L = 0x18;
                                    // Adjusted for INT routine time, inserted two asm("NOP") to get 1.000125ms
                                    // 64538(FC1Ah))

#define LEDONTIME   250             // LED On Time (250ms) 
#define LEDOFFTIME  250             // LED Off Time (250ms)


// Status bFlags_1 Flags
#define SENS_STAT           0x01    // If output is high, set  
//#define LEDBLINK            0x02    // Indicates if LED is blinking
#define LEDONOFF            0x04    // Indicates if LED is On or Off
//#define EDGEHIGH            0x08    // Sensor high pulse is still high
#define BSLEDBLINK          0x10    // Status of BAD_STATE LED blinking
//#define FlagName            0x20    // Unused  
//#define FlagName            0x40    // Unused   
//#define FlagName            0x80    // Unused


// Status bFlags_2 Flags
//#define FlagName           0x01    //  
//#define FlagName           0x02    // 
//#define FlagName           0x04    // 
//#define FlagName           0x08    // 
//#define FlagName           0x10    // 
//#define FlagName           0x20    //  
//#define FlagName           0x40    // 
//#define FlagName           0x80    // 


// State Machine
#define RESETSTATE           1   // Power up state
#define BEGIN_COUNT          2   // Timers and Counters are reset
#define COUNTING             3   // Count edges with ISR, time duration of state
#define EVALUATE             4   // Compare edge count and trigger duration
#define NORMAL_WAIT          5   // Signal that the cycle was within parameters, wait for next enable
#define ERROR_WAIT           6   // Signal that the cycle was outside parameters, wait for next enable
#define TIMEOUT_ERROR        7   // Signal that the cycle was enabled for too long, discard cycle

#define TIMER_LIMIT 10000 // (10 seconds) If the timer exceeds this value, the program will ented the timeout_error state

#define EDGE_TIME_MIN_RATIO 4 // (4 rotations per second) 

#define STAT_LED_OFF    LATAbits.LATA4 = 1  // Turn green Status LED off
#define STAT_LED_ON     LATAbits.LATA4 = 0  // Turn green Status LED on

#define SENS_STAT_OFF   LATAbits.LATA5 = 1  // Turn Sensor status off
#define SENS_STAT_ON    LATAbits.LATA5 = 0  // Turn Sensor status on

#define SENSOR_IN       PORTAbits.RA2       // Optical sensor input line, No light = pin low, Light = pin high 

#define TRG_STATUS      PORTCbits.RC3       // Tool trigger status, Low indicates trigger pulled



// S T R U C T U R E S ---------------------------------------------------------

// Flags
// Clear status flags
    //flags.BUTTONPRESS = 0;
    //flags.STUCKBUTTON = 0;   
//struct
//{
//  unsigned BUTTONPRESS :1;  // indicates button status (1 = pressed),(0 = released)
//  unsigned STUCKBUTTON :1;  // indicates state of button after stuck button check (1 = stuck, 0 = not stuck)
//  
//}flags;


// P R O T O T Y P E S ---------------------------------------------------------

void __interrupt() ISR_Routine(void);
void TMR1_Init(void);
void ADC_Init(void);
void SPI_MasterInit(void);
void SPI_SlaveInit(void);
void RS232_Init(void);
void UART_Write(unsigned short data);
unsigned char UART_ChkRec_Err();
unsigned short Read_ADC_Value(void);
void Enable_Timer1_Int(void);
void Disable_Timer1_Int(void);
void Enable_External_Rising_Int(void);
void Disable_External_Rising_Int(void);
void Enable_UART_RX_Int(void);




// WRITE INTIAL EEPROM VALUES ---------------------------------------------------------

//__EEPROM_DATA(0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07);    //Address 0 - 7
//__EEPROM_DATA(0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F);    //Address 8 - 15
//__EEPROM_DATA(0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17);    //Address 16 - 23
//__EEPROM_DATA(0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F);    //Address 24 - 31
//__EEPROM_DATA(0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27);    //Address 32 - 39
//__EEPROM_DATA(0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F);    //Address 40 - 47
//__EEPROM_DATA(0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37);    //Address 48 - 55
//__EEPROM_DATA(0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F);    //Address 56 - 63
//__EEPROM_DATA(0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47);    //Address 64 - 71
//__EEPROM_DATA(0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F);    //Address 72 - 79
//__EEPROM_DATA(0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57);    //Address 80 - 87
//__EEPROM_DATA(0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F);    //Address 88 - 95
//__EEPROM_DATA(0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67);    //Address 96 - 103
//__EEPROM_DATA(0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F);    //Address 104 - 111
//__EEPROM_DATA(0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77);    //Address 112 - 119
//__EEPROM_DATA(0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F);    //Address 120 - 127
//__EEPROM_DATA(0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07);    //Address 128 - 135
//__EEPROM_DATA(0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F);    //Address 136 - 143
//__EEPROM_DATA(0x10,0x11,0x12,0x13,0x14,0x15,0x16,0x17);    //Address 144 - 151
//__EEPROM_DATA(0x18,0x19,0x1A,0x1B,0x1C,0x1D,0x1E,0x1F);    //Address 152 - 159
//__EEPROM_DATA(0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27);    //Address 160 - 167
//__EEPROM_DATA(0x28,0x29,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F);    //Address 168 - 175
//__EEPROM_DATA(0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37);    //Address 176 - 183
//__EEPROM_DATA(0x38,0x39,0x3A,0x3B,0x3C,0x3D,0x3E,0x3F);    //Address 184 - 191
//__EEPROM_DATA(0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47);    //Address 192 - 199
//__EEPROM_DATA(0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F);    //Address 200 - 207
//__EEPROM_DATA(0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57);    //Address 208 - 215
//__EEPROM_DATA(0x58,0x59,0x5A,0x5B,0x5C,0x5D,0x5E,0x5F);    //Address 216 - 223
//__EEPROM_DATA(0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67);    //Address 224 - 231
//__EEPROM_DATA(0x68,0x69,0x6A,0x6B,0x6C,0x6D,0x6E,0x6F);    //Address 232 - 239
//__EEPROM_DATA(0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77);    //Address 240 - 247
//__EEPROM_DATA(0x78,0x79,0x7A,0x7B,0x7C,0x7D,0x7E,0x7F);    //Address 248 - 255
