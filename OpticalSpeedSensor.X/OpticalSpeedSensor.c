// 09/28/2020
// OpticalSpeedSensor.c
//
// Optical speed sensor for ratchet bolt run down testing
//
//
//---------------------Revisions
//
//  REV 1.0: 07/18/2017
//	This program will detect the optical sensor output that is monitoring the socket
//  in the bolt run down test. It will also monitor the PLC trigger output to use as feedback
//  when the tools trigger is moved into the run position.
//	The output of this sensor board will be an open collector NPN transistor that the PLC
//  will use as an indication when the trigger is active, the socket spins accordingly.
//
//	An LED will be used for status, socket spinning/not spinning, possibly power good, and
//	other indications.



//---------------------Specifications
//  PIC16F1825, 14 pin TSSOP package
//  Fosc, Clock frequency = 16MHz
//  Tcy, Instruction cycle time = 1/(16MHz/4) = 250ns
//  Vcc = 5.0VDC



// C O N F I G U R A T I O N   B I T S------------------------------------------

// !!!!!!!!TURN MCLRE ON WHEN DEBUGGING

// CONFIG1
#pragma config FOSC = INTOSC    // Oscillator Selection (INTOSC oscillator: I/O function on CLKIN pin)
#pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled)
#pragma config MCLRE = OFF      // MCLR Pin Function Select (MCLR/VPP pin function is digital input)
#pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Memory Code Protection (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled)
#pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin)
#pragma config IESO = OFF       // Internal/External Switchover (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)
#pragma config PLLEN = OFF      // PLL, 4x PLL disabled
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config LVP = OFF        // Low-Voltage Programming Enable (LVP = OFF, MCLR pin is a digital input, High-voltage on MCLR/VPP must be used for programming)



// I N C L U D E S -------------------------------------------------------------
#include <xc.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <pic16f1825.h>
#include "OpticalSpeedSensor.h"
#include "MacroLib.h"




void main (void)
{

    // set up oscillator control register
    OSCCONbits.SPLLEN=0x00;     // 4xPLL is disabled
    OSCCONbits.SCS=0x02;        // Set the SCS bits to select oscillator selected by OSCCON:IRCF bits
    OSCCONbits.IRCF=0x0F;       // Set OSCCON IRCF bits to select OSC frequency = 16MHz(0x0F)
    

    // Set up I/O pins
    // All weak pull up resistors are enabled by individual WPUx latch values. (see OPTION_REG, spec. pg 176)
    OPTION_REGbits.nWPUEN = 0;  // 

    //------------------------- SETUP PORTA

    // Setup PORTA Analog pins as Digital channels
    ANSELAbits.ANSA0 = 0;       // RA0 = Digital
    ANSELAbits.ANSA1 = 0;       // RA1 = Digital
    ANSELAbits.ANSA2 = 0;       // RA2 = Digital
    ANSELAbits.ANSA4 = 0;       // RA4 = Digital

    WPUA = 0x00;                // All PORTA pull up resistors disabled
                                //WPUAbits.WPUA0 = 0;         // RA0 pull up disabled
                                //WPUAbits.WPUA1 = 0;         // RA1 pull up disabled
                                //WPUAbits.WPUA2 = 0;         // RA2 pull up disabled
                                //WPUAbits.WPUA3 = 0;         // RA3 pull up disabled
                                //WPUAbits.WPUA4 = 0;         // RA4 pull up disabled
                                //WPUAbits.WPUA5 = 0;         // RA5 pull up disabled


    TRISAbits.TRISA0 = 0;       // RA0 = (not used) set as output and ground it. (ICSPDAT)
    LATAbits.LATA0 = 0;         // RA0 = output and grounded, NOT USED

    TRISAbits.TRISA1 = 0;       // RA1 = (not used) set as output and ground it. (ICSPCLK)
    LATAbits.LATA1 = 0;         // RA1 = output and grounded, NOT USED

    TRISAbits.TRISA2 = 1;       // RA2 = Optical sensor signal input
                                // RA2 = External INT, No light = pin low, Light = pin high

    TRISAbits.TRISA3 = 1;       // RA3 = (MCLR),(not used as MCLR function) set as digital input pulled up with external 10K

    TRISAbits.TRISA4 = 0;       // RA4 = Status LED output, set as output
    STAT_LED_OFF;               // Turn off Status LED

    TRISAbits.TRISA5 = 0;       // RA5 = Sensor status output, set as output
    SENS_STAT_OFF;              // Turn off sensor status to the PLC


    

    //------------------------- SETUP PORTC

    // Setup PORTC Analog pins to Digital channels
    ANSELCbits.ANSC0 = 0;       // RC0 = Digital
    ANSELCbits.ANSC1 = 0;       // RC1 = Digital
    ANSELCbits.ANSC2 = 0;       // RC2 = Digital
    ANSELCbits.ANSC3 = 0;       // RC3 = Digital
    
    WPUC = 0x00;                // All PORTC pull up resistors disabled
                                // WPUCbits.WPUC0 = 0;         // RC0 pull up disabled
                                // WPUCbits.WPUC1 = 0;         // RC1 pull up disabled
                                // WPUCbits.WPUC2 = 0;         // RC2 pull up disabled
                                // WPUCbits.WPUC3 = 0;         // RC3 pull up disabled
                                // WPUCbits.WPUC4 = 0;         // RC4 pull up disabled
                                // WPUCbits.WPUC5 = 0;         // RC5 pull up disabled
                                


    TRISCbits.TRISC0 = 0;       // RC0 = (not used) set as output and ground it. 
    LATCbits.LATC0 = 0;         // RC0 = output and grounded, NOT USED
                                // (If used as SPI SCK, configure as an output)
    
    TRISCbits.TRISC1 = 0;       // RC1 = (not used) set as output and ground it. 
    LATCbits.LATC1 = 0;         // RC1 = output and grounded, NOT USED
                                // (If used as SPI SDI, configure as an input and enable weak pull up on RC1)

    TRISCbits.TRISC2 = 0;       // RC2 = (not used) set as output and ground it. 
    LATCbits.LATC2 = 0;         // RC2 = output and grounded, NOT USED
                                // (If used as SPI SDO, configure as an output)

    TRISCbits.TRISC3 = 1;       // RC3 = TRG_STAT, PLC Tool Trigger status input
                                // ( If used SPI SS configure as output and pull high when not writing/reading from slave)
    
    TRISCbits.TRISC4 = 0;       // RC4 = (not used) set as output and ground it. (If used as TX configure as output and)
    LATCbits.LATC4 = 0;         // RC4 = output and grounded, NOT USED
                                // (If used as TX configure as output and make sure there is a 10K pull up in this pin)
    
    TRISCbits.TRISC5 = 0;       // RC5 = (not used) set as output and ground it. (If used as RX configure as input)
    LATCbits.LATC5 = 0;         // RC5 = output and grounded, NOT USED
                                // (If used as RX configure as input and make sure there is a 10K pull up in this pin)
    
    // These pins are not used in this application but if used as TX/RX functions these have to be set this way
    // APFCON0bits.TXCKSEL = 0;    // TX function on pin RC4
    // APFCON0bits.RXDTSEL = 0;    // RX function on pin RC5
    
        
    // Turn off DAC (Not Used)
    DACCON0 = 0;                // Disable and turn off DAC



    // Setup Timer1
    TMR1_Init();
        
    // Enable RS232 Transmit/Receive Port
    // RS232_Init();

    // Setup SPI as a Master Port
    // SPI_MasterInit();    
    
    // Setup SPI as a Slave Port
    // SPI_SlaveInit();

    Enable_Int();  //Enables Interrupts
    
    Enable_Timer1_Int();  //Enable Interrupts from timer overflow
    
    // Enable UART Receive Int
    // Enable_UART_RX_Int();
    
    // Enable INTS
    Enable_External_Rising_Int();               // Enable optical sensor external rising edge interrupt
    
    sCurrentState = RESETSTATE;                   // Set up starting state

    trigStatDebug = 0;
    // This is how you would check if tool trigger is pulled
    if(TRG_STATUS == 0)
    {
        // Tool trigger is pulled
    }
    
    
    
    while(1)
    {
        NOP();
        //--------------SENSOR STATE MACHINE
        switch(sCurrentState)
        {
            // Power up
            case RESETSTATE:       
                //check if the output is on, and change the output to off
                if (CHKTRUE(bFlags_1, SENS_STAT)){
                    SENS_STAT_OFF;
                    RESETBIT(bFlags_1, SENS_STAT);
                }
                
                RESETBIT(bFlags_1,BSLEDBLINK);          // Reset blink LED flag
                SETBIT(bFlags_1,LEDONOFF);          // Initialize LED flag to on
                STAT_LED_ON;                            //Turn On LED
                
                
                if (TRG_STATUS)
                {
                    sLastState = RESETSTATE;              // Set last state
                    sCurrentState = BEGIN_COUNT;         // Set current state
                }
                else
                {
                    sLastState = RESETSTATE;              // Set last state
                    sCurrentState = NORMAL_WAIT;       // Set current state
                }
            break;


            // Timers and Counters are reset
            case BEGIN_COUNT:
                //check if the output is on, and change the output to off
                if (CHKTRUE(bFlags_1, SENS_STAT)){
                    SENS_STAT_OFF;
                    RESETBIT(bFlags_1, SENS_STAT);
                }
                
                // Re-Load Timer1
                TMR1H = TIMER1HVAL;
                TMR1L = TIMER1LVAL;

                bRisingEdges = 0;   //reset edge counter
                sGenTimer1 = 0;     //reset clock
                
                sLastState = BEGIN_COUNT;          // Set last state
                sCurrentState = COUNTING;    // Set current state
            
            break;


            // Count edges with ISR, time duration of state
            case COUNTING:
                //check if the output is on, and change the output to off
                if (CHKTRUE(bFlags_1, SENS_STAT)){
                    SENS_STAT_OFF;
                    RESETBIT(bFlags_1, SENS_STAT);
                }
                //continue timer and count edges, both handled by ISR

                if (!TRG_STATUS) //if enable falls low, move to next state
                {
                    sLastState = COUNTING;       // Set last state
                    sCurrentState = EVALUATE;    // Set current state
                }
                else if (sGenTimer1 >= TIMER_LIMIT) //if timer exceeds set limit, move to error state
                {
                    sLastState = COUNTING;       // Set last state
                    sCurrentState = TIMEOUT_ERROR;    // Set current state
                }
                
            break;



            // Compare edge count and trigger duration
            case EVALUATE:
                //check if the output is on, and change the output to off
                if (CHKTRUE(bFlags_1, SENS_STAT)){
                    SENS_STAT_OFF;
                    RESETBIT(bFlags_1, SENS_STAT);
                }
                float sGenTimer1InSec = sGenTimer1/(float)1000; // convert from ms to s
                if (bRisingEdges/sGenTimer1InSec >= EDGE_TIME_MIN_RATIO)
                {
                    sLastState = EVALUATE;       // Set last state
                    sCurrentState = NORMAL_WAIT;    // Set current state
                }
                else // error detected
                {
                    sLastState = EVALUATE;       // Set last state
                    sCurrentState = ERROR_WAIT;    // Set current state
                }
                

            break;



            // Signal that the cycle was within parameters, wait for next enable
            case NORMAL_WAIT:
                //check if the output is on, and change the output to off
                if (CHKTRUE(bFlags_1, SENS_STAT)){
                    SENS_STAT_OFF;
                    RESETBIT(bFlags_1, SENS_STAT);
                }
                if (TRG_STATUS)
                {
                    sLastState = NORMAL_WAIT;              // Set last state
                    sCurrentState = BEGIN_COUNT;         // Set current state
                }
                

            break;


            // Signal that the cycle was outside parameters, wait for next enable
            case ERROR_WAIT:
                //check if the output is off, and change the output to on
                if (CHKFALSE(bFlags_1, SENS_STAT)){
                    SENS_STAT_ON;
                    SETBIT(bFlags_1, SENS_STAT);
                }
                if (TRG_STATUS)
                {
                    sLastState = ERROR_WAIT;              // Set last state
                    sCurrentState = BEGIN_COUNT;         // Set current state
                }
                
            break;
            
            // Signal that the cycle was enabled for too long, discard cycle
            case TIMEOUT_ERROR:
                //check if the output is off, and change the output to on
                if (CHKFALSE(bFlags_1, SENS_STAT)){
                    SENS_STAT_ON;
                    SETBIT(bFlags_1, SENS_STAT);
                }
                if (!TRG_STATUS)
                {
                    sLastState = TIMEOUT_ERROR;              // Set last state
                    sCurrentState = ERROR_WAIT;         // Set current state
                }
                
            break;


            default:    // Should never get here....but never say never
                //check if the output is off, and change the output to on
                if (CHKFALSE(bFlags_1, SENS_STAT)){
                    SENS_STAT_ON;
                    SETBIT(bFlags_1, SENS_STAT);
                }
                
                SETBIT(bFlags_1,BSLEDBLINK);          // Set BAD_STATE blink LED flag
                RESETBIT(bFlags_1,LEDONOFF);          // Initialize LED flag to off
                
                for(int ii = 0; ii > 3; ii++){
                    STAT_LED_OFF;
                    sGenTimer1 = 0;     //reset clock
                    while(sGenTimer1 < 500){} //wait 500 ms
                    STAT_LED_ON;
                    sGenTimer1 = 0;     //reset clock
                    while(sGenTimer1 < 500){} //wait 500 ms
                }
                
                sCurrentState = RESETSTATE;         // Set current state
            break;  // Normally after (default:) program would come out of switch
        }
        //--------------END SPI STATE MACHINE
    }




     
    

    

    

        

    
}











//------------------------------------------------------------------------------
// All Interrupt(s) for the PIC16F1825 vector to (0x0004)
//
//------------------------INTERRUPT SERVICE ROUTINE
//
//------------------------------------------------------------------------------
void __interrupt() ISR_Routine(void)
{
    
    __asm("NOP");
    //----------------------Check if Timer1 overflow occurred
    if (PIR1bits.TMR1IF)                            
    { //Overflow occurred
        
        __asm("NOP");                             // Adjust with NOPs if you need to
        
        // The time to get to this point should be 1ms if that was the time to overflow Timer1
        // The interrupt latency time...is the time from when the int event occurs to the time code
        // execution at the interrupt vector begins
        //  Asynchronous interrupts have 3 to 5 instruction cycles
        //  Synchronous interrupts have 3 to 4 instruction cycles before the 
        
        // Re-Load Timer1
        TMR1H = TIMER1HVAL;
        TMR1L = TIMER1LVAL;
        
        // Check if General timers are running and decrement
        /*
        if(sGenTimer1)
            sGenTimer1--;
            
        if(sGenTimer2)
            sGenTimer2--;
        */
        
        //increment timers
        sGenTimer1++;
        sGenTimer2++;
               
        PIR1bits.TMR1IF = 0;                        // Clear timer1 int flag
    }

       
        
    
    //----------------------Check if optical sensor had a rising edge occur
    
    if (INTCONbits.INTF)
    { // Optical sensor had rising edge
        
        // Could delay a short time to eliminate runt pulses
        __delay_ms(10);
        
        __asm("NOP");
        
        // Check if pulse is still high
        if(SENSOR_IN) 
        { // Pulse still high            
            bRisingEdges++;                 // Increment a rising edge counter
            
            SETBIT(bFlags_1,EDGEHIGH);      // Set pulse is still high flag
            
        }
        
        // I don't like to sit and spin in the ISR waiting so.......
        // We could leave the INT routine and go back to the WaitForPulseLow state in the runtime
        // routine and then check the pulse until it goes low and then move to the CheckRange
        // state were we check if the number of pulses are in range for a full count and if the
        // trigger is still on or has been released.
            
        INTCONbits.INTF = 0;        // Clear external interrupt flag
    }
    
    
    //  Check if received a data byte from RS232 port
    //if (PIR1bits.RCIF)                    // Check if received a data byte from RS232 port
    //{ // Received a data byte
    
           // Do something with UART received data
    //}
    
}




//------------------------------------------------------------------------------
//                          F U N C T I O N S
//------------------------------------------------------------------------------


void TMR1_Init(void)
{   // Setup Timer 1 to interrupt every 1ms
    // Fosc = 16MHz, instr = 16MHz/4 = 4MHz (250ns))

    // Set up the Timer1 Control Register (T1CON)
    T1CONbits.TMR1CS = 0;       // Timer 1 source is instruction clock (Fosc/4)= 250ns
    T1CONbits.T1CKPS = 2;       // 1ms, Set prescale 1:4 (250ns * 4 = 1us)
    T1CONbits.T1OSCEN = 0;      // Dedicated Timer1 oscillator circuit disabled
    T1CONbits.nT1SYNC = 1;      // Do not synchronize external clock input
    T1CONbits.TMR1ON = 1;       // Timer 1 on

    // Disable Timer1 Gate Function, use Timer1 as an overflow counter
    T1GCON = 0;                 // Timer1 gate function disabled

    // Setup Timer1 to overflow in 1ms and interrupt
    // Time unit = 1us (prescaled value)
    // 1ms / 1us = 1000
    // 65536 - 1000 = 64536(FC18h)
    // Load Timer, TMR1H = 0xFC and TMR1L = 0x18
    // TMR1H = TIMER1HVAL;
    // TMR1L = TIMER1LVAL;

    // Setup Timer1 to overflow in 100us and interrupt
    // Time unit = 250ns ( no prescale value)
    // 100us / 250ns = 400
    // 65536 - 400 = 64136(FE70h)
    // Load Timer, TMR1H = 0xFE and TMR1L = 0x70
    // TMR1H = TIMER1HVAL;
    // TMR1L = TIMER1LVAL;

    PIR1bits.TMR1IF = 0;                        // Clear timer1 int flag
}



void ADC_Init(void)
{ // Initialize the Analog-To-Digital converter.
    //Make sure the input pin is enabled as an analog input
    
    ADCON1bits.ADFM = 1;        // Right justify ADC result
    ADCON1bits.ADCS = 0x02;     // Select ADC conversion clock to, Fosc/32 = 500KHz (2us),(ADCS = 0x02)
    ADCON1bits.ADPREF = 0x00;   // Select ADC voltage reference as VCC.
    
    // The TAD minimum for the PIC16F1829 is 1.0us. per spec sheet, pg.354
    // At 16MHz clock (Fosc), selecting ADCS = FOSC/32 = 500kHz.  One clock period
    // 1 / 500kHz = 2us, which greater than minimum required 1.0us.
    // The total conversion will take 12*TAD = 12 * 2uS = 24uS per 10 bit sample
    //
    // The Tacq minimum acquisition time should take into account the internal
    // acquisition time TACQ of the ADC, and the settling time of of the
    // application circuit connected to the ADC pin.
    //
    // The input impedance (RS of this circuit is 470ohm. Temp operating = 50C
    // Chold = 12.5pF, RIC = <=1K, RSS @3.3VDD = 9.5K, RS = 470ohm
    // Tconversion  = -Chold * (RIC + RSS + RS) * ln(1/2047))
    //              = -12.5pF * (1K + 9.5K + 470ohm) * ln(1/2047)
    //              = -12.pF * (10.97K) * (-7.6241305)
    //              = 1.00364us
    //
    // Tacquisition = 1us + 1.00364us + [(50C-25C)*(0.05us/C)]
    //              = 1us + 1.00364us + [1.25us]
    //              = 3.254us
    //
    // So may need to delay 7us to 8us after turning on ADC to get better ADC values.

}

void SPI_MasterInit(void)
{   // Initialize the SPI Port for Master Mode (Use Mode 1, CKP=0,CKE=0)
    // Clock Idles low and SSP1STATbits.CKE = 0 (xmit occurs on transition from idle to active clock state)
    // 
    
    //-----------------------SSPSTAT REGISTER

    // Master SPI mode only
    //#define   SMPEND        0b10000000            // Input data sample at end of data out             
    //#define   SMPMID        0b00000000            // Input data sample at middle of data out

     // Mode Reference
    //#define   MODE_00       0b00000000            // Setting for SPI bus Mode 0,0
    //CKE           0x40                            // SSPSTAT register 
    //CKP           0x00                            // SSPCON1 register 

    //#define   MODE_01       0b00000001            // Setting for SPI bus Mode 0,1
    //CKE           0x00                            // SSPSTAT register 
    //CKP           0x00                            // SSPCON1 register

    //#define   MODE_10       0b00000010            // Setting for SPI bus Mode 1,0
    //CKE           0x40                            // SSPSTAT register
    //CKP           0x10                            // SSPCON1 register

    //#define   MODE_11       0b00000011            // Setting for SPI bus Mode 1,1
    //CKE           0x00                            // SSPSTAT register
    //CKP           0x10                            // SSPCON1 register
    
    // Set SPI mode to 1 (CKP=0,CKE=0) Master Mode, Clock Speed = 32MHz /64 = 500KHz
    
    SSP1STAT = 0b00000000;                          // Full port setting
                                                    // Bit7, SMP, 0 = Input data sampled at the middle of data output time, 1 = Input data sampled at the end of data output time  
                                                    // Bit6, CKE, 0 = Transmit occurs on transition from idle to active clock state, data captured on clocks falling edge
                                                    // Bit5-1, Don't Care
                                                    // Bit0, Read Only, 0 = Buffer Empty, 1 = Buffer full
                                            
    SSP1CON1 = 0b00100010;                          // Full port setting
                                                    // Bit7, WCOL, 0 = No collision, 1 = collision occurred, NOT USED IN SPI MASTER MODE 
                                                    // Bit6, SSPOV, 0 = no overflow, 1 = overflow, NOT USED IN SPI MASTER MODE
                                                    // Bit5, SSPEN, 1 = Enable SPI port pins. For master mode SCK=output, SDI=input, SDO=output, SS=output
                                                    // Bit4, CKP, Clock polarity is low, 0 = Clock idle is low, 1 = Clock idle is high
                                                    // Bit 0-3, SSPM, 0010 = Enable master mode with Fosc/64
}


void SPI_SlaveInit(void)
{   // Initialize the SPI Port for Slave Mode (Use Mode 1, CKP=0,CKE=1)
    // Clock Idles low and SSP1STATbits.CKE = 0 (xmit occurs on transition from idle to active clock state)
    // Make sure to Keep SSP1STATbits.SMP clear in Slave mode

    //-----------------------SSPSTAT REGISTER

    // Master SPI mode only
    //#define   SMPEND        0b10000000            // Input data sample at end of data out
    //#define   SMPMID        0b00000000            // Input data sample at middle of data out


    // Mode Reference
    // NOTE: These settings are for PIC (other uC indicate CPOL(CKP) and CPHA(CKE))
    // CPOL(CKP) is the clocks idle state, CPHA(CKE) is the clock phase with respect to when the data is captured/changes
    //#define   MODE_0        0b00000000            // Setting for SPI bus Mode 0
    //CKE = 1  (CPHA = 0)                           // SSPSTAT register
    //CKP = 0  (CPOL = 0)                           // SSPCON1 register

    //#define   MODE_1        0b00000001            // Setting for SPI bus Mode 1
    //CKE = 0  (CPHA = 1)                           // SSPSTAT register
    //CKP = 0  (CPOL = 0)                           // SSPCON1 register

    //#define   MODE_2        0b00000010            // Setting for SPI bus Mode  2
    //CKE = 1  (CPHA = 0)                           // SSPSTAT register
    //CKP = 1  (CPOL = 1)                           // SSPCON1 register

    //#define   MODE_3        0b00000011            // Setting for SPI bus Mode  3
    //CKE = 0  (CPHA = 1)                           // SSPSTAT register
    //CKP = 1  (CPOL = 1)                           // SSPCON1 register

    // Set SPI Slave Mode 0, (CKE = 1, CKP = 0),  Maximum input clock speed at 32MHz Xtal, (Tcy + 20ns) Max. clock speed = (Tcy = 32MHz/4 = 8MHz = (125ns + 20ns * 2 = 3.44MHz)

    SSP1STAT = 0b01000000;                          // Full port setting
                                                    // Bit7, SMP, Make sure this bit is clear when in slave mode
                                                    // Bit6, CKE, 0 = Transition from idle to active clock state, 1 = Transition from active to idle clock state
                                                    // Bit5-1, Don't Care
                                                    // Bit0, Read Only, 0 = Buffer Empty, 1 = Buffer full

    SSP1CON1 = 0b00100100;                          // Full port setting
                                                    // Bit7, WCOL, 0 = No collision, 1 = collision occurred
                                                    // Bit6, SSPOV, 0 = no overflow, 1 = overflow, (In SPI slave mode user must read SSP1BUF, even if only transmitting data to avoid overflow bit being set)
                                                    // Bit5, SSPEN, 1 = Enable SPI port pins. For slave mode SCK=input, SDI=input, SDO=output, SS=input,  0 = disable SPI pins
                                                    // Bit4, CKP, Clock polarity, 0 = Clock idle is low, 1 = Clock idle is high
                                                    // Bit 0-3, SSPM, 0100 = Enable slave mode with SS pin control
}


void RS232_Init(void)
{   // Use Baud Rate of 38400

    BAUDCON = 0x00;
                                //  BAUDCONbits.ABDOVF = 0;     // Bit 7, Auto-Baud detect overflow did not overflow
                                //  BAUDCONbits.RCIDL = 1;      // Bit 6, Receiver is idle, (don't care, read only)
                                                                // Bit 5, Unimplemented
                                //  BAUDCONbits.SCKP = 0;       // Bit 4, Transmit non-inverted data to the TX/CK pin
                                //  BAUDCONbits.BRG16 = 0;      // Bit 3, Select 8bit Async mode, 8 bit Baud Rate generator formula = Fosc/(64*(n+1))
                                                                // Bit 2, Unimplemented
                                //  BAUDCONbits.WUE = 0;        // Bit 1, Wake-up receiver is disabled, normal operation is enabled
                                //  BAUDCONbits.ABDEN = 0;      // Bit 0, Auto-Baud detect is disabled

    TXSTA = 0x20;
                                //  TXSTAbits.CSRC = 0;         // Bit 7, Clock source, (don't care in Async mode))
                                //  TXSTAbits.TX9 = 0;          // Bit 6, Select 8bit transmission
                                //  TXSTAbits.TXEN = 1;         // Bit 5, Transmit enable
                                //  TXSTAbits.SYNC = 0;         // Bit 4, Async mode
                                //  TXSTAbits.SENDB = 0;        // Bit 3, Sync break transmission completed
                                //  TXSTAbits.BRGH = 0;         // Bit 2, Low speed baud rate
                                //  TXSTAbits.TRMT = 0;         // Bit 1, Don't Care (read only bit)
                                //  TXSTAbits.TX9D = 0;         // Bit 0, Not using 9 bit so don't care

    RCSTA = 0x90;
                                //  RCSTAbits.SPEN = 1;         // Bit 7, Enable serial port pins TX and RX as USART
                                //  RCSTAbits.RX9 = 0;          // Bit 6, Select 8bit reception
                                //  RCSTAbits.SREN = 0;         // Bit 5, Single receive (don't care because using Async mode)
                                //  RCSTAbits.CREN = 1;         // Bit 4, Enable continuous receive
                                //  RCSTAbits.ADDEN = 0;        // Bit 3, Disable address detection
                                //  RCSTAbits.FERR = 0;         // Bit 2, Framing error, (don't care, read only)
                                //  RCSTAbits.OERR = 0;         // Bit 1, Overrun error, (don't care, read only)
                                //  RCSTAbits.RX9D = 0;         // Bit 0, 9th bit of received data, (don't care, not using 9 bit receive)

                                // Select Baud Rate: Fosc = 32MHz, Baud Rate = 38400, SPBRGH:L = ((32MHz / 38400) / 64) - 1 = 12
                                // Calculated Baud Rate = 32Mhz / (64 * (SPBRGH:L + 1)) = 32MHz / (64 * (12 + 1)) = 38461.5
                                // Percent Error = ((38461.5 - 38400) / 38400) * 100 = 0.16% error
    SPBRGH = 0;                 // Baud Rate = 9600;   SPBRGH 0; SPBRGL 51;
    SPBRGL = 12;                // Baud Rate = 38400;  SPBRGH 0; SPBRGL 12;
                                // Baud Rate = 500K;   SPBRGH 0; SPBRGL 0;


}


void UART_Write( unsigned short data)
{   // Write data to RS232 Port
                                    // !!!!!!CODE BLOCK COMING....HOULD HAVE WATCHDOG RUNNING IN CASE THE TRMT NEVER RESETS
    while(!TXSTAbits.TRMT);         // Check if shift register is empty before sending data.
    TXREG = data;
}


unsigned char UART_ChkRec_Err()
{   // Check if any receive data errors, 0 = no errors,
    // return 1 = framing error
    // return 2 = overrun error
    // This routine will clear all receive errors if any

    unsigned char tmpdata;

    if(RCSTAbits.FERR)
    {   // Framing Error (stop bit not identified at expected time)
        tmpdata = RCREG;                    // Read received data
        return(1);
    }
    else if(RCSTAbits.OERR)
    {   // Overrun error (If a 3rd character is received before the FIFO is accessed.)

        RCSTAbits.CREN = 0;                 // Clear overrun error
        RCSTAbits.CREN = 1;                 // Reset continuous receive function
        return(2);
    }
    else
    {   // No errors
        return(0);
    }
}

void Enable_Int(void)
{
    INTCONbits.GIE = 1;         //Enables interrupts globally
    INTCONbits.PEIE = 1;        //Enables active peripheral interrupts
}

void Disable_Int(void)
{
    INTCONbits.GIE = 0;         //Disables interrupts globally
    INTCONbits.PEIE = 0;        //Disables active peripheral interrupts
}

void Enable_Timer1_Int(void)
{   // Enable Timer 1 INTS
    PIR1bits.TMR1IF = 0;        // Clear timer1 int flag
    PIE1bits.TMR1IE = 1;        // Enable Timer 1 overflow int
}

void Disable_Timer1_Int(void)
{   // Disable Timer 1 INTS
    PIR1bits.TMR1IF = 0;        // Clear timer1 int flag
    PIE1bits.TMR1IE = 0;        // Disable Timer 1 overflow int    
}

void Enable_External_Rising_Int(void)
{   // Enable External INTS
    OPTION_REGbits.INTEDG = 1;  // Interrupt on the rising edge
    INTCONbits.INTF = 0;        // Clear external interrupt flag
    INTCONbits.INTE = 1;        // Enable the external interrupt
}

void Disable_External_Rising_Int(void)
{   // Disable External INTS
    INTCONbits.INTF = 0;        // Clear external interrupt flag
    INTCONbits.INTE = 0;        // Disable the external interrupt
}

void Enable_UART_RX_Int(void)
{   // Setup UART Receive INTS
    PIR1bits.RCIF = 0;          // Clear receive interrupt flag
    PIE1bits.RCIE = 1;          // Set receive interrupt
}



unsigned short Read_ADC_Value(void)
{   // Do ADC conversion and adjust to get 10bit AD value
    unsigned short ADCValue;
    char i;

    ADCON0bits.CHS = 0x04;                              // Select Channel AN4
    ADCON0bits.ADON = 1;                                // Turn ADC on
    for(i=0; i<2;i++)                                   // Delay for 8.0us for ADC acquisition time. (i<8)
    {                                                   // instr = 125ns, 8us/125ns = 64
        asm ("NOP");                                    // The loop takes 8us, adjusted for bloat in free compilier code
    }                                                   // !!!!When using the Pro version, adjust this loop to equal 8us
    ADCON0bits.GO = 1;                                  // start conversion
    while (ADCON0bits.GO);                              // wait for conversion to finish
    ADCValue = ADRESH << 8;                             // get the 2 msbs of the result and rotate 8 bits to the left
    bADCHigh = ADRESH;                                  // get 2 MSBs of 10 bit result (for RS232 output)
    bADCLow = ADRESL;                                   // get 8 LSBs of 10 bit result (for RS232 output)
    ADCValue = (unsigned short) ADCValue + ADRESL;      // now add the low 8 bits of the result into our return variable
    ADCON0bits.ADON = 0;                                // Turn ADC off
    return (ADCValue);                                  // return the 10bit result in a single variable
}






//----------------------------------Test Programs


// !!!!!!!!!!!!!!!!!!!!!!!!!TESTING SERIAL PORT
//
//    // Fill up data buffer with test bytes ASCII values of 0 through 9
//
//        pt = &gendata[0];               // Set pointer to start of data array
//        for(p=0;p<10;p++)
//        {
//           gendata[p] = p + 48;
//        }
//
//    sGenTimer2 = 2000;                      // Set 2s timer
//    while(1)
//    {
//        if(sGenTimer2 == 0)
//        {
//            GREEN_LED = ON;                 // Turn LED on when transmitting
//           pt = &gendata[0];               // Set pointer to start of data array
//
//            for(p=0;p<10;p++)
//            {
//                UART_Write(gendata[p]);     // Write data to the RS232 port
//                UART_Write(0x20);           // Send space character
//                pt++;
//			}
//           UART_Write(0x0A);               // Send line feed character
//            UART_Write(0x0D);               // Send carriage return character
//
//            sGenTimer2 = 2000;              // Set 2s timer
//            GREEN_LED = OFF;               // Turn LED off when transmitting
//        }
//        else
//        {
//            // Sit and spin until timer is done
//        }
//    }
//
//   // !!!!!!!!!!!!!!!!!!!!!!!!!!END TESTING

// while(1)
//    {
//        pt = &gendata[0];                   // Set pointer to start of data array
//
//        uiRS232ConVal = swDCFiltVal;        // Get filtered ADC value
//        itoa(gendata,uiRS232ConVal,10);     // Convert int to string, This takes about 1ms to convert
//
//        for(p=0;p<4;p++)
//            {
//                UART_Write(gendata[p]);     // Write data to the RS232 port (This is decimal value of ADC conversion)
//                pt++;                       // Could convert to actual voltage level (ADCVal * 3223)/1000
//			}
//        UART_Write(0x0D);                   // Send carriage return character
//   }



// Testing EEPROM READ/WRITE Times
    //sCurrentState= eeprom_read(0x00);   // Takes 5.25us for read (42 cycles)
    //sCurrentState = 0x45;
    //eeprom_write(0xFF, sCurrentState); // tAKES 8ms for byte write

//SETBIT(bFlags, ADCRUNNING);             // Start sampling the analog input
    //SETBIT(bFlags, LPFRUNNING);             // Start LPF
    //RESETBIT(bFlags, LPFRUNNING);           // Do not run LPF
    //bLPFSampTime = LPFSAMPLE_TIME;          // Load the number of LPF samples to take before determining valid ADC final value


// start in power up state
//    sCurrentState = POWER_UP;
//
//    while(1)
//    {
//        switch(sCurrentState)
//       {
//            case  POWER_UP:                         //------------POWER UP STATE
//                //bFlags = 0;                         // Reset all flags
//                sGenTimer1 = 0;                     // Reset general timers
//               sGenTimer2 = 0;                     //
//                //bLPFSampTime = LPFSAMPLE_TIME;      // Load the number of LPF samples to take before determining switch state valid
//                //swCurrentADCVal = 0;                // Reset Current ADC sample
//                //swDCFiltVal = 0;                    // Reset LPF filtered value
//                //SETBIT(bFlags, ADCRUNNING);         // Start sampling the analog input
//                //SETBIT(bFlags, LPFRUNNING);         // Start LPF
//               sCurrentState = LED_STATUS;         // Check LED status
//            break;
//
//            //------------LED STATUS
//            case  LED_STATUS:
//                if( (CHKTRUE(bFlags, LEDBLINK) && (sGenTimer1 == 0)) )
//                {
//                    if(CHKTRUE(bFlags, LEDONOFF))
//                    { // LED is ON
//                        GREEN_LED = OFF;            // Turn LEDs off
//                        YELLOW_LED = OFF;
//                        RED_LED = OFF;
//                        sGenTimer1 = LEDOFFTIME;    // Load LED off time
//                        RESETBIT(bFlags,LEDONOFF);
//                   }
//                    else
//                    { // LED is OFF
//                        GREEN_LED = ON;             // Turn LEDs on
//                        YELLOW_LED = ON;
//                        RED_LED = ON;
//                        sGenTimer1 = LEDONTIME;     // Load LED on time
//                        SETBIT(bFlags,LEDONOFF);
//                    }
//                }
//                else
//                {
//                 break;
//                }
//
//            default:                                //------------DEFAULT STATE
//                break;                              // should never get here
//                                                    // Normally after (default:) program would come out of switch
//        }
//    }