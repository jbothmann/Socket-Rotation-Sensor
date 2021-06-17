// MacroLib.h
//
//
#define CHKTRUE(bmem,flg)       (((bmem) & (flg)) != 0)
#define CHKFALSE(bmem,flg)      (((bmem) & (flg)) == 0)
#define SETBIT(bmem,flg)        ((bmem) |= (flg))
#define RESETBIT(bmem,flg)      ((bmem) &= ((~(flg))))
#define MC_XORBIT(bmem,flg)     ((bmem) ^= (flg))
#define MC_MASKBIT(bmem,flg)    ((bmem) &= (flg))
#define MC_ABSOLUTEX(mem)       (((mem) >= 0)? (mem) : -(mem))
#define MC_MINIMUM(x,y)         ( ( (x) < (y) )? (x):(y) )
#define MC_MAXIMUM(x,y)         ( ( (x) < (y) )? (y):(x) )
#define IS_EVEN(x)              ((x%2 == 0) ? 1 : 0)    // Approx 30 cycles
#define IS_ODD(x)               ((x%2 != 0) ? 1 : 0)    // Approx 129 cycles


// define the following in a header file separate of the MacroLib.h
//
// unsigned char tempFlags (8 bit register of flags)
// #define FLAG_1 					0x01
// #define FLAG_2					0x02
// #define FLAG_3					0x04
// #define FLAG_4 					0x08
// #define FLAG_5					0x10
// #define FLAG_6					0x20
// #define FLAG_7 					0x40
// #define FLAG_8					0x80
//
// Usage: CHKTRUE/CHKFALSE
//
// if(CHKTRUE(tempFlags, FLAG_1)	// Check if bit0 is != 0
// if(CHKFALSE(tempFlags, FLAG_8)	// Check if bit7 is = 0
//
// Usage: SETBIT/RESETBIT
//
// tempFlags = 0;  				// Reset all flags
// SETBIT(tempFlags, FLAG_3);	// Set bit2
// RESETBIT(tempFlags, FLAG_4); // Reset bit3
//
// Usage: (rarely used)
//
// MC_XORBIT/MC_MASKBIT (straight forward)
//
// Usage:
//
// MC_ABSOLUTEX(nAcFil_A - (nDcFil_A * nDcFil_A)) // Get the absolute value of the equation

// Usage:
//
// MC_MINIMUM(Temp_Amps, 65535) // Check if Temp_Amps is smaller than 65535,
//								// or used as overflow protection, will return 65535 if Temp_Amps is 65535.5
//
// wMaxRmsVal = MC_MAXIMUM((USHORT)nRmsout_A,(USHORT)nRmsout_B);
// Return the maximum value of the 2 (nRmsout_A and nRmsout_B)


// Convert value from decimal to ASCII
    //bTempChar = 32;
    //ASCdigit[0] = (bTempChar/100) | 0x30;
    //bTempChar = (unsigned char)bTempChar % 100;
    //ASCdigit[1] = (bTempChar/10)| 0x30;
    //ASCdigit[2] = (bTempChar%10) | 0x30;

/*
    
    td = &tooldata[48];                  // Set pointer to start of data array
    //for(t=0; t < TLDATAMAX; t++)
    for(t=48; t < 58; t++)
    {
        // Convert value from decimal to ASCII
        bTempChar = tooldata[t];
        ASCdigit[0] = bTempChar/100;
        bTempChar = bTempChar % 100;
        ASCdigit[1] = bTempChar/10;
        ASCdigit[2] = bTempChar%10;
        
        for(d=0;d<3;d++)
        {
            if((t < 10) && (d < 2) )
            {
                ASCdigit[d] = 0x20;  // Insert SPACE 
            }
            else if( (t >= 10 && t <= 99) && (d < 1) )
            {
                ASCdigit[d] = 0x20;  // Insert SPACE 
            }
            else
            {
                ASCdigit[d] |= 0x30;   
            }         
        }       
        
        UART_Write(tooldata[t]);
        UART_Write(0x0A);
        UART_Write(0x0D);
        td++;
    }
 */
