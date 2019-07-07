/*
 * File:   FlashEE.c
 * Author: Ben
 *
 * Created on 02 May 2017, 22:47
 * 06 07 2019 - Updated code for v2 device
 */


#include <xc.h>
#include "p24fxxxx.h"

#define BASE_ADDRESS	0x0A000
#define ADDR_OFFSET		1536


void FlashEEWrite(char idx, unsigned int value)
{
	if (idx < 4)
	{
		// Name: Erase Page, Type: C Code:
		// C example using MPLAB XC16
		unsigned long progAddr = BASE_ADDRESS + (idx * ADDR_OFFSET); // Address of word to program
        unsigned int offset;
        //Set up pointer to the first memory location to be written
        TBLPAG = progAddr>>16;  // Initialize PM Page Boundary SFR
        offset = progAddr & 0xFFFF;  // Initialize lower word of address
        __builtin_tblwtl(offset, 0x0000);  // Set base address of erase block
        // with dummy latch write
        NVMCON = 0x4042;  // Initialize NVMCON
        asm("DISI #5");  // Block all interrupts with priority <7
        // for next 5 instructions
        __builtin_write_NVM();  // check function to perform unlock
        // sequence and set WR


		// Name: Write Words, Type: C Code:
		// C example using MPLAB XC16
		//unsigned long progAddr = 0x015F00; // Address of word to program
        unsigned int progDataL = value; // Data to program lower word
        unsigned char progDataH = 0x00; // Data to program upper byte
        //Set up NVMCON for word programming
        NVMCON = 0x4003; // Initialize NVMCON
        //Set up pointer to the first memory location to be written
        TBLPAG = progAddr>>16; // Initialize PM Page Boundary SFR
        offset = progAddr & 0xFFFF; // Initialize lower word of address
        //Perform TBLWT instructions to write latches
        __builtin_tblwtl(offset, progDataL); // Write to address low word
        __builtin_tblwth(offset, progDataH); // Write to upper byte
        asm("DISI #5"); // Block interrupts with priority <7
        // for next 5 instructions
        __builtin_write_NVM(); // C30 function to perform unlock
        // sequence and set WR

	}
}


unsigned int FlashEERead(char idx)
{
	unsigned int retval = 0xFFFF;

	if (idx < 4)
	{
   	    // Name: C Code, Type: C Code:
    	// Calculate address and set registers
        unsigned char savetbl = TBLPAG;
        unsigned int addr = (BASE_ADDRESS & 0xffff) + (idx * ADDR_OFFSET);

        TBLPAG = ((BASE_ADDRESS & 0x7f0000) >> 16);

        // Read values
        retval = __builtin_tblrdh(addr);
        retval = retval << 16;
        retval |= __builtin_tblrdl(addr) & 0xffff;

        // restore TBLPAG
        TBLPAG = savetbl;
	}

	return retval;
}