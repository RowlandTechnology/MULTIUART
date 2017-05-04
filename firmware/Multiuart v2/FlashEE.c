/*
 * File:   FlashEE.c
 * Author: Ben
 *
 * Created on 02 May 2017, 22:47
 */

#include <xc.h>

#include "../support/generic/h/libpic30.h"

#include "FlashEE.h"

#define FlashEEFALSE (0)
#define FlashEEMAXCELL (253)
#define FlashEETRUE (1)

#define FlashEEPageAddress 64

volatile unsigned char FlashBuffer[0x0800] __attribute__((space(prog),aligned(0x0800), noload)); 
volatile unsigned char FlashEEPAGE;
volatile unsigned int FlashEECOUNT = (0x0); // Counter for page use
volatile unsigned int FlashEEINITIALIZED = (0x0); // Has the component been initialized?
volatile unsigned char FlashEECELL = (0x0);


void FlashEEConsolidatePage();


void FlashEEClearPage(unsigned char FCL_PAGENUMBER)
{
    //Local variable definitions
    unsigned long FCL_MEMADDRESS;

    FCL_MEMADDRESS = FlashEEPageAddress + (512 * FCL_PAGENUMBER);

    #if (0)

    //Code has been optimised out by the pre-processor
    #else

        {
        // Calculate address and set registers
        unsigned char savetbl = TBLPAG;
        unsigned int addr = (FCL_MEMADDRESS & 0xffff);
        unsigned int savesr;

        TBLPAG = ((FCL_MEMADDRESS & 0x7f0000) >> 16);

        // Write dummy value
        __builtin_tblwtl(addr, 0x0000);

        // Save interrupt register and disable interrupts
        savesr = SR;
        SR = SR | 0xE0;

        // Set programming mode to WORD programming
        NVMCON = _FLASH_ERASE_CODE;

        // Write to flash
        __builtin_write_NVM();

        // Reenable interrupts
        SR = savesr;

        // Wait for operation to complete
        while (NVMCON & 0x8000) ;

        // restore TBLPAG
        TBLPAG = savetbl;
        }

    #endif

}

/*=----------------------------------------------------------------------=*\
   Use :Read address and data from specified page and cell position
       :
       :Parameters for macro ReadCell:
       :  page : unsigned char
       :  cell : unsigned char
       :
       :Returns : unsigned long
\*=----------------------------------------------------------------------=*/
unsigned long FlashEEReadCell(unsigned char FCL_PAGE, unsigned char FCL_CELL)
{
    //Local variable definitions
    unsigned long FCL_MEMADDRESS;
    unsigned long FCR_RETVAL;

    FCL_MEMADDRESS = FlashEEPageAddress + (512 * FCL_PAGE) + (2 * FCL_CELL);

    {
    // Calculate address and set registers
    unsigned char savetbl = TBLPAG;
    unsigned int addr = (FCL_MEMADDRESS & 0xffff);

    TBLPAG = ((FCL_MEMADDRESS & 0x7f0000) >> 16);

    // Read values
    FCR_RETVAL = __builtin_tblrdh(addr);
    FCR_RETVAL = FCR_RETVAL << 16;
    FCR_RETVAL |= __builtin_tblrdl(addr) & 0xffff;

    // restore TBLPAG
    TBLPAG = savetbl;
    }

    return (FCR_RETVAL);

}

/*=----------------------------------------------------------------------=*\
   Use :Write address and data to specified page and cell position
       :
       :Parameters for macro WriteCell:
       :  page : unsigned char
       :  cell : unsigned char
       :  address : unsigned char
       :  value : unsigned int
\*=----------------------------------------------------------------------=*/
void FlashEEWriteCell(unsigned char FCL_PAGE, unsigned char FCL_CELL, unsigned char FCL_ADDRESS, unsigned int FCL_VALUE)
{
    //Local variable definitions
    unsigned long FCL_MEMADDRESS;
    FCL_MEMADDRESS = FlashEEPageAddress + (512 * FCL_PAGE) + (2 * FCL_CELL);

    #if (0)

    //Code has been optimised out by the pre-processor
    #else

        {

        #ifndef _FLASH_WRITE_WORD_CODE
        #error "Controller not supported by FLASH EEPROM emulation"
        #endif

        // Calculate address and set registers
        unsigned char savetbl = TBLPAG;
        unsigned int addr = (FCL_MEMADDRESS & 0xffff);
        unsigned int savesr;

        TBLPAG = ((FCL_MEMADDRESS & 0x7f0000) >> 16);

        // Store values
        __builtin_tblwtl(addr, FCL_VALUE);
        __builtin_tblwth(addr, FCL_ADDRESS);

        // Save interrupt register and disable interrupts
        savesr = SR;
        SR = SR | 0xE0;

        // Set programming mode to WORD programming
        NVMCON = _FLASH_WRITE_WORD_CODE;

        // Write to flash
        __builtin_write_NVM();

        // Reenable interrupts
        SR = savesr;

        // Wait for operation to complete
        while (NVMCON & 0x8000) ;

        // restore TBLPAG
        TBLPAG = savetbl;
        }

    #endif

}


/*========================================================================*\
   Use :flasheeprom1
       :Macro implementations
\*========================================================================*/
/*=----------------------------------------------------------------------=*\
   Use :Clear the old page if at the last page of a block of flash. If not at the end, mark
       :the page expired so it can be cleared at a future call
       :
       :Parameters for macro Clear:
       :  page : unsigned char
\*=----------------------------------------------------------------------=*/
void FlashEEClear(unsigned char FCL_PAGE)
{
    //Local variable definitions
    unsigned int FCL_I;
    unsigned char FCL_ALLEXPIRED;
    unsigned long FCL_VALUE;


    if ((FCL_PAGE % 2) == (2 - 1))
    {

        FCL_I = (FCL_PAGE / 2) * 2;
        FCL_ALLEXPIRED = FlashEETRUE;

        while (FCL_I < FCL_PAGE && FCL_ALLEXPIRED)
        {

            FCL_VALUE = FlashEEReadCell(FCL_I, 0);

            if ((FCL_VALUE >> 16) == 0)
            {

                FCL_I = FCL_I + 1;

            } else {

                FCL_ALLEXPIRED = FlashEEFALSE;

            }


        }

        if (FCL_ALLEXPIRED)
        {

            FlashEEClearPage((FCL_PAGE / 2) * 2);

        } else {

            //Comment:
            //Error condition.

        }

    } else {

        FCL_I = FlashEEReadCell(FCL_PAGE, 0);

        FlashEEWriteCell(FCL_PAGE, 0, 00, FCL_I);

    }

}

/*=----------------------------------------------------------------------=*\
   Use :Return the 16 bit value stored at address.
       :
       :Parameters for macro Read:
       :  address : Address to read (<250)
       :
       :Returns : unsigned int
\*=----------------------------------------------------------------------=*/
unsigned int FlashEERead(unsigned char FCL_ADDRESS)
{
    //Local variable definitions
    unsigned char FCL_RETVAL = (0x0);
    unsigned char FCL_I;
    unsigned long FCL_RESULT;
    unsigned int FCR_RETVAL;


    if (FlashEEINITIALIZED != 0xD0D0)
    {

        FCL_RETVAL = FlashEEInitialise();

    // } else {

    }

    FCR_RETVAL = 0xffff;

    if (FCL_RETVAL == 0)
    {

        FCL_I = FlashEECELL - 1;
        FCL_RESULT = 0xffffff;

        while (FCL_I > 0 && ((FCL_RESULT >> 16) & 0xff) != FCL_ADDRESS)
        {

            FCL_RESULT = FlashEEReadCell(FlashEEPAGE, FCL_I);

            if (((FCL_RESULT >> 16) & 0xff) == FCL_ADDRESS)
            {

                FCR_RETVAL = FCL_RESULT & 0xffff;

            // } else {

            }

            FCL_I = FCL_I - 1;


        }

    // } else {

    }

    return (FCR_RETVAL);

}

/*=----------------------------------------------------------------------=*\
   Use :This macro unconditionaly clears all memory reserved for storage. All previously stored information will be LOST!
\*=----------------------------------------------------------------------=*/
void FlashEEClearAll()
{
    //Local variable definitions
    unsigned char FCL_I;


    FCL_I = 0;

    while (FCL_I < 4 * 2)
    {

        FlashEEClearPage(FCL_I);

        FCL_I = FCL_I + 2;


    }

}

/*=----------------------------------------------------------------------=*\
   Use :Write the 16 bit value to address. There are 250 valid memory locations. Returns 0 for success. Returns <> 0 for error.
       :
       :Parameters for macro Write:
       :  address : Address for write. Must be < 250
       :  value : value to write
       :
       :Returns : unsigned char
\*=----------------------------------------------------------------------=*/
unsigned char FlashEEWrite(unsigned char FCL_ADDRESS, unsigned int FCL_VALUE)
{
    //Local variable definitions
    unsigned char FCL_RETVAL = (0x0);
    unsigned int FCL_CURVAL = (0x0);
    unsigned char FCR_RETVAL;


    if (FlashEEINITIALIZED != 0xD0D0)
    {

        FCL_RETVAL = FlashEEInitialise();

    // } else {

    }

    FCR_RETVAL = 0;

    if (FCL_RETVAL == 0 && FCL_ADDRESS < FlashEEMAXCELL)
    {

        FCL_CURVAL = FlashEERead(FCL_ADDRESS);

        if (FCL_CURVAL != FCL_VALUE)
        {

            FlashEEWriteCell(FlashEEPAGE, FlashEECELL, FCL_ADDRESS, FCL_VALUE);

            FlashEECELL = FlashEECELL + 1;

            if (FlashEECELL > FlashEEMAXCELL)
            {

                FlashEEConsolidatePage();

            // } else {

            }

        // } else {

        }

    } else {

        FCR_RETVAL = -1;

    }

    return (FCR_RETVAL);

}

/*=----------------------------------------------------------------------=*\
   Use :Consolidate data stored on the current page onto a new page, omitting superseded entries. Clear the old page and update the globals to point to the new page and next free cell.
\*=----------------------------------------------------------------------=*/
void FlashEEConsolidatePage()
{
    //Local variable definitions
    unsigned char FCL_CURPAGE = (0x0);
    unsigned char FCL_NEWPAGE = (0x0);
    unsigned char FCL_CURINDEX = (0x0);
    unsigned char FCL_NEWINDEX = (0x0);
    unsigned char FCL_B = (0x0); // Cell index
    unsigned int FCL_VALUE;


    FCL_CURPAGE = FlashEEPAGE;
    FCL_CURINDEX = FlashEECELL;
    FCL_NEWPAGE = FCL_CURPAGE + 1;
    FCL_NEWINDEX = 1;

    if (FCL_NEWPAGE >= 4 * 2)
    {

        FCL_NEWPAGE = 0;

    // } else {

    }

    FCL_B = 0;

    while (FCL_B < FlashEEMAXCELL)
    {

        FCL_VALUE = FlashEERead(FCL_B);

        if (FCL_VALUE != 0xffff)
        {

            FlashEEWriteCell(FCL_NEWPAGE, FCL_NEWINDEX, FCL_B, FCL_VALUE);

            FCL_NEWINDEX = FCL_NEWINDEX + 1;

        // } else {

        }

        FCL_B = FCL_B + 1;


    }

    FlashEEWriteCell(FCL_NEWPAGE, 0, 0xf0, FlashEECOUNT + 1);

    FlashEEClear(FCL_CURPAGE);

    FlashEEPAGE = FCL_NEWPAGE;
    FlashEECELL = FCL_NEWINDEX;
    FlashEECOUNT = FlashEECOUNT + 1;

}

/*=----------------------------------------------------------------------=*\
   Use :Initialise the component. Will be called by Read/Write if not explicitly called, This macro consolidates the storage if required (due to unexpected power failure). If an unresolvable error condition is detected the return value will be <> 0.
       :
       :Returns : unsigned char
\*=----------------------------------------------------------------------=*/
unsigned char FlashEEInitialise()
{
    //Local variable definitions
    unsigned long FCL_RESULT;
    unsigned char FCL_I = (0x0);
    unsigned char FCL_ZEROPAGE;
    unsigned char FCR_RETVAL;


    //Comment:
    //Start by checking all pages to find the current valid one. Once found
    //check the other pages for data. If a page with data is found it needs
    //to be cleared and a new consolidation is required.

    FlashEEPAGE = 0;
    FCL_ZEROPAGE = 0;

    while (1)
    {

        FCL_RESULT = FlashEEReadCell(FlashEEPAGE, 0);

        if ((FCL_RESULT & 0xffffff) == 0)
        {

            FCL_ZEROPAGE = FCL_ZEROPAGE + 1;

        // } else {

        }

        if (FCL_RESULT == 0xffffff || (FCL_RESULT >> 16) == 0)
        {

            FlashEEPAGE = FlashEEPAGE + 1;

        } else {

            FlashEECOUNT = FCL_RESULT & 0xffff;

        }


        if ((FlashEEPAGE < (4 * 2) && (FCL_RESULT == 0xffffff || (FCL_RESULT >> 16) == 0)) == 0) break;
    }

    if (FCL_ZEROPAGE == 4 * 2)
    {

        FlashEEClearAll();

        FlashEEPAGE = 0;
        FlashEECOUNT = 1;

        FlashEEWriteCell(FlashEEPAGE, 0, 0xf0, FlashEECOUNT);

    } else {

        if (FlashEEPAGE < 4 * 2)
        {

            if (FlashEEPAGE == 0)
            {

                FCL_RESULT = FlashEEReadCell((4 * 2) - 1, 0);

                if (FCL_RESULT != 0xffffff)
                {

                    FlashEEClearPage((4 * 2) - 1);

                // } else {

                }

            // } else {

            }

            if (FlashEEPAGE < (4 * 2) - 2)
            {

                FCL_RESULT = FlashEEReadCell(FlashEEPAGE + 1, 0);

                if ((FCL_RESULT >> 16) == 0xf0)
                {

                    FlashEEClearPage(FlashEEPAGE);

                    FlashEEPAGE = FlashEEPAGE + 1;
                    FlashEECOUNT = FCL_RESULT & 0xffff;

                // } else {

                }

            // } else {

            }

        } else {

            FlashEEPAGE = 0;
            FlashEECOUNT = 1;

            FlashEEWriteCell(FlashEEPAGE, 0, 0xf0, FlashEECOUNT);

        }

    }

    //Comment:
    //To do: Make sure all other pages are cleared to prevent corruption issues

    //Comment:
    //Find the last used cell in the page

    FCL_I = 1;

    while (1)
    {

        FCL_RESULT = FlashEEReadCell(FlashEEPAGE, FCL_I);

        if (FCL_RESULT != 0xffffff)
        {

            FCL_I = FCL_I + 1;

        // } else {

        }


        if ((FCL_I < FlashEEMAXCELL && FCL_RESULT != 0xffffff) == 0) break;
    }

    FlashEEINITIALIZED = 0xD0D0;
    FlashEECELL = FCL_I;
    FCR_RETVAL = 0;

    return (FCR_RETVAL);

}

