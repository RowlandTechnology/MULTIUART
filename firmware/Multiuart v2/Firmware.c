//Copyright: Rowland Technology
//Owner: Ben Rowland
//Creation Date: 30/04/17
//Target Chip: PIC24FJ128GA202

//Chip Config
#include <xc.h>

// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F      // Deep Sleep Watchdog Timer Postscale Select bits (1:68719476736 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select (DSWDT uses LPRC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep BOR Enable bit (DSBOR Enabled)
#pragma config DSWDTEN = OFF            // Deep Sleep Watchdog Timer Enable (DSWDT Disabled)
#pragma config DSSWEN = ON              // DSEN Bit Enable (Deep Sleep is controlled by the register bit DSEN)
#pragma config PLLDIV = PLL4X           // USB 96 MHz PLL Prescaler Select bits (4x PLL selected)
#pragma config I2C1SEL = DISABLE        // Alternate I2C1 enable bit (I2C1 uses SCL1 and SDA1 pins)
#pragma config IOL1WAY = OFF            // PPS IOLOCK Set Only Once Enable bit (The IOLOCK bit can be set and cleared using the unlock sequence)

// CONFIG3
#pragma config WPFP = WPFP127           // Write Protection Flash Page Segment Boundary (Page 127 (0x1FC00))
#pragma config SOSCSEL = OFF            // SOSC Selection bits (Digital (SCLKI) mode)
#pragma config WDTWIN = PS25_0          // Window Mode Watchdog Timer Window Width Select (Watch Dog Timer Window Width is 25 percent)
#pragma config PLLSS = PLL_FRC          // PLL Secondary Selection Configuration bit (PLL is fed by the on-chip Fast RC (FRC) oscillator)
#pragma config BOREN = ON               // Brown-out Reset Enable (Brown-out Reset Enable)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Disabled)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMD = NONE            // Primary Oscillator Select (Primary Oscillator Disabled)
#pragma config WDTCLK = LPRC            // WDT Clock Source Select bits (WDT uses LPRC)
#pragma config OSCIOFCN = ON            // OSCO Pin Configuration (OSCO/CLKO/RA3 functions as port I/O (RA3))
#pragma config FCKSM = CSDCMD           // Clock Switching and Fail-Safe Clock Monitor Configuration bits (Clock switching and Fail-Safe Clock Monitor are disabled)
#pragma config FNOSC = FRCPLL           // Initial Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))
#pragma config ALTCMPI = CxINC_RB       // Alternate Comparator Input bit (C1INC is on RB13, C2INC is on RB9 and C3INC is on RA0)
#pragma config WDTCMX = WDTCLK          // WDT Clock Source Select bits (WDT clock source is determined by the WDTCLK Configuration bits)
#pragma config IESO = ON                // Internal External Switchover (Enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler Select (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler Ratio Select (1:128)
#pragma config WINDIS = OFF             // Windowed WDT Disable (Standard Watchdog Timer)
#pragma config FWDTEN = OFF             // Watchdog Timer Enable (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config LPCFG = OFF              // Low power regulator control (Disabled - regardless of RETEN)
#pragma config GWRP = OFF               // General Segment Write Protect (Write to program memory allowed)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = OFF             // JTAG Port Enable (Disabled)

#define CLK_SPEED 32000000

extern void __delay32(unsigned long count);

#define delay_s(A)				__delay32((unsigned long)(A)*(CLK_SPEED / 2))
#define delay_ms(A)				__delay32((unsigned long)(A)*(CLK_SPEED / 2000))

// Pin RP Definitions
#define RT_UART_1_RX_RP		4                    //B4
#define RT_UART_2_RX_RP		5                    //B5
#define RT_UART_3_RX_RP		6                    //B6
#define RT_UART_4_RX_RP		7                    //B7
#define RT_SPI_1_SS_RP		8                    //B8
#define RT_SPI_1_CLK_RP		11                   //B11
#define RT_SPI_1_SDI_RP		10                   //B10

#define RT_UART_1_TX_RPOR	RPOR6bits.RP12R      //B12 RP12 LSB
#define RT_UART_2_TX_RPOR	RPOR6bits.RP13R      //B13 RP13 LSB
#define RT_UART_3_TX_RPOR	RPOR7bits.RP14R      //B14 RP14 LSB
#define RT_UART_4_TX_RPOR	RPOR7bits.RP15R      //B15 RP15 LSB
#define RT_SPI_1_SDO_RPOR	RPOR4bits.RP9R       //B9 RP9
#define RT_PWM_RPOR         RPOR1bits.RP2R       //B2 RP2 MSB


//Flash EE Memory
#include "FlashEE.h"


//Circular Buffers
//0-3 = Receive Buffers
//4-7 = Transmit Buffers
#define CBSIZE	1000
#define NUMBUF	8
unsigned char CB[NUMBUF][CBSIZE];						//Buffers
unsigned int CB_Start[NUMBUF] = {0,0,0,0,0,0,0,0};		//CB Start Pointers
unsigned int CB_End[NUMBUF] = {0,0,0,0,0,0,0,0};		//CB End Pointers


unsigned int sizeCB(unsigned char buffer)
{
	unsigned int start;
	if (buffer < NUMBUF)
	{
        start = CB_Start[buffer];

        if (start == CB_End[buffer])
        {
            return (0);
        }
        else
        {
            if (start < CB_End[buffer])
                return (CB_End[buffer] - start);
            else
                return ((CBSIZE - start) + CB_End[buffer]);
        }
	}
    return 0;
}

unsigned char getCB(unsigned char buffer)
{
	unsigned char retval = 0;
	if (buffer < NUMBUF)
	{
		if (CB_Start[buffer] != CB_End[buffer])
		{
			retval = CB[buffer][CB_Start[buffer]];
			CB_Start[buffer]++;
			if (CB_Start[buffer] == CBSIZE)
			{
				CB_Start[buffer] = 0;
			}
		}
	}
	return retval;
}

unsigned char putCB(unsigned char buffer, unsigned char value)
{
	unsigned char retval = 0;
	unsigned int end;
	if (buffer < NUMBUF)
	{
		end = CB_End[buffer] + 1;
		if (end == CBSIZE)
		{
			end = 0;
		}
		if (end != CB_Start[buffer])
		{
			CB[buffer][CB_End[buffer]] = value;
			CB_End[buffer] = end;
			retval = 1;
		}
	}
	return retval;
}


//UARTs
#define RT_HARD_BAUD_1200	(((CLK_SPEED / 1200) - 8) / 32)
#define RT_HARD_BAUD_2400	(((CLK_SPEED / 2400) - 8) / 32)
#define RT_HARD_BAUD_4800	(((CLK_SPEED / 4800) - 8) / 32)
#define RT_HARD_BAUD_9600	(((CLK_SPEED / 9600) - 8) / 32)
#define RT_HARD_BAUD_19200	(((CLK_SPEED / 19200) - 8) / 32)
#define RT_HARD_BAUD_31250	(((CLK_SPEED / 31250) - 8) / 32)
#define RT_HARD_BAUD_38400	(((CLK_SPEED / 38400) - 8) / 32)
#define RT_HARD_BAUD_57600	(((CLK_SPEED / 57600) - 8) / 32)
#define RT_HARD_BAUD_62500	(((CLK_SPEED / 62500) - 8) / 32)
#define RT_HARD_BAUD_115200	(((CLK_SPEED / 115200) - 8) / 32)
#define TXUART1(data)	U1TXREG = data;
#define TXUART2(data)	U2TXREG = data;
#define TXUART3(data)	U3TXREG = data;
#define TXUART4(data)	U4TXREG = data;


unsigned char dataFlags = 0;

void initUarts()
{
    unsigned int EEbaud[4];
    unsigned char idx = 0;
    
    TRISB &= 0x0FFF;                            //UART TX pins output
    
    while (idx < 4)                             //Read Baud Rates From ROM       
    {
        //EEbaud[idx] = FlashEERead(idx);   
        EEbaud[idx] = RT_HARD_BAUD_9600;        //Flash Write routine currently has a bug on this device! Workaround
        
        if ((EEbaud[idx] > RT_HARD_BAUD_1200) || (EEbaud[idx] < RT_HARD_BAUD_115200))              //Unprogrammed location?
        {
            EEbaud[idx] = RT_HARD_BAUD_9600;            //Default to 9600
            //FlashEEWrite(idx, RT_HARD_BAUD_9600);     //Write baud to flash
        }
        idx++;
    }
    
    
	//Channel 1
	RT_UART_1_TX_RPOR = 3;
	RPINR18bits.U1RXR = RT_UART_1_RX_RP;
	U1BRG = EEbaud[0];                          // Set the baud rate
	U1STA = 0;    								// Reset the UART
	U1MODE = 0;									// Reset the mode
	U1MODEbits.UARTEN = 1;         				// turn on serial interface 1
	U1STAbits.UTXEN = 1;
	IEC0bits.U1RXIE = 1;						// turn on RX interrupt
    IEC0bits.U1TXIE = 1;						// turn on TX interrupt
    
	//Channel 2
	RT_UART_2_TX_RPOR = 5;
	RPINR19bits.U2RXR = RT_UART_2_RX_RP;
	U2BRG = EEbaud[1];                          // Set the baud rate
	U2STA = 0;    								// Reset the UART
	U2MODE = 0;									// Reset the mode
	U2MODEbits.UARTEN = 1;         				// turn on serial interface 2
	U2STAbits.UTXEN = 1;
	IEC1bits.U2RXIE = 1;						// turn on RX interrupt
    IEC1bits.U2TXIE = 1;						// turn on TX interrupt
    
	//Channel 3
	RT_UART_3_TX_RPOR = 19;
	RPINR17bits.U3RXR = RT_UART_3_RX_RP;
	U3BRG = EEbaud[2];                          // Set the baud rate
	U3STA = 0;    								// Reset the UART
	U3MODE = 0;									// Reset the mode
	U3MODEbits.UARTEN = 1;         				// turn on serial interface 3
	U3STAbits.UTXEN = 1;
	IEC5bits.U3RXIE = 1;						// turn on RX interrupt
    IEC5bits.U3TXIE = 1;						// turn on TX interrupt
    
	//Channel 4
	RT_UART_4_TX_RPOR = 21;
	RPINR27bits.U4RXR = RT_UART_4_RX_RP;
	U4BRG = EEbaud[3];                          // Set the baud rate
	U4STA = 0;    								// Reset the UART
	U4MODE = 0;									// Reset the mode
	U4MODEbits.UARTEN = 1;         				// turn on serial interface 4
	U4STAbits.UTXEN = 1;
	IEC5bits.U4RXIE = 1;						// turn on RX interrupt
    IEC5bits.U4TXIE = 1;						// turn on TX interrupt
}


unsigned char RXUART1 ()
{
    putCB(0, U1RXREG);			//Load lower Byte into CB
    dataFlags |= 1;
	return 1;
}

unsigned char RXUART2 ()
{
    putCB(1, U2RXREG);			//Load lower Byte into CB
    dataFlags |= 2;
	return 1;
}

unsigned char RXUART3 ()
{
    putCB(2, U3RXREG);			//Load lower Byte into CB
    dataFlags |= 4;
	return 1;
}

unsigned char RXUART4 ()
{
    putCB(3, U4RXREG);			//Load lower Byte into CB
    dataFlags |= 8;
	return 1;
}

void changeBaud (unsigned char UART, unsigned char BAUD)
{
	unsigned int baudrate;

	if (UART >= 4)
		return;
	if (BAUD > 9)
		return;

	if (BAUD == 0)
	{
		baudrate = RT_HARD_BAUD_1200;
	}
	else if (BAUD == 1)
	{
		baudrate = RT_HARD_BAUD_2400;
	}
	else if (BAUD == 2)
	{
		baudrate = RT_HARD_BAUD_4800;
	}
	else if (BAUD == 3)
	{
		baudrate = RT_HARD_BAUD_9600;
	}
	else if (BAUD == 4)
	{
		baudrate = RT_HARD_BAUD_19200;
	}
	else if (BAUD == 5)
	{
		baudrate = RT_HARD_BAUD_38400;
	}
	else if (BAUD == 6)
	{
		baudrate = RT_HARD_BAUD_57600;
	}
	else if (BAUD == 7)
	{
		baudrate = RT_HARD_BAUD_115200;
	}
	else if (BAUD == 8)
	{
		baudrate = RT_HARD_BAUD_31250;
	}
	else if (BAUD == 9)
	{
		baudrate = RT_HARD_BAUD_62500;
	}
	else
	{
		return;
	}

	if (UART == 0)
	{
		U1MODEbits.UARTEN = 0;         				// turn off serial interface
		U1STAbits.UTXEN = 0;
		U1BRG = baudrate; 							// Set the baud rate
        //FlashEEWrite(0, baudrate);
		U1MODEbits.UARTEN = 1;         				// turn on serial interface
		U1STAbits.UTXEN = 1;
	}
	else if (UART == 1)
	{
		U2MODEbits.UARTEN = 0;         				// turn off serial interface
		U2STAbits.UTXEN = 0;
		U2BRG = baudrate;   						// Set the baud rate
        //FlashEEWrite(1, baudrate);
		U2MODEbits.UARTEN = 1;         				// turn on serial interface
		U2STAbits.UTXEN = 1;
	}
	else if (UART == 2)
	{
		U3MODEbits.UARTEN = 0;         				// turn off serial interface
		U3STAbits.UTXEN = 0;
		U3BRG = baudrate;   						// Set the baud rate
        //FlashEEWrite(2, baudrate);
		U3MODEbits.UARTEN = 1;         				// turn on serial interface
		U3STAbits.UTXEN = 1;
	}
	else if (UART == 3)
	{
		U4MODEbits.UARTEN = 0;         				// turn off serial interface
		U4STAbits.UTXEN = 0;
		U4BRG = baudrate;   						// Set the baud rate
        //FlashEEWrite(3, baudrate);
		U4MODEbits.UARTEN = 1;         				// turn on serial interface
		U4STAbits.UTXEN = 1;
	}
}


//SPI Slave

unsigned char fsmState = 0;
unsigned char fsmCommand;
unsigned char fsmBuffer;
unsigned char fsmCount;
unsigned char fsmIdx;

void initSPISlave()
{
	//MISO is output pin  - can we keep as input until needed?
	//MOSI is input pin
	//CLK is input pin
	//SS in an input pin
    TRISB &= 0xFDFF;
   
    SPI1CON1Lbits.SPIEN = 0;        //Ensure SPI is disabled

	RPINR20bits.SCK1R = RT_SPI_1_CLK_RP;		//SPI Clock Input
    RPINR20bits.SDI1R = RT_SPI_1_SDI_RP;        //SPI Data Input
	RPINR21bits.SS1R = RT_SPI_1_SS_RP;			//SPI SS Input
    RT_SPI_1_SDO_RPOR = 7;                      //SPI Data Output
    
	SPI1BUFL = 0;                   //Clear the SPIxBUF Register
    SPI1BUFH = 0;
        
	IFS3bits.SPI1RXIF = 0;          //Clear the SPIxIF bit
	IEC3bits.SPI1RXIE = 1;          //Enable SPI Slave interrupt
	IPC14bits.SPI1RXIP = 7;         //Set the interrupt priority - Highest
    
	SPI1CON1L = 0x0180;             //Slave mode using SS pin
	SPI1CON1H = 0x3000;
    SPI1CON2L = 0x0000;             //8-bit Data Mode
    SPI1IMSKL = 0x0001;             //Trigger Interrupt on Rx Buffer Full Event
    SPI1STATL = 0x0008;
    SPI1STATH = 0x0000;
	SPI1CON1Lbits.SPIEN = 1;		//Enable SPI operation
}


//PWM

void initPWM ()
{
    TRISB &= 0xFFFB;                //PWM pin is an output
	RT_PWM_RPOR = 13;               //Assign PWM Channel 1 -> RP
	OC1CON2 = 0x000C;			    //Timer 2 is clock source
	OC1RS = 0;				        //Enable Capture Compare Channel 1
	OC1R = 0;
	OC1CON1 = 0x0005;
    PR2 = 255;                      //Setup Timer 2
    T2CON = 0x8020;
}




void errState (unsigned char err)
{
    /*
    //Show the error
    unsigned char num = 0;
    while (1)
    {
        num = 0;
        while (num < err)
        {
            OC1RS = 255;
            delay_ms(100);
            OC1RS = 0;
            delay_ms(200);
            num++;
        }
        OC1RS = 0;
        delay_s(2);
    }
    */
    
    switch (err)
    {
        case 1:
            U1STA = 0;    								// Reset the UART
            U1MODE = 0;									// Reset the mode
            U1MODEbits.UARTEN = 1;         				// turn on serial interface 1
            U1STAbits.UTXEN = 1;
            IEC0bits.U1RXIE = 1;						// turn on RX interrupt
            IEC0bits.U1TXIE = 1;						// turn on TX interrupt
            break;
        case 2:
            U2STA = 0;    								// Reset the UART
            U2MODE = 0;									// Reset the mode
            U2MODEbits.UARTEN = 1;         				// turn on serial interface 2
            U2STAbits.UTXEN = 1;
            IEC1bits.U2RXIE = 1;						// turn on RX interrupt
            IEC1bits.U2TXIE = 1;						// turn on TX interrupt
            break;
        case 3:
            U3STA = 0;    								// Reset the UART
            U3MODE = 0;									// Reset the mode
            U3MODEbits.UARTEN = 1;         				// turn on serial interface 3
            U3STAbits.UTXEN = 1;
            IEC5bits.U3RXIE = 1;						// turn on RX interrupt
            IEC5bits.U3TXIE = 1;						// turn on TX interrupt
            break;
        case 4:
            U4STA = 0;    								// Reset the UART
            U4MODE = 0;									// Reset the mode
            U4MODEbits.UARTEN = 1;         				// turn on serial interface 4
            U4STAbits.UTXEN = 1;
            IEC5bits.U4RXIE = 1;						// turn on RX interrupt
            IEC5bits.U4TXIE = 1;						// turn on TX interrupt
            break;
        case 5:
            SPI1STATL = 0;  //Switch Off SPI
            SPI1STATH = 0;
            fsmState = 0;
            initSPISlave();
        default:
            break;
    }
}


//Program Code

int main (void)
{
    unsigned char dummy;

    CLKDIVbits.RCDIV = 0;   //8MHz x 4PLL = 32MHz
    ANSB = 0x0000;          //PortB Digital Mode

    /*
    //Test OSC timings & LED
    TRISB &= 0xFFFB;
    while (1)
    {
        LATB |= 0x0004;
        delay_s(1);
        LATB &= 0xFFFB;
        delay_s(1);
    }
    */

	initPWM();
    
    /*  
    //Test PWM  
    while (1)
    {
        if (OC1RS-- == 0)
            OC1RS = 200;
        delay_ms(10);
    }
    */
    
    FlashEEInitialise();   
    initUarts();
	initSPISlave();

    OC1RS = 10;

	while (1)
	{
        //LED PWM Output - Dim LED slowly in between interrupts
        if (OC1RS > 10)
        {
            OC1RS--;
        }

        //Check for pending UART tx data
        if ((sizeCB(4) > 0) && ((dataFlags & 0x10) == 0))
        {
            dataFlags |= 0x10;
            TXUART1(getCB(4));
        }
        if ((sizeCB(5) > 0) && ((dataFlags & 0x20) == 0))
        {
            dataFlags |= 0x20;
            TXUART2(getCB(5));
        }
        if ((sizeCB(6) > 0) && ((dataFlags & 0x40) == 0))
        {
            dataFlags |= 0x40;
            TXUART3(getCB(6));
        }
        if ((sizeCB(7) > 0) && ((dataFlags & 0x80) == 0))
        {
            dataFlags |= 0x80;
            TXUART4(getCB(7));
        }

        
        //Check for errors and clear them
        if(U1STA & 0x000E)                  //Check for UART1 related Errors
        {
            dummy = U1RXREG;     	 	//Clear Frame Error
            errState(1);                //Restart UART
        }
        if(U2STA & 0x000E)                  //Check for UART2 related Errors
        {
            dummy = U2RXREG;     	 	//Clear Frame Error
            errState(2);                //Restart UART  
        }
        if(U3STA & 0x000E)                  //Check for UART3 related Errors
        {
            dummy = U3RXREG;     	 	//Clear Frame Error
            errState(3);                //Restart UART
        }
        if(U4STA & 0x000E)                  //Check for UART4 related Errors
        {
            dummy = U4RXREG;     	 	//Clear Frame Error
            errState(4);                //Restart UART
        }
        if(SPI1STATL & 0x1140)              //Check for SPI related Errors
        {
            errState(5);
        }
    }
        
    return 1;           //We should never get here!
}


//No Auto PSV - Ensure interrupt latency is at a minimum
void __attribute__((interrupt, no_auto_psv)) _U1RXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U2RXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U3RXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U4RXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U1TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U2TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U3TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _U4TXInterrupt(void);
void __attribute__((interrupt, no_auto_psv)) _SPI1RXInterrupt(void);


//UART Interrupts
void _ISR _U1RXInterrupt(void)
{
        OC1RS = 64;
	RXUART1();				//Add data to receive buffer
	IFS0bits.U1RXIF = 0;
}

void _ISR _U2RXInterrupt(void)
{
        OC1RS = 64;
	RXUART2();				//Add data to receive buffer
	IFS1bits.U2RXIF = 0;
}

void _ISR _U3RXInterrupt(void)
{
        OC1RS = 64;
	RXUART3();				//Add data to receive buffer
	IFS5bits.U3RXIF = 0;
}

void _ISR _U4RXInterrupt(void)
{
        OC1RS = 64;
	RXUART4();				//Add data to receive buffer
	IFS5bits.U4RXIF = 0;
}

void _ISR _U1TXInterrupt(void)
{
	unsigned char data;
        OC1RS = 64;
	if (sizeCB(4) > 0)		//Anything else to send?
	{
		//dataFlags |= 0x10;
		data = getCB(4);
		TXUART1(data);
	}
	else
	{
		dataFlags &= 0xEF;
	}
	IFS0bits.U1TXIF = 0;
}

void _ISR _U2TXInterrupt(void)
{
	unsigned char data;
        OC1RS = 64;
	if (sizeCB(5) > 0)		//Anything else to send?
	{
		//dataFlags |= 0x20;
		data = getCB(5);
		TXUART2(data);
	}
	else
	{
		dataFlags &= 0xDF;
	}
	IFS1bits.U2TXIF = 0;
}

void _ISR _U3TXInterrupt(void)
{
	unsigned char data;
        OC1RS = 64;
	if (sizeCB(6) > 0)		//Anything else to send?
	{
		//dataFlags |= 0x40;
		data = getCB(6);
		TXUART3(data);
	}
	else
	{
		dataFlags &= 0xBF;
	}
	IFS5bits.U3TXIF = 0;
}

void _ISR _U4TXInterrupt(void)
{
	unsigned char data;
        OC1RS = 64;
	if (sizeCB(7) > 0)		//Anything else to send?
	{
		//dataFlags |= 0x80;
		data = getCB(7);
		TXUART4(data);
	}
	else
	{
		dataFlags &= 0x7F;
	}
	IFS5bits.U4TXIF = 0;
}



//SPI Interrupts
#define CMD_MASK	0xF0
#define CHN_MASK	0x03

#define CMD_CheckRX		0x10
#define CMD_CheckTX		0x30
#define CMD_GetRX		0x20
#define CMD_PutTX		0x40
#define CMD_SetBaud		0x80


void _ISR _SPI1RXInterrupt(void)
{
	unsigned char din = SPI1BUFL;				//receive the byte
    unsigned char dout = din;
	unsigned int size;

    OC1RS = 200;                                //LED PWM bright

    switch (fsmState)
    {
        case 0:
            fsmCommand = din & CMD_MASK;
            fsmBuffer = din & CHN_MASK;

            switch (fsmCommand)
            {
                case CMD_CheckRX:
                    fsmState = 7;						//Allow byte to be read before returning to mode 0
                    size = sizeCB(fsmBuffer);
                    if (size > 255)
                       dout = 255;
                    else
                       dout = size;
                    break;
                case CMD_CheckTX:
                    fsmState = 7;						//Allow byte to be read before returning to mode 0
                    size = sizeCB(fsmBuffer + 4);
                    if (size > 255)
                        dout = 255;
                    else
                        dout = size;
                    break;
                case CMD_GetRX:
                    fsmState = 1;
                    break;
                case CMD_PutTX:
                    fsmState = 2;
                    break;
                case CMD_SetBaud:
                    fsmState = 6;
                    break;
                default:
                    fsmState = 0;
                    dout = 0xFF;
                    break;
            }
            break;
        case 1:     //CMD_GetRX Length
            fsmCount = din;
            dout = getCB(fsmBuffer);
            fsmIdx = 1;
            if (fsmIdx == fsmCount)         //1 Byte Only?
            {
                fsmState = 7;				//Allow last byte to be read before returning to mode 0
            }
            else
                fsmState = 4;               //Else Goto multi byte receive state
            break;
        case 2:     //CMD_PutTX Length
            fsmCount = din;
            fsmIdx = 0;
            fsmState = 5;                   //Goto multi byte transmit state
            break;
        case 4:     //CMD_GetRX Data
            if (fsmIdx < fsmCount)
            {
                dout = getCB(fsmBuffer);
                fsmIdx++;
            }
            if (fsmIdx == fsmCount)
            {
                fsmState = 7;				//Allow last byte to be read before returning to mode 0
            }
            break;
        case 5:     //CMD_PutTX Data
            if (fsmIdx < fsmCount)
            {
                putCB(fsmBuffer + 4, din);
                fsmIdx++;
            }
            if (fsmIdx == fsmCount)         //Start TX if stopped
            {
                fsmState = 0;
            }
            break;
        case 6:
            changeBaud (fsmBuffer, din);
            fsmState = 0;                   //Allow last byte to be read before returning to mode 0
            break;
        default:
            fsmState = 0;
            break;
    }

	SPI1BUFL = dout;						// Assign output value
	IFS3bits.SPI1RXIF = 0;
}



