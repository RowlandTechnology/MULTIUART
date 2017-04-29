//Copyright: Rowland Technologies
//Owner: Ben Rowland
//Creation Date: 27/03/15
//Target Chip: PIC24FJ64GA306

//Chip Config
#include <xc.h>

// CONFIG4
#pragma config DSWDTPS = DSWDTPS1F      // Deep Sleep Watchdog Timer Postscale Select bits (1:68719476736 (25.7 Days))
#pragma config DSWDTOSC = LPRC          // DSWDT Reference Clock Select (DSWDT uses LPRC as reference clock)
#pragma config DSBOREN = ON             // Deep Sleep BOR Enable bit (DSBOR Enabled)
#pragma config DSWDTEN = ON             // Deep Sleep Watchdog Timer Enable (DSWDT Enabled)
#pragma config DSSWEN = ON              // DSEN Bit Enable (Deep Sleep is controlled by the register bit DSEN)

// CONFIG3
#pragma config WPFP = WPFP63            // Write Protection Flash Page Segment Boundary (Page 52 (0xFC00))
#pragma config VBTBOR = ON              // VBAT BOR enable bit (VBAT BOR enabled)
#pragma config SOSCSEL = ON             // SOSC Selection bits (SOSC circuit selected)
#pragma config WDTWIN = PS25_0          // Watch Dog Timer Window Width (Watch Dog Timer Window Width is 25 percent)
#pragma config BOREN = ON               // Brown-out Reset Enable (Brown-out Reset Enable)
#pragma config WPDIS = WPDIS            // Segment Write Protection Disable (Disabled)
#pragma config WPCFG = WPCFGDIS         // Write Protect Configuration Page Select (Disabled)
#pragma config WPEND = WPENDMEM         // Segment Write Protection End Page Select (Write Protect from WPFP to the last page of memory)

// CONFIG2
#pragma config POSCMD = NONE            // Primary Oscillator Select (Primary Oscillator Disabled)
#pragma config BOREN1 = EN              // BOR Override bit (BOR Enabled [When BOREN=1])
#pragma config IOL1WAY = ON             // IOLOCK One-Way Set Enable bit (Once set, the IOLOCK bit cannot be cleared)
#pragma config OSCIOFCN = ON            // OSCO Pin Configuration (OSCO/CLKO/RC15 functions as port I/O (RC15))
#pragma config FCKSM = CSDCMD           // Clock Switching and Fail-Safe Clock Monitor Configuration bits (Clock switching and Fail-Safe Clock Monitor are disabled)
#pragma config FNOSC = FRCPLL           // Initial Oscillator Select (Fast RC Oscillator with PLL module (FRCPLL))
#pragma config ALTVREF = DLT_AV_DLT_CV  // Alternate VREF/CVREF Pins Selection bit (Voltage reference input, ADC =RA9/RA10 Comparator =RA9,RA10)
#pragma config IESO = ON                // Internal External Switchover (Enabled)

// CONFIG1
#pragma config WDTPS = PS32768          // Watchdog Timer Postscaler Select (1:32,768)
#pragma config FWPSA = PR128            // WDT Prescaler Ratio Select (1:128)
#pragma config FWDTEN = WDT_DIS         // Watchdog Timer Enable (WDT disabled in hardware; SWDTEN bit disabled)
#pragma config WINDIS = OFF             // Windowed WDT Disable (Standard Watchdog Timer)
#pragma config ICS = PGx1               // Emulator Pin Placement Select bits (Emulator functions are shared with PGEC1/PGED1)
#pragma config LPCFG = OFF              // Low power regulator control (Disabled)
#pragma config GWRP = OFF               // General Segment Write Protect (Disabled)
#pragma config GCP = OFF                // General Segment Code Protect (Code protection is disabled)
#pragma config JTAGEN = ON              // JTAG Port Enable (Enabled)


#define CLK_SPEED 32000000

extern void __delay32(unsigned long count);

#define delay_s(A)				__delay32((unsigned long)(A)*(CLK_SPEED / 2))
#define delay_ms(A)				__delay32((unsigned long)(A)*(CLK_SPEED / 2000))



// Pin RP Definitions

#define RT_UART_1_RX_RP		25  //D4
#define RT_UART_2_RX_RP		23  //D2
#define RT_UART_3_RX_RP		11  //D0
#define RT_UART_4_RX_RP		2   //D8
#define RT_UART_1_TX_RPOR	RPOR10      //D5 RP20 LSB
#define RT_UART_2_TX_RPOR	RPOR11      //D3 RP22 LSB
#define RT_UART_3_TX_RPOR	RPOR12      //D1 RP24 LSB
#define RT_UART_4_TX_RPOR	RPOR2       //D9 RP4 LSB

#define RT_SPI_1_SS_RP		17          //F5
#define RT_SPI_1_CLK_RP		10          //F4
#define RT_SPI_1_SDI_RP		16          //F3
#define RT_SPI_1_SDO_RPOR	RPOR15      //F2 RP30 LSB

#define RT_PWM_RPOR         RPOR3       //B7 RP7 MSB


//Circular Buffers

//0-3 = Receive Buffers
//4-7 = Transmit Buffers

#define CBSIZE	512
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

#define RT_HARD_BAUD_1200	(((CLK_SPEED / 1200) - 8) / 32)		//Was - 16 but had issues with rounding down
#define RT_HARD_BAUD_2400	(((CLK_SPEED / 2400) - 8) / 32)
#define RT_HARD_BAUD_4800	(((CLK_SPEED / 4800) - 8) / 32)
#define RT_HARD_BAUD_9600	(((CLK_SPEED / 9600) - 8) / 32)
#define RT_HARD_BAUD_19200	(((CLK_SPEED / 19200) - 8) / 32)
#define RT_HARD_BAUD_38400	(((CLK_SPEED / 38400) - 8) / 32)
#define RT_HARD_BAUD_57600	(((CLK_SPEED / 57600) - 8) / 32)
#define RT_HARD_BAUD_115200	(((CLK_SPEED / 115200) - 8) / 32)

#define TXUART1(data)	U1TXREG = data;
#define TXUART2(data)	U2TXREG = data;
#define TXUART3(data)	U3TXREG = data;
#define TXUART4(data)	U4TXREG = data;

unsigned char dataFlags = 0;

void initUarts()
{
	//Channel 1
	RT_UART_1_TX_RPOR = 3;
	RPINR18 = RT_UART_1_RX_RP;
	U1BRG = RT_HARD_BAUD_9600; 					// Set the baud rate
	U1STA = 0;    								// Reset the UART
	U1MODE = 0;									// Reset the mode
	U1MODEbits.UARTEN = 1;         				// turn on serial interface 1
	U1STAbits.UTXEN = 1;
	IEC0bits.U1RXIE = 1;						// turn on RX interrupt
    IEC0bits.U1TXIE = 1;						// turn on TX interrupt

	//Channel 2
	RT_UART_2_TX_RPOR = 5;
	RPINR19 = RT_UART_2_RX_RP;
	U2BRG = RT_HARD_BAUD_9600;   				// Set the baud rate
	U2STA = 0;    								// Reset the UART
	U2MODE = 0;									// Reset the mode
	U2MODEbits.UARTEN = 1;         				// turn on serial interface 2
	U2STAbits.UTXEN = 1;
	IEC1bits.U2RXIE = 1;						// turn on RX interrupt
    IEC1bits.U2TXIE = 1;						// turn on TX interrupt

	//Channel 3
	RT_UART_3_TX_RPOR = 28;
	RPINR17 = (RT_UART_3_RX_RP << 8);
	U3BRG = RT_HARD_BAUD_9600;   				// Set the baud rate
	U3STA = 0;    								// Reset the UART
	U3MODE = 0;									// Reset the mode
	U3MODEbits.UARTEN = 1;         				// turn on serial interface 3
	U3STAbits.UTXEN = 1;
	IEC5bits.U3RXIE = 1;						// turn on RX interrupt
    IEC5bits.U3TXIE = 1;						// turn on TX interrupt

	//Channel 4
	RT_UART_4_TX_RPOR = 30;
	RPINR27 = RT_UART_4_RX_RP;
	U4BRG = RT_HARD_BAUD_9600;   				// Set the baud rate
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
/*	unsigned char retval = 0;
        unsigned char dummy;
	if (U2STAbits.URXDA)
	{
		if (U2STAbits.FERR)				//Frame Error?
		{
			dummy = U2RXREG;     	 	//Clear Frame Error
		}
		else if (U2STAbits.OERR)		//Overrun Error?
		{
			U2STAbits.OERR = 0;			//Clear Overrun Error
		}
		else
		{*/
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
	if (BAUD > 7)
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
	else
	{
		return;
	}

	if (UART == 0)
	{
		U1MODEbits.UARTEN = 0;         				// turn off serial interface
		U1STAbits.UTXEN = 0;
		U1BRG = baudrate; 							// Set the baud rate
		U1MODEbits.UARTEN = 1;         				// turn on serial interface
		U1STAbits.UTXEN = 1;
	}
	else if (UART == 1)
	{
		U2MODEbits.UARTEN = 0;         				// turn off serial interface
		U2STAbits.UTXEN = 0;
		U2BRG = baudrate;   						// Set the baud rate
		U2MODEbits.UARTEN = 1;         				// turn on serial interface
		U2STAbits.UTXEN = 1;
	}
	else if (UART == 2)
	{
		U3MODEbits.UARTEN = 0;         				// turn off serial interface
		U3STAbits.UTXEN = 0;
		U3BRG = baudrate;   						// Set the baud rate
		U3MODEbits.UARTEN = 1;         				// turn on serial interface
		U3STAbits.UTXEN = 1;
	}
	else if (UART == 3)
	{
		U4MODEbits.UARTEN = 0;         				// turn off serial interface
		U4STAbits.UTXEN = 0;
		U4BRG = baudrate;   						// Set the baud rate
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

	RPINR20 = RT_SPI_1_SDI_RP | (RT_SPI_1_CLK_RP << 8);		//SPI Clock Input & Data Input
	RPINR21 = RT_SPI_1_SS_RP;					//SPI SS Input

	RT_SPI_1_SDO_RPOR = 7;		//SPI Data Output

	SPI1BUF = 0;			//Clear the SPIxBUF Register
	IFS0bits.SPI1IF = 0;            //Clear the SPIxIF bit
	IEC0bits.SPI1IE = 1;		//Enable SPI Slave interrupt
	IPC2bits.SPI1IP = 7;		//Set the interrupt priority - Highest

	SPI1CON1 = 0x180;		//Slave mode using SS pin
	SPI1CON2 = 0;

	SPI1STATbits.SPIROV = 0;
	SPI1STATbits.SPIEN = 1;		//Enable SPI operation
}


//PWM

void initPWM ()
{
        TRISB &= 0xFF7F;                        //PWM pin is an output
	RT_PWM_RPOR = 18 << 8;			//Assign PWM Channel 1 -> RP7
	OC1CON2 = 0x000C;			//Timer 2 is clock source
	OC1RS = 0;				//Enable Capture Compare Channel 1
	OC1R = 0;
	OC1CON1 = 0x0005;
        PR2 = 255;                              //Setup Timer 2
        T2CON = 0x8020;
}




//Program Code

int main (void)
{
    unsigned char dummy;

    CLKDIVbits.RCDIV = 0;   //8MHz x 4PLL = 32MHz

    /*
    TRISB &= 0xFF7F;
    while (1)
    {
        LATB |= 0x80;
        delay_s(1);
        LATB &= 0xFF7F;
        delay_s(1);
    }*/


	initPWM();
	initSPISlave();
	initUarts();

        OC1RS = 10;

	while (1)
	{
            if (OC1RS > 10)
            {
                OC1RS--;
            }

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
            if(U1STA & 0x000E)
            {
                dummy = U1RXREG;     	 	//Clear Frame Error
                errState(1);// U1STA = U1STA & 0xFFF1;
            }
            if(U2STA & 0x000E)
            {
                dummy = U2RXREG;     	 	//Clear Frame Error
                errState(2);// U2STA = U2STA & 0xFFF1;
            }
            if(U3STA & 0x000E)
            {
                dummy = U3RXREG;     	 	//Clear Frame Error
                errState(3);// U3STA = U3STA & 0xFFF1;
            }
            if(U4STA & 0x000E)
            {
                dummy = U4RXREG;     	 	//Clear Frame Error
                errState(4);// U4STA = U4STA & 0xFFF1;
            }
            if(SPI1STATbits.SPIROV)
                errState(5);
        }
        return 1;
}



void errState (unsigned char err)
{
/*    unsigned char num = 0;
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
*/
    
    switch (err)
    {
        case 1:
            U1MODE = 0;
            initUarts();
            break;
        case 2:
            U2MODE = 0;
            initUarts();
            break;
        case 3:
            U3MODE = 0;
            initUarts();
            break;
        case 4:
            U4MODE = 0;
            initUarts();
            break;
        case 5:
            SPI1STAT = 0;
            fsmState = 0;
            initSPISlave();
        default:
            break;
    }
}



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


void _ISR _SPI1Interrupt(void)
{
	unsigned char din = SPI1BUF;				//receive the byte
	unsigned char dout = din;
	unsigned int size;

        OC1RS = 200;

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
                fsmState = 4;
                break;
            case 2:     //CMD_PutTX Length
				fsmCount = din;
				fsmIdx = 0;
				fsmState = 5;
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
				if (fsmIdx == fsmCount)				//Start TX if stopped
				{
                    fsmState = 0;
				}
                break;
            case 6:
				changeBaud (fsmBuffer, din);
				fsmState = 0;			//Allow last byte to be read before returning to mode 0
                break;
            default:
				fsmState = 0;
                break;
        }

	SPI1BUF = dout;						// Assign output value
	IFS0bits.SPI1IF = 0;
}



