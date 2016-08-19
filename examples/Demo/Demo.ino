/*****************
MULTIUART Arduino Library - Example and Test File
Created By: Ben Rowland
Date: 19 August 2016
Copyright: Rowland Technology
Released into the public domain.

Compatible with the MULTIUART or SPI2UART module
Uses a single SPI bus to control up to four buffered hardware UART channels
*****************/

#include <SPI.h>
#include <MULTIUART.h>

/*****
//Change the number to alter the chip select pin
*****/

MULTIUART multiuart(10);	//New MULTI UART library - CS pin = D10

char LENGTH;
char Str1[20];

void setup()
{

  //SPI Prescaler Options
  //SPI_CLOCK_DIV4 / SPI_CLOCK_DIV16 / SPI_CLOCK_DIV64
  //SPI_CLOCK_DIV128 / SPI_CLOCK_DIV2 / SPI_CLOCK_DIV8 / SPI_CLOCK_DIV32

  multiuart.initialise(SPI_CLOCK_DIV64);	// set up the SPI and MultiUART Library
  Serial.begin(9600);           		// set up Serial library at 9600 bps
  
  // Initialise the UART baud rates
  // 0=1200, 1=2400, 2=4800, 3=9600, 4=19200, 5=38400, 6=57600, 7=115200
  
  multiuart.SetBaud(0, 3);		// UART0 = 9600 Baud
  multiuart.SetBaud(1, 3);		// UART1 = 9600 Baud
  multiuart.SetBaud(2, 7);		// UART2 = 115200 Baud
  multiuart.SetBaud(3, 7);		// UART3 = 115200 Baud
}

void loop()
{
  delay(2000);			//2 second delay
  
  //Send out data on all four UARTs
  multiuart.TransmitString(0, "UART 0 Test", 12);
  multiuart.TransmitString(1, "UART 1 Test", 12);
  multiuart.TransmitString(2, "UART 2 Test", 12);
  multiuart.TransmitString(3, "UART 3 Test", 12);

  delay(2000);			//2 second delay

  Serial.print("UART 0: ");
  LENGTH = multiuart.CheckRx(0);	//Check UART 0 for incoming data
  if (LENGTH > 0)
  {
    multiuart.ReceiveString(Str1, 0, LENGTH);	//Collect incoming data from buffer
    Serial.println(Str1);			//Forward data to PC
  }
  
  Serial.print("UART 1: ");
  LENGTH = multiuart.CheckRx(1);	//Check UART 1 for incoming data
  if (LENGTH > 0)
  {
    multiuart.ReceiveString(Str1, 1, LENGTH);	//Collect incoming data from buffer
    Serial.println(Str1);			//Forward data to PC
  }
  
  Serial.print("UART 2: ");
  LENGTH = multiuart.CheckRx(2);	//Check UART 2 for incoming data
  if (LENGTH > 0)
  {
    multiuart.ReceiveString(Str1, 2, LENGTH);	//Collect incoming data from buffer
    Serial.println(Str1);			//Forward data to PC
  }
  
  Serial.print("UART 3: ");
  LENGTH = multiuart.CheckRx(3);	//Check UART 3 for incoming data
  if (LENGTH > 0)
  {
    multiuart.ReceiveString(Str1, 3, LENGTH);	//Collect incoming data from buffer
    Serial.println(Str1);			//Forward data to PC
  }

}