/*****************
MULTIUART Arduino Library
Created By: Ben Rowland
Date: 19 August 2016
Copyright: Rowland Technology
Released into the public domain.

Compatible with the MULTIUART or SPI2UART module
Uses a single SPI bus to control up to four buffered hardware UART channels
*****************/

#ifndef MULTIUART_h
#define MULTIUART_h

	#include "Arduino.h"

	class MULTIUART
	{
	  public:
		MULTIUART(int ss);
		void initialise(int SPIDivider);
		char CheckRx(char UART);
		char CheckTx(char UART);
		char ReceiveByte(char UART);
		void ReceiveString(char *RETVAL, char UART, char NUMBYTES);
		void TransmitByte(char UART, char DATA);
		void TransmitString(char UART, char *DATA, char NUMBYTES);
		void SetBaud(char UART, char BAUD);
	  private:
		int _ss_pin;
	};

#endif

