//robot tracking logic

#include <C8051F38x.h>
#include <stdlib.h>
#include <stdio.h>


void UART1_Init (unsigned long baudrate)
{
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x10;
	
	if (((SYSCLK/baudrate)/2L)/0xFFFFL < 1){
		SBRL1 = 0x10000L-((SYSCLK/baudrate)/2L);
		SBCON1 |= 0x03; // set prescaler to 1
	}
	
	else if (((SYSCLK/baudrate)/2L)/0xFFFFL < 4){
		SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/4L);
		SBCON1 &= ~0x03;
		SBCON1 |= 0x01; // set prescaler to 4
	}
	
	else if (((SYSCLK/baudrate)/2L)/0xFFFFL < 12){
		SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/12L);
		SBCON1 &= ~0x03; // set prescaler to 12
	}
	
	else{
		SBRL1 = 0x10000L-(((SYSCLK/baudrate)/2L)/48L);
		SBCON1 &= ~0x03;
		SBCON1 |= 0x02; // set prescaler to ?
	}
	
	SCON1 |= 0x02; // indicate ready for TX
	SBCON1 |= 0x40; // enable baud rate generator
}

//UART1 transmit/receive
void putchar1 (char c)
{
	if (c == '\n' )
	{
		while (!(SCON1 & 0x02));
		SCON1 &= ~0x02;
		SBUF1 = '\r' ;
	}
	
	while (!(SCON1 & 0x02));
	SCON1 &= ~0x02;
	SBUF1 = c;
}

char getchar1 (void)
{
	char c;
	while (!(SCON1 & 0x01));
	SCON1 &= ~0x01;
	c = SBUF1;
	return (c);
}

//robot tries to keep d1=d2
void main (void)
{
	float d, d1, d2;
	int field_present; //take signal from comparator and convert to logic (1=on, 0=off)
	
	if (d1>d){
		//move motor 1 back
	}
	
	if (d2>d){
		//move motor 2 back
	}
	
	if (d1<d){
		//move motor 1 forward
	}
	
	if (d2<d){
		//move motor 2 forward
	}
}
	
		
	
