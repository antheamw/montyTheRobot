//robot tracking logic

#include <C8051F38x.h>
#include <stdlib.h>
#include <stdio.h>

#define SYSCLK 48000000L // SYSCLK frequency in Hz
#define BAUDRATE 115200L //Baud rate of UART in bps

#define VDD 3.325  //The measure value of VDD in volts

char _c51_external_startup (void)
{

	// [START standard F38x startup code]
	
	PCA0MD&=(~0x40) ;    // DISABLE WDT: clear Watchdog Enable bit
	VDM0CN=0x80; // enable VDD monitor
	RSTSRC=0x02|0x04; // Enable reset on missing clock detector and VDD

	// CLKSEL&=0b_1111_1000; // Not needed because CLKSEL==0 after reset
	#if (SYSCLK == 12000000L)
		//CLKSEL|=0b_0000_0000;  // SYSCLK derived from the Internal High-Frequency Oscillator / 4 
	#elif (SYSCLK == 24000000L)
		CLKSEL|=0b_0000_0010; // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
	#elif (SYSCLK == 48000000L)
		CLKSEL|=0b_0000_0011; // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
	#else
		#error SYSCLK must be either 12000000L, 24000000L, or 48000000L
	#endif
	OSCICN |= 0x03; // Configure internal oscillator for its maximum frequency
	RSTSRC  = 0x04;   // Enable missing clock detector

	// Configure UART0
	SCON0 = 0x10; 
	#if (SYSCLK/BAUDRATE/2L/256L < 1)
	TH1 = 0x10000-((SYSCLK/BAUDRATE)/2L);
	CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
	CKCON |=  0x08;
	#elif (SYSCLK/BAUDRATE/2L/256L < 4)
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/4L);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 01                  
	CKCON |=  0x01;
	#elif (SYSCLK/BAUDRATE/2L/256L < 12)
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2L/12L);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 00
	#else
	TH1 = 0x10000-(SYSCLK/BAUDRATE/2/48);
	CKCON &= ~0x0B; // T1M = 0; SCA1:0 = 10
	CKCON |=  0x02;
	#endif
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit autoreload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready
		

	// [END standard F38x startup code]

	P2MDOUT|=0b_0111_1111; // Set motor control pins
	
	// Configure P2.1, P2.6, P2.0 and P2.7 as analog input
	P2MDIN &= 0b_0011_1100; // P2.1, P2.6, P2.0 and P2.7
	P2SKIP |= 0b_1100_0011; // Skip Crossbar decoding for these pins
	
	// Init ADC
	ADC0CF = 0xF8; // SAR clock = 31, Right-justified result
	ADC0CN = 0b_1000_0000; // AD0EN=1, AD0TM=0
  	REF0CN=0b_0000_1000; //Select VDD as the voltage reference for the converter
  	
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
	P0MDOUT|=0x10;     // Enable Uart TX as push-pull output
	P1MDOUT|=0b_1111_1111;
	XBR0=0x01;         // Enable UART on P0.4(TX) and P0.5(RX)
	XBR1=0x40;         // Enable crossbar and weak pull-ups
	
	// Configure pin 0_1 to trigger an interrupt
	IT0 = 1;
	EX0 = 1;
	IT01CF|=0b_0000_0001;
	
	// Initialize timer 2 for periodic interrupts
	TMR2CN=0x00;   // Stop Timer2; Clear TF2;
	CKCON|=0b_0001_0000;
	TMR2RL=(-(SYSCLK/(DEFAULT_F))); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Start Timer2
	
	return 0;
}

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

//ADC Initialization
//Configure P2.0 as Analog Input
P2MDIN &= 0b_1111_1110;  //P2.0 only
P2SKIP |= 0b_0000_0001; //Skip Crossbar decoding for this pin

//Init ADC multiplexer to read the voltage between P2.0 and ground.
//IMPORTANT: check section 6.5 in datasheet.  The constants for 
//each pint are available in "c8051f38x.h" both for the 32 and 48
//pin packages.
AMX0P = LQFP32_MUX_P2_0;  //Select positive input from P0.0
AMX0N = LQFP32_MUX_GND;	  //GND is negative input (Single-ended Mode)

//Init ADC
ADC0CF = 0xF8; //SAR clock = 31, Right-justified result
ADC0CN = 0b_1000_0000;  //AD0EN=1, AD0TM = 0
REF0CN = 0b_0000_1000;  //Seelct Vdd as the voltage reference

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
	
		
	