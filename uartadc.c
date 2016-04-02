// C8051F381_ADC_multiple_inputs.c:  Shows how to use the 10-bit ADC and the
// multiplexer.  This program measures the voltages applied to pins P2.0 to P2.3.
//
// (c) 2008-2014, Jesus Calvino-Fraga
//
// ~C51~ 
// save 2.0 to 2.3 for adc
#include <stdio.h>
#include <stdlib.h>
#include <c8051f38x.h>

#define MHZ 1000000L
#define SYSCLK (24*MHZ)
#define BAUDRATE 115200L
#define     DEFAULT_F      10300L


#define LEFT1 P2_5
#define LEFT0 P2_4
#define RIGHT1 P2_7
#define RIGHT0 P2_6

//commands
#define 	AUTO	0x0F

#define 	MOV_DEF 0x80

#define 	MOV_U	0x88
#define 	MOV_D	0x84
#define 	MOV_L	0x82
#define 	MOV_R	0x81

#define 	MOV_UL	0x8A
#define 	MOV_UR	0x89
#define 	MOV_DL	0x86
#define 	MOV_DR	0x85

#define 	AIM_DEF	0x40

#define 	AIM_U	0x48
#define 	AIM_D	0x44
#define 	AIM_L	0x42
#define 	AIM_R	0x41

#define 	AIM_UL	0x4A
#define 	AIM_UR	0x49
#define 	AIM_DL	0x46
#define 	AIM_DR	0x45

#define 	AIM_S	0x50    // shoot

//variables for certain distances 
#define     	d        0.6 	//60cm - distance to keep in AUTO mode
#define 	dmin     3.48	//15cm - closest distance btwn trans & rec
#define 	dmax	 0.4	//75cm - farthest distance btwn trans & rec

#define		THRESHOLD 0.08	//+/- 80mV for voltage measurement

volatile float current_voltage1;
volatile float current_voltage2;

//initialize uart
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

char _c51_external_startup (void)
{
	PCA0MD&=(~0x40) ;    // DISABLE WDT: clear Watchdog Enable bit
	// CLKSEL&=0b_1111_1000; // Not needed because CLKSEL==0 after reset
	#if (SYSCLK == (12*MHZ))
		//CLKSEL|=0b_0000_0000;  // SYSCLK derived from the Internal High-Frequency Oscillator / 4 
	#elif (SYSCLK == (24*MHZ))
		CLKSEL|=0b_0000_0010; // SYSCLK derived from the Internal High-Frequency Oscillator / 2.
	#elif (SYSCLK == (48*MHZ))
		CLKSEL|=0b_0000_0011; // SYSCLK derived from the Internal High-Frequency Oscillator / 1.
	#else
		#error SYSCLK must be either 12MHZ, 24MHZ, or 48MHZ
	#endif
	OSCICN |= 0x03; // Configure internal oscillator for its maximum frequency
	
	// Configure P2.0 to P2.3 as analog inputs
	P2MDIN &= 0b_1111_0000; // P2.0 to P2.3
	P2SKIP |= 0b_0000_1111; // Skip Crossbar decoding for these pins

	// Init ADC multiplexer to read the voltage between P2.0 and ground.
	// These values will be changed when measuring to get the voltages from
	// other pins.
	// IMPORTANT: check section 6.5 in datasheet.  The constants for
	// each pin are available in "c8051f38x.h" both for the 32 and 48
	// pin packages.
	AMX0P = LQFP32_MUX_P2_0; // Select positive input from P2.0
	AMX0N = LQFP32_MUX_GND;  // GND is negative input (Single-ended Mode)
	
	// Init ADC
	ADC0CF = 0xF8; // SAR clock = 31, Right-justified result
	ADC0CN = 0b_1000_0000; // AD0EN=1, AD0TM=0
  	REF0CN=0b_0000_1000; //Select VDD as the voltage reference for the converter
  	
	VDM0CN=0x80;       // enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD
	P0MDOUT|=0x10;     // Enable Uart TX as push-pull output
	XBR0=0x01;         // Enable UART on P0.4(TX) and P0.5(RX)
	XBR1=0x40;         // Enable crossbar and weak pull-ups
	
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
	
	TL1 = TH1;     // Init timer 1
	TMOD &= 0x0f;  // TMOD: timer 1 in 8-bit autoreload
	TMOD |= 0x20;                       
	TR1 = 1;       // Start timer1
	SCON = 0x52;
	
	return 0;
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON:
	CKCON|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN & 0x80));  // Wait for overflow
		TMR3CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	unsigned char k;
	for(j=0; j<ms; j++)
		for (k=0; k<4; k++) Timer3us(250);
}

#define VDD      3.325 // The measured value of VDD in volts
#define NUM_INS  4

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

//function to get current voltage from P2.0 P2.1
//returns voltage in global variable
void get_currentVoltage(void)
{
	float v;
	unsigned char j;
	printf("\x1B[6;1H"); // ANSI escape sequence: move to row 6, column 1

	for(j=0; j<NUM_INS; j++)
	{
		AD0BUSY = 1; // Start ADC 0 conversion to measure previously selected input
			
		// Select next channel while ADC0 is busy
		switch(j)
		{
			case 0:
				AMX0P=LQFP32_MUX_P2_1;
			break;
			case 1:
				AMX0P=LQFP32_MUX_P2_0;
			break;
			
		}
			
		while (AD0BUSY); // Wait for conversion to complete
		v = ((ADC0L+(ADC0H*0x100))*VDD)/1023.0; // Read 0-1023 value in ADC0 and convert to volts
			
		// Display measured values
		switch(j)
		{
			case 0:
				printf("V0=%5.3fV, ", v);
				current_voltage1=v;
			break;
			case 1:
				printf("V1=%5.3fV, ", v);
				current_voltage2=v;
			break;
			
		{
	}
 
}
}
}

void main (void)
{
	
	unsigned char command;
	float d1;
	float d2;
	printf("\x1b[2J"); // Clear screen using ANSI escape sequence.
	
	printf ("ADC/Multiplexer test program\n"
	        "Apply analog voltages to P2.0, P2.1, P2.2, and P2.3\n"
	        "File: %s\n"
	        "Compiled: %s, %s\n\n",
	        __FILE__, __DATE__, __TIME__);

	// Start the ADC in order to select the first channel.
	// Since we don't know how the input multiplexer was set up,
	// this initial conversion needs to be discarded.
	AD0BUSY=1;
	while (AD0BUSY); // Wait for conversion to complete

	while(1)
	{
	
		command = getchar1();
		printf("%u\n", command);
		
		//compare command to define statement 
		if (command==AUTO){
			printf("auto\n");
			get_currentVoltage();
			d1 = current_voltage1;
			d2 = current_voltage2;
			//keep the robot 60cm away
			//buffer zone of d-THRESHOLD < d1 < d+THRESHOLD (d1<d)
			/*
			if ((d1>(d-THRESHOLD))&&(d1<(d+THRESHOLD))){
				//move motor 1 and 2 forward
				RIGHT0=1;
				RIGHT1=0;
				LEFT0=1;
				LEFT1=0;
				printf("forwards");
			} */
			
			if (d1 < d - THRESHOLD)		// if d1 is further back than desired d including threshold
			{
				//move motor 1 and 2 forward
				RIGHT0=1;
				RIGHT1=0;
				LEFT0=1;
				LEFT1=0;
				printf("forwards");
			}
			
			//buffer zone of 
			else if (d1 > d + THRESHOLD){		// if d1 is closer than desired d + threshold
				//move motor 2 and 1 backward
				RIGHT0=0;
				RIGHT1=1;
				LEFT0=0;
				LEFT1=1;
				printf("backwards");
			}
		
			else if (d1>d2){
				//move right back and left forward(turn right)
				RIGHT0=0;
				RIGHT1=1;
				LEFT0=1;
				LEFT1=0;
				printf("right");
			}
		
			else if (d2>d1){
				//move left back and right forward (turn left)
				RIGHT0=1;
				RIGHT1=0;
				LEFT0=0;
				LEFT1=1;
				printf("left");
			}
			printf("\x1B[K"); // ANSI escape sequence: Clear to end of line
			waitms(100);  // Wait 100ms before next round of measurements.
			
		}
		
		else if (command==MOV_DEF){
			RIGHT0=0;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=0;
			printf("mov_def\n");
		}	

		else if (command==MOV_R){
			RIGHT0=0;
			RIGHT1=1;
			LEFT0=1;
			LEFT1=0;
			printf("RIGHT\n");
		}	
			
		else if (command==MOV_L){
			RIGHT0=1;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=1;
			printf("LEFT\n");
		}
		
		else if (command==MOV_U){
			RIGHT0=1;
			RIGHT1=0;
			LEFT0=1;
			LEFT1=0;
			printf("forwards\n");
		}

		else if (command==MOV_D){
			RIGHT0=0;
			RIGHT1=1;
			LEFT0=0;
			LEFT1=1;
			printf("backwards\n");
		}

		else if (command==MOV_UR){
			RIGHT0=0;
			RIGHT1=0;
			LEFT0=1;
			LEFT1=0;
			printf("up right\n");
		}

		else if (command==MOV_UL){
			RIGHT0=1;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=0;
			printf("up left\n");
		}
		
		else if	(command==MOV_DR){
			RIGHT0=0;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=1;
			printf("down right\n");
		}
		
		else {//(command==MOV_DL){
			RIGHT0=0;
			RIGHT1=1;
			LEFT0=0;
			LEFT1=0;
			printf("down left\n");
		}
/*
		if(input==gunright){

		}
		if(input==gunleft){

		}
		if(input==gunup){

		}
		if(input==gundown){

		}
		if(input==shoot){

		}
		if(input==laseron){

		}
		if(input==laseroff){

		}*/
		
		
	
		
	 }  

}	

