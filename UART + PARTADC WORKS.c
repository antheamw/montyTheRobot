// uses UART1 to receive command signals at P0.3
// ~C51

#include 	<C8051F38x.h>
#include 	<stdio.h>
#include 	<stdlib.h>

#define		SYSCLK         48000000L // System clock frequency in Hz
#define		BAUDRATE       115200L
#define		SMB_FREQUENCY  100000L   // I2C SCL clock rate (10kHz to 100kHz)
#define     DEFAULT_F      10300L

//motors
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

#define     	d        0.6 	//60cm - distance to keep in AUTO mode

#define VDD      3.325 // The measured value of VDD in volts
#define NUM_INS  4

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
	PCA0MD&=(~0x40) ;  // DISABLE WDT: clear Watchdog Enable bit
	VDM0CN=0x80;       // Enable VDD monitor
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

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

	// configure UART1
	UART1_Init(110);

	// Initialize Crossbar and GPIO
	P0MDOUT = 0x10;           // Enable Uart TX as push-pull output
	P1MDOUT = 0b1000_0000;
	P2MDOUT |= 0b0000_0111;   // Make the LED (P2.2) a push-pull output.  P2.1 used for debuging.
	XBR0 = 0b0000_0101;       // Enable SMBus pins and UART pins P0.4(TX) and P0.5(RX)
	XBR1 = 0x40;              // Enable crossbar and weak pull-up
	XBR2		= 0x01; // Enable UART1 on P0.0(TX1) and P0.1(RX1)
	
	// Configure Timer 0 as the I2C clock source
	CKCON |= 0x04; // Timer0 clock source = SYSCLK
	TMOD &= 0xf0;  // Mask out timer 1 bits
	TMOD |= 0x02;  // Timer0 in 8-bit auto-reload mode
	// Timer 0 configured to overflow at 1/3 the rate defined by SMB_FREQUENCY
	TL0 = TH0 = 256-(SYSCLK/SMB_FREQUENCY/3);
	TR0 = 1; // Enable timer 0

	// initialize timer 1
	TL1 = TH1;     // Init timer 1
	TMOD &= ~0xf0;
	TI = 1;  // Indicate TX0 ready
	TMOD |= 0x20;  // TMOD: timer 1 in 8-bit autoreload                     
	TR1 = 1;       // Start timer1

	// Initialize timer 2 for periodic interrupts
	TMR2CN=0x00;   // Stop Timer2; Clear TF2;
	CKCON|=0b_0001_0000;
	TMR2RL=(-(SYSCLK/(2*DEFAULT_F))); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=0;         // Don't start Timer2
	
	// Configure and enable SMBus
	SMB0CF = INH | EXTHOLD | SMBTOE | SMBFTE ;
	SMB0CF |= ENSMB;  // Enable SMBus

	EA = 1;

	
	return 0;
}

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
			//buffer zone of d-THRESHOLD < d1 < d+THRESHOLD 
			//will not move if within this range
	
			if (d1 < d - THRESHOLD)		// if d1 is further back than desired d including threshold
			{
				//move motor 1 and 2 forward
				RIGHT0=1;
				RIGHT1=0;
				LEFT0=1;
				LEFT1=0;
				printf("forwards\n");
			}
			
				//buffer zone of 
			else if (d1 > d + THRESHOLD){		// if d1 is closer than desired d + threshold
				//move motor 2 and 1 backward
				RIGHT0=0;
				RIGHT1=1;
				LEFT0=0;
				LEFT1=1;
				printf("backwards\n");
			}
		
			else if (d1>d2){
				//move right back and left forward(turn right)
				RIGHT0=0;
				RIGHT1=1;
				LEFT0=1;
				LEFT1=0;
				printf("right\n");
			}
		
			else if (d2>d1) {
				//move left back and right forward (turn left)
				RIGHT0=1;
				RIGHT1=0;
				LEFT0=0;
				LEFT1=1;
				printf("left\n");
			}
			
			else 
				printf("don't move\n");
				
			printf("\x1B[K"); // ANSI escape sequence: Clear to end of line
			waitms(100);  // Wait 100ms before next round of measurements.
			
			
		}
		
		else if (command==MOV_DL){
			
			RIGHT0=0;
			RIGHT1=1;
			LEFT0=0;
			LEFT1=0;
			printf("down left\n");
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
		

		else 
		{	
			RIGHT0=0;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=0;
			printf("DEFAULT\n");
		}
	}
	
	
}
