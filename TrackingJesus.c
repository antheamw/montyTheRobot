// uses UART1 to receive command signals at P0.3
// ~C51

#include 	<C8051F38x.h>
#include 	<stdio.h>
#include 	<stdlib.h>

#define		SYSCLK         48000000L // System clock frequency in Hz
#define		BAUDRATE       115200L
#define		SMB_FREQUENCY  100000L   // I2C SCL clock rate (10kHz to 100kHz)
#define     DEFAULT_F      10300L
#define 	MHZ 1000000L

//motors
#define HIGH P1_4
#define SERVO180 P1_6
#define BASEMOTOR P1_5
#define LEFT0 P2_5
#define LEFT1 P2_4
#define RIGHT0 P2_7
#define RIGHT1 P2_6
#define laser P1_7
//commands
#define 	AUTO	0xAA

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

volatile float current_voltage1;
volatile float current_voltage2;
volatile int gunmov = 0;
volatile unsigned char pwm_count=0;
volatile unsigned int num1;
volatile unsigned int num2;
volatile unsigned int motor;
volatile unsigned int num=19;
volatile unsigned int pwmwidth;


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
	// Initialize Crossbar and GPIO
	P0MDOUT = 0x10;           // Enable Uart TX as push-pull output
	P1MDOUT = 0b1111_0000;
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
	TMR2RL=(-(SYSCLK/(2*48))/100L); // Initialize reload value
	TMR2=0xffff;   // Set to reload immediately
	ET2=1;         // Enable Timer2 interrupts
	TR2=1;         // Don't start Timer2
	
	// Configure and enable SMBus
	SMB0CF = INH | EXTHOLD | SMBTOE | SMBFTE ;
	SMB0CF |= ENSMB;  // Enable SMBus

	EA = 1;
	
	
	return 0;
}

void Timer2_ISR (void) interrupt 5
{
	unsigned char page_save;
	page_save=SFRPAGE;
	SFRPAGE=0;
	pwm_count++;
	
	if(pwm_count>num) pwm_count=0;

	if(motor==0){
		num=19;
		P1_4=0;
		HIGH=pwm_count>1?0:1;
		//P1_4=0;
	P1_6=0;
	P1_5=0;
	}
	
	else if(motor==1){
		num=19;
		P1_4=1;
		HIGH=pwm_count>15?0:1;
    //P1_4=1;
	P1_6=0;
	P1_5=0;
	}
	
	else if(motor==2){
		num=9;
		P1_6=0;
		SERVO180=pwm_count>2?0:1;
	P1_4=0;
	//P1_6=1;
	P1_5=0;
	}	
	
	else if(motor==3){
		num=19;
		P1_5=1;
		BASEMOTOR=pwm_count>17?0:1;
	P1_4=0;
	P1_6=0;
	//P1_5=1;
	}
	
	else if(motor==4){
		num=19;
		P1_5=0;
		BASEMOTOR=pwm_count>6?0:1;
	P1_4=0;
	P1_6=0;
	//P1_5=0;
	}
	
	else if(motor==5){
		num=9;
		P1_6=1;
		SERVO180=pwm_count>8?0:1;
	P1_4=0;
	//P1_6=1;
	P1_5=0;
	}	
SFRPAGE=page_save;
	TF2H = 0; // Clear Timer2 interrupt flag
	TF2L=0;
		
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

#define VDD      3.325 // The measured value of VDD in volts
#define NUM_INS  4

void main (void)
{

	unsigned char command;
	
	float v;
	unsigned char j;
	float d1=0.6;
	float d2=0.6;
	float d= 0.15;
	float buffer = 0.06;
	AD0BUSY=1;
	while (AD0BUSY); // Wait for conversion to complete

	while(1)
	{
		command = getchar1();
		printf("%u\n", command);
		
		//compare command to define statement 
		if (command==AUTO){
			
			printf("auto\n");
			for(j=0; j<NUM_INS; j++){
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
								d1=v;
							break;
							case 1:
								printf("V1=%5.3fV, ", v);
								d2=v;
							break;
						}
			
					}
					
				
				if (d1>0.2){		// 0.1 - 0.2 V is around the 60cm range
					//move motor 1 and 2 forward
					RIGHT0=1;
					RIGHT1=0;
					LEFT0=1;
					LEFT1=0;
					motor=0;
					EA=1;
			ET2=0;
			printf("forwards");
				}
				
				else if (d1<0.1){	// 0.1V is around the distance of 60cm, this allows you to have a buffer zone
					//move motor 2 and 1 backward
					RIGHT0=0;
					RIGHT1=1;
					LEFT0=0;
					LEFT1=1;
					motor=0;
					printf("backwards");
					EA=1;
			ET2=0;
				}
					else if (d2 < 0.7 - buffer ){
					//move right back and left forward(turn right)
					RIGHT0=1;
					RIGHT1=0;
					LEFT0=0;
					LEFT1=0;
					motor=0;
					EA=1;
					ET2=0;
			printf("right");
				}
				
				else if (d2 > 0.7 + buffer ){
					//move left back and right forward (turn left)
					RIGHT0=0;
					RIGHT1=1;
					LEFT0=0;
					LEFT1=0;
					motor=0;
					EA=1;
					ET2=0;
					
			printf("left");
				}
				
				else	// tracking conditions satisfied -> don't move
				{
					RIGHT0=0;
					RIGHT1=0;
					LEFT0=0;
					LEFT1=0;
					motor=0;
					EA=1;
					ET2=0;
				}
		}
		
else if (command==AIM_DEF) {

ET2=1;
			motor = 0;
		//	laser=0;
			
	
		}
		
		else if (command==AIM_U) {
			motor=5;
ET2=1;
		//	laser=0;
			printf("upshoot");
		}
		
		else if (command==AIM_D) {
			motor=2;
ET2=1;
		//	laser=0;
			
			printf("downshoot");
		}
		
		else if (command==AIM_L) {
			motor = 4;
ET2=1;
		//	laser=0;
			
			printf("leftshoot");
		}
		
		else if (command==AIM_R) {
			motor = 3;
ET2=1;
		//	laser=0;
			
			printf("rightshoot");
		}
		
		
		else if (command==AIM_S) {
			motor = 1;
		//	laser=0;
ET2=1;		
			printf("shoot");
		}
		
		else if (command==MOV_DL){
			motor = 0;
ET2=0;
			laser=1;
			RIGHT0=0;
			RIGHT1=1;
			LEFT0=0;
			LEFT1=0;
			printf("down left\n");
		}	

		else if (command==MOV_R){
ET2=0;
			motor = 0;
			laser=1;
			RIGHT0=0;
			RIGHT1=1;
			LEFT0=1;
			LEFT1=0;
			printf("RIGHT\n");
		}	
			
		else if (command==MOV_L){
ET2=0;
			motor = 0;
			laser=1;
			RIGHT0=1;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=1;
			printf("LEFT\n");
		}
		
		else if (command==MOV_U){
ET2=0;
			motor = 0;
			laser=1;
			RIGHT0=1;
			RIGHT1=0;
			LEFT0=1;
			LEFT1=0;
			printf("forwards\n");
		}

		else if (command==MOV_D){
ET2=0;
			motor = 0;
			laser=1;
			RIGHT0=0;
			RIGHT1=1;
			LEFT0=0;
			LEFT1=1;
			printf("backwards\n");
		}

		else if (command==MOV_UR){
ET2=0;
			motor = 0;
			laser=1;
			RIGHT0=0;
			RIGHT1=0;
			LEFT0=1;
			LEFT1=0;
			printf("up right\n");
		}

		else if (command==MOV_UL){

ET2=0;
			motor = 0;
			laser=1;
			RIGHT0=1;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=0;
			printf("up left\n");
		}
		
		else if	(command==MOV_DR){
			motor = 0;
			ET2=0;
			laser=1;
			RIGHT0=0;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=1;
			printf("down right\n");
		}
		

		else {	
			motor = 0;
			ET2=0;
			laser=1;
			RIGHT0=0;
			RIGHT1=0;
			LEFT0=0;
			LEFT1=0;
			printf("DEFAULT\n");
		}
		}
	
	
}
