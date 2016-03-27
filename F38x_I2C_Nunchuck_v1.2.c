// F38x_I2C_Nunchuck_v1.2.c: Nunchuk and frequency generation
// uses Timer 2 for frequency generation
// takes input from the Nunchuk
// calibration code added in
// uses UART1 to send command signals out of P0.2
//  ~C51~  

#include <C8051F38x.h>
#include <stdio.h>
#include <stdlib.h>

#define OUT0 P2_0
#define OUT1 P2_1
#define DEFAULT_F 10300L

#define  SYSCLK         48000000L // System clock frequency in Hz
#define  BAUDRATE       115200L
#define  SMB_FREQUENCY  100000L   // I2C SCL clock rate (10kHz to 100kHz)

#define  LED        P2_2
#define  LED_ON     0
#define  LED_OFF    1

//commands
#define AUTO	0x0F

#define MOV_DEF 0x80

#define MOV_U	0x88
#define MOV_D	0x84
#define MOV_L	0x82
#define MOV_R	0x81

#define MOV_UL	0x8A
#define MOV_UR	0x89
#define MOV_DL	0x86
#define MOV_DR	0x85

#define AIM_DEF	0x40

#define AIM_U	0x48
#define AIM_D	0x44
#define AIM_L	0x42
#define AIM_R	0x41

#define AIM_UL	0x4A
#define AIM_UR	0x49
#define AIM_DL	0x46
#define AIM_DR	0x45

#define AIM_S	0x50    // shoot


volatile unsigned char pwm_count=0;

void UART1_Init (unsigned long baudrate) {
	SMOD1 = 0x0C; // no parity, 8 data bits, 1 stop bit
	SCON1 = 0x02;
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
	
	TL1 = TH1;     // Init timer 1
	TMOD &= ~0xf0;
	TI = 1;  // Indicate TX0 ready
	TMOD |= 0x20;  // TMOD: timer 1 in 8-bit autoreload                     
	TR1 = 1;       // Start timer1

	// Initialize Crossbar and GPIO
	P0MDOUT = 0x10;           // Enable Uart TX as push-pull output
	
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

	LED = LED_OFF;
	
	return 0;
}

// Uses Timer4 to delay <ms> mili-seconds. 
void Timer4ms(unsigned char ms)
{
	unsigned char i;// usec counter
	unsigned char k;
	
	k=SFRPAGE;
	SFRPAGE=0xf;
	// The input for Timer 4 is selected as SYSCLK by setting bit 0 of CKCON1:
	CKCON1|=0b_0000_0001;
	
	TMR4RL = 65536-(SYSCLK/1000L); // Set Timer4 to overflow in 1 ms.
	TMR4 = TMR4RL;                 // Initialize Timer4 for first overflow
	
	TMR4CN = 0x04;                 // Start Timer4 and clear overflow flag
	for (i = 0; i < ms; i++)       // Count <ms> overflows
	{
		while (!(TMR4CN & 0x80));  // Wait for overflow
		TMR4CN &= ~(0x80);         // Clear overflow indicator
	}
	TMR4CN = 0 ;                   // Stop Timer4 and clear overflow flag
	SFRPAGE=k;	
}

void Timer2_ISR (void) interrupt 5
{
	unsigned char page_save;
	page_save = SFRPAGE;
	SFRPAGE = 0;
	
	TF2H = 0; // Clear Timer2 interrupt flag
	OUT0=!OUT0;
	OUT1=!OUT0;
	
	SFRPAGE=page_save;
}

void putchar1 (char c) {
	if (c == '\n' )	{
		while (!(SCON1 & 0x02));
			SCON1 &= ~0x02;
			SBUF1 = '\r' ;
	}
	while (!(SCON1 & 0x02));
	SCON1 &= ~0x02;
	SBUF1 = c;
}

char getchar1 (void) {
	char c;
	while (!(SCON1 & 0x01));
	SCON1 &= ~0x01;
	c = SBUF1;
	return (c);
}	

void I2C_write (unsigned char output_data)
{
	SMB0DAT = output_data; // Put data into buffer
	SI0 = 0;
	while (!SI0); // Wait until done with send
}

unsigned char I2C_read (void)
{
	unsigned char input_data;

	SI0 = 0;
	while (!SI0); // Wait until we have data to read
	input_data = SMB0DAT; // Read the data

	return input_data;
}

void I2C_start (void)
{
	ACK0 = 1;
	STA0 = 1;     // Send I2C start
	STO0 = 0;
	SI0 = 0;
	while (!SI0); // Wait until start sent
	STA0 = 0;     // Reset I2C start
}

void I2C_stop(void)
{
	STO0 = 1;  	// Perform I2C stop
	SI0 = 0;	// Clear SI
	//while (!SI0);	   // Wait until stop complete (Doesn't work???)
}

void nunchuck_init(bit print_extension_type)
{
	unsigned char i, buf[6];
	
	// Older initialization format that works only for older nunchucks
	//I2C_start();
	//I2C_write(0xA4);
	//I2C_write(0x40);
	//I2C_write(0x00);
	//I2C_stop();
	
	// Newer initialization format that works for all nunchucks
	I2C_start();
	I2C_write(0xA4);
	I2C_write(0xF0);
	I2C_write(0x55);
	I2C_stop();
	Timer4ms(1);
	 
	I2C_start();
	I2C_write(0xA4);
	I2C_write(0xFB);
	I2C_write(0x00);
	I2C_stop();
	Timer4ms(1);

	// Read the extension type from the register block.  For the original Nunchuk it should be
	// 00 00 a4 20 00 00.
	I2C_start();
	I2C_write(0xA4);
	I2C_write(0xFA); // extension type register
	I2C_stop();
	Timer4ms(3); // 3 ms required to complete acquisition

	I2C_start();
	I2C_write(0xA5);
	
	// Receive values
	for(i=0; i<6; i++)
	{
		buf[i]=I2C_read();
	}
	ACK0=0;
	I2C_stop();
	Timer4ms(3);
	
	if(print_extension_type)
	{
		printf("Extension type: %02x  %02x  %02x  %02x  %02x  %02x\n", 
			buf[0],  buf[1], buf[2], buf[3], buf[4], buf[5]);
	}

	// Send the crypto key (zeros), in 3 blocks of 6, 6 & 4.

	I2C_start();
	I2C_write(0xA4);
	I2C_write(0xF0);
	I2C_write(0xAA);
	I2C_stop();
	Timer4ms(1);

	I2C_start();
	I2C_write(0xA4);
	I2C_write(0x40);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_stop();
	Timer4ms(1);

	I2C_start();
	I2C_write(0xA4);
	I2C_write(0x40);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_stop();
	Timer4ms(1);

	I2C_start();
	I2C_write(0xA4);
	I2C_write(0x40);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_write(0x00);
	I2C_stop();
	Timer4ms(1);
	
}

void nunchuck_getdata(unsigned char * s)
{
	unsigned char i;

	// Start measurement
	I2C_start();
	I2C_write(0xA4);
	I2C_write(0x00);
	I2C_stop();
	Timer4ms(3); 	// 3 ms required to complete acquisition

	// Request values
	I2C_start();
	I2C_write(0xA5);
	
	// Receive values
	for(i=0; i<6; i++)
	{
		s[i]=(I2C_read()^0x17)+0x17; // Read and decrypt
	}
	ACK0=0;
	I2C_stop();
}

void main (void)
{
	unsigned char rbuf[6];
 	int joy_x, joy_y, off_x, off_y, acc_x, acc_y, acc_z;
 	bit but1, but2;
 	int max_left, max_right, max_up, max_down, auto_flag = 0;

	printf("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	printf("\n\nWII Nunchuck I2C Reader\n");

	Timer4ms(200);
	nunchuck_init(1);
	Timer4ms(100);

	nunchuck_getdata(rbuf);

	off_x=(int)rbuf[0]-128;
	off_y=(int)rbuf[1]-128;
	printf("Offset_X:%4d Offset_Y:%4d\n\n", off_x, off_y);
	
	/*---------------*/
	/*  Calibration  */
	/*---------------*/
	printf("Calibrating controller: Move the joystick to its maximum and click C.\n");
	while(1)
	{
		nunchuck_getdata(rbuf);
		joy_x=(int)rbuf[0]-128-off_x;
		but2=(rbuf[5] & 0x02)?1:0;
		
		printf("Left: %4d \x1b[0J\r", joy_x);
		if(!but2)
		{
			max_left = joy_x;
			while(!but2)
			{
				Timer4ms(100);
				nunchuck_getdata(rbuf);
				but2=(rbuf[5] & 0x02)?1:0;
			}
			Timer4ms(100);
			break;
		}
		Timer4ms(100);
	}
	printf("\n");
	while(1)
	{
		nunchuck_getdata(rbuf);
		joy_x=(int)rbuf[0]-128-off_x;
		but2=(rbuf[5] & 0x02)?1:0;
		
		printf("Right: %4d \x1b[0J\r", joy_x);
		if(!but2)
		{
			max_right = joy_x;
			while(!but2)
			{
				Timer4ms(100);
				nunchuck_getdata(rbuf);
				but2=(rbuf[5] & 0x02)?1:0;
			}
			Timer4ms(100);
			break;
		}
		Timer4ms(100);
	}
	printf("\n");
	while(1)
	{
		nunchuck_getdata(rbuf);
		joy_y=(int)rbuf[1]-128-off_y;
		but2=(rbuf[5] & 0x02)?1:0;
		
		printf("Up: %4d \x1b[0J\r", joy_y);
		if(!but2)
		{
			max_up = joy_y;
			while(!but2)
			{
				Timer4ms(100);
				nunchuck_getdata(rbuf);
				but2=(rbuf[5] & 0x02)?1:0;
			}
			Timer4ms(100);
			break;
		}
		Timer4ms(100);
	}
	printf("\n");
	while(1)
	{	
		nunchuck_getdata(rbuf);
		joy_y=(int)rbuf[1]-128-off_y;
		but2=(rbuf[5] & 0x02)?1:0;
		
		printf("Down: %4d \x1b[0J\r", joy_y);
		if(!but2)
		{
			max_down = joy_y;
			while(!but2)
			{
				Timer4ms(100);
				nunchuck_getdata(rbuf);
				but2=(rbuf[5] & 0x02)?1:0;
			}
			Timer4ms(100);
			break;
		}
		Timer4ms(100);
	}
	printf("\nCalibration complete.\nL: %4d  R: %4d  U: %4d  D: %4d\n\n", max_left, max_right, max_up, max_down);
	Timer4ms(500);
	/*---------------*/
	
	TR2=1; // Start timer 2

	while(1)
	{
	
		nunchuck_getdata(rbuf);

		joy_x=(int)rbuf[0]-128-off_x;
		joy_y=(int)rbuf[1]-128-off_y;
		acc_x=rbuf[2]*4; 
		acc_y=rbuf[3]*4;
		acc_z=rbuf[4]*4;

		but1=(rbuf[5] & 0x01)?1:0;
		but2=(rbuf[5] & 0x02)?1:0;
		if (rbuf[5] & 0x04) acc_x+=2;
		if (rbuf[5] & 0x08) acc_x+=1;
		if (rbuf[5] & 0x10) acc_y+=2;
		if (rbuf[5] & 0x20) acc_y+=1;
		if (rbuf[5] & 0x40) acc_z+=2;
		if (rbuf[5] & 0x80) acc_z+=1;
		
		printf("Buttons(Z:%c, C:%c) Joystick(%4d, %4d) Accelerometer(%3d, %3d, %3d)\x1b[0J\r",
			   but1?'1':'0', but2?'1':'0', joy_x, joy_y, acc_x, acc_y, acc_z);
		
		
		/*------SIGNAL OUTPUT------*/
		
		if (auto_flag)								// if auto mode is on
		{
			putchar1(AUTO);							// output signal is auto signal
			if (!but2)								// if C pressed, then should exit auto mode
			{
				Timer4ms(50);						// button debounce
				while(!but2)						// while C is pressed, just hold
				{
					Timer4ms(100);
					nunchuck_getdata(rbuf);			// retrieves data (checks if C is still pressed)
					but2=(rbuf[5] & 0x02)?1:0;
				}
				auto_flag=0;						// clear auto flag to exit automode
				putchar1(0x80);						// output drive mode
			}
		}
		else if(!but1)								// if Z pressed, go to aim
		{
			Timer4ms(50);
			while(!but1)							// while Z is held
			{
				Timer4ms(200);
				
				nunchuck_getdata(rbuf);				// retrieves joystick and button data

				joy_x=(int)rbuf[0]-128-off_x;
				joy_y=(int)rbuf[1]-128-off_y;

				but1=(rbuf[5] & 0x01)?1:0;
				but2=(rbuf[5] & 0x02)?1:0;
				//aiming
				printf("\rAIMING");
				if(!but2)							// if C pressed, shoot
				{
					Timer4ms(50);
					while(!but2)					// while C is held, don't do anything
					{
						Timer4ms(200);
						nunchuck_getdata(rbuf);		// constantly retrieve data to check if C still pressed
						but2=(rbuf[5] & 0x02)?1:0;
					}
					putchar1(AIM_S);				// send shoot signal
				}
				else if(joy_x < 0)					// when joystick is moved left
				{
					if(joy_y < 0)
						putchar1(AIM_DL);
					else if (joy_y > 0)
						putchar1(AIM_UL);
					else 
						putchar1(AIM_L);
				}
				else if(joy_x > 0)			// when joystick moved right
				{
					if(joy_y < 0)
						putchar1(AIM_DR);
					else if (joy_y > 0)
						putchar1(AIM_UR);
					else
						putchar1(AIM_R);
				}
				else if(joy_y > 0)			// up
					putchar1(AIM_U);
				else if(joy_y < 0)			// down
					putchar1(AIM_D);
				else
					putchar1(AIM_DEF);		// AIM signal, 0b01000000
			}
		}
		else if(!but2)						// if C pressed, change to auto mode
		{
			Timer4ms(50);
			while(!but2)					// while C is pressed, just hold
			{
				//Timer4ms(100);
				nunchuck_getdata(rbuf);		// constantly retireve data to check if C still held
				but2=(rbuf[5] & 0x02)?1:0;
			}
			putchar1(AUTO);					// send auto signal
			auto_flag = 1;					// set auto flag
		}
		else if(joy_x < 0)					// if joystick left
		{
			if(joy_y < 0)
				putchar1(MOV_DL);
			else if (joy_y > 0)
				putchar1(MOV_UL);
			else 
				putchar1(MOV_L);
		}
		else if(joy_x > 0)					// right
		{
			if(joy_y < 0)
				putchar1(MOV_DR);
			else if (joy_y > 0)
				putchar1(MOV_UR);
			else
				putchar1(MOV_R);
		}
		else if(joy_y > 0)					// up
			putchar1(MOV_U);
		else if(joy_y < 0)					// down
			putchar1(MOV_D);
		else
			putchar1(MOV_DEF);				// default signal is MOV 0b10000000
		/*---------------*/
		
		
		Timer4ms(200);

   }
}
