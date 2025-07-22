#include <EFM8LB1.h>
#include <stdlib.h>
#include <stdio.h>
#include "EFM8_JDY40_test.h"
//#include "Project2_servos.h"


volatile unsigned int motor_counter=0;
volatile unsigned char pwm1=40, pwm2=65, motorsignal1=0, motorsignal2=0;

volatile unsigned int servo_counter=0;
volatile unsigned char baseservo=175, armservo=240;

//volatile unsigned char sonar = 150;
//volatile unsigned char sonar_counter = 0;

volatile unsigned int pickup_count = 0;
//#define COLPITTS P2_1
#define SERVO1   P1_7 // ARM
#define SERVO2   P1_5 // BASE
#define EMAGNET  P1_4
//#define MOTOR1     P1_6
//#define MOTOR2     P1_7
#define PERIOD_PIN P2_1
#define LEFT1     P1_0
#define RIGHT1      P1_1
#define LEFT2     P1_2
#define RIGHT2      P1_3
//#define OUTPIN5    P1_4
#define BOOT       P3_7

#define ECHO P0_1
#define TRIG P1_6


#define SYSCLK 72000000L // SYSCLK frequency in Hz
#define BAUDRATE 115200L
#define SARCLK 18000000L
#define RELOAD_10us (0x10000L-(SYSCLK/(12L*100000L))) // 10us rate

char _c51_external_startup (void)
{
	// Disable Watchdog with key sequence
	SFRPAGE = 0x00;
	WDTCN = 0xDE; //First key
	WDTCN = 0xAD; //Second key
  
	VDM0CN |= 0x80;
	RSTSRC=0x02|0x04;  // Enable reset on missing clock detector and VDD

	#if (SYSCLK == 48000000L)	
		SFRPAGE = 0x10;
		PFE0CN  = 0x10; // SYSCLK < 50 MHz.
		SFRPAGE = 0x00;
	#elif (SYSCLK == 72000000L)
		SFRPAGE = 0x10;
		PFE0CN  = 0x20; // SYSCLK < 75 MHz.
		SFRPAGE = 0x00;
	#endif
	
	#if (SYSCLK == 12250000L)
		CLKSEL = 0x10;
		CLKSEL = 0x10;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 24500000L)
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 48000000L)	
		// Before setting clock to 48 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x07;
		CLKSEL = 0x07;
		while ((CLKSEL & 0x80) == 0);
	#elif (SYSCLK == 72000000L)
		// Before setting clock to 72 MHz, must transition to 24.5 MHz first
		CLKSEL = 0x00;
		CLKSEL = 0x00;
		while ((CLKSEL & 0x80) == 0);
		CLKSEL = 0x03;
		CLKSEL = 0x03;
		while ((CLKSEL & 0x80) == 0);
	#else
		#error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
	#endif
	
	// Configure the pins used as outputs
	P1MDOUT |= 0b_1111_1111; // SERVO2, SERVO1, OUPTUT1 to OUTPUT5
	P0MDOUT |= 0x11; // Enable UART0 TX (P0.4) and UART1 TX (P0.0) as push-pull outputs
	P2MDOUT |= 0x01; // P2.0 in push-pull mode
	XBR0     = 0x01; // Enable UART0 on P0.4(TX) and P0.5(RX)                     
	XBR1     = 0X00; // 
	XBR2     = 0x41; // Enable crossbar and uart 1//was0x040


	#if (((SYSCLK/BAUDRATE)/(2L*12L))>0xFFL)
		#error Timer 0 reload value is incorrect because (SYSCLK/BAUDRATE)/(2L*12L) > 0xFF
	#endif
	// Configure Uart 0
	SCON0 = 0x10;
	CKCON0 |= 0b_0000_0000 ; // Timer 1 uses the system clock divided by 12.
	TH1 = 0x100-((SYSCLK/BAUDRATE)/(2L*12L));
	TL1 = TH1;      // Init Timer1
	TMOD &= ~0xf0;  // TMOD: timer 1 in 8-bit auto-reload
	TMOD |=  0x20;                       
	TR1 = 1; // START Timer1
	TI = 1;  // Indicate TX0 ready

	P2_0=1; // 'set' pin to 1 is normal operation mode.
	// Initialize timer 5 for periodic interrupts
	SFRPAGE=0x10;
	TMR5CN0=0x00;
	TMR5=0xffff;   // Set to reload immediately
	EIE2|=0b_0000_1000; // Enable Timer5 interrupts
	TR5=1;         // Start Timer5 (TMR5CN0 is bit addressable)
	
	EA=1;
	
	SFRPAGE=0x00;
	
	return 0;
}

void InitADC (void)
{
	SFRPAGE = 0x00;
	ADEN=0; // Disable ADC
	
	ADC0CN1=
		(0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift. 0x1: Shift right 1 bit. 0x2: Shift right 2 bits. 0x3: Shift right 3 bits.		
		(0x0 << 0) ; // Accumulate n conversions: 0x0: 1, 0x1:4, 0x2:8, 0x3:16, 0x4:32
	
	ADC0CF0=
	    ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider. Max is 18MHz. Fsarclk = (Fadcclk) / (ADSC + 1)
		(0x0 << 2); // 0:SYSCLK ADCCLK = SYSCLK. 1:HFOSC0 ADCCLK = HFOSC0.
	
	ADC0CF1=
		(0 << 7)   | // 0: Disable low power mode. 1: Enable low power mode.
		(0x1E << 0); // Conversion Tracking Time. Tadtk = ADTK / (Fsarclk)
	
	ADC0CN0 =
		(0x0 << 7) | // ADEN. 0: Disable ADC0. 1: Enable ADC0.
		(0x0 << 6) | // IPOEN. 0: Keep ADC powered on when ADEN is 1. 1: Power down when ADC is idle.
		(0x0 << 5) | // ADINT. Set by hardware upon completion of a data conversion. Must be cleared by firmware.
		(0x0 << 4) | // ADBUSY. Writing 1 to this bit initiates an ADC conversion when ADCM = 000. This bit should not be polled to indicate when a conversion is complete. Instead, the ADINT bit should be used when polling for conversion completion.
		(0x0 << 3) | // ADWINT. Set by hardware when the contents of ADC0H:ADC0L fall within the window specified by ADC0GTH:ADC0GTL and ADC0LTH:ADC0LTL. Can trigger an interrupt. Must be cleared by firmware.
		(0x0 << 2) | // ADGN (Gain Control). 0x0: PGA gain=1. 0x1: PGA gain=0.75. 0x2: PGA gain=0.5. 0x3: PGA gain=0.25.
		(0x0 << 0) ; // TEMPE. 0: Disable the Temperature Sensor. 1: Enable the Temperature Sensor.

	ADC0CF2= 
		(0x0 << 7) | // GNDSL. 0: reference is the GND pin. 1: reference is the AGND pin.
		(0x1 << 5) | // REFSL. 0x0: VREF pin (external or on-chip). 0x1: VDD pin. 0x2: 1.8V. 0x3: internal voltage reference.
		(0x1F << 0); // ADPWR. Power Up Delay Time. Tpwrtime = ((4 * (ADPWR + 1)) + 2) / (Fadcclk)
	
	ADC0CN2 =
		(0x0 << 7) | // PACEN. 0x0: The ADC accumulator is over-written.  0x1: The ADC accumulator adds to results.
		(0x0 << 0) ; // ADCM. 0x0: ADBUSY, 0x1: TIMER0, 0x2: TIMER2, 0x3: TIMER3, 0x4: CNVSTR, 0x5: CEX5, 0x6: TIMER4, 0x7: TIMER5, 0x8: CLU0, 0x9: CLU1, 0xA: CLU2, 0xB: CLU3

	ADEN=1; // Enable ADC
}

void InitPinADC (unsigned char portno, unsigned char pin_num)
{
	unsigned char mask;
	
	mask=1<<pin_num;

	SFRPAGE = 0x20;
	switch (portno)
	{
		case 0:
			P0MDIN &= (~mask); // Set pin as analog input
			P0SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 1:
			P1MDIN &= (~mask); // Set pin as analog input
			P1SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		case 2:
			P2MDIN &= (~mask); // Set pin as analog input
			P2SKIP |= mask; // Skip Crossbar decoding for this pin
		break;
		default:
		break;
	}
	SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
	ADC0MX = pin;   // Select input from pin
	ADINT = 0;
	ADBUSY = 1;     // Convert voltage at the pin
	while (!ADINT); // Wait for conversion to complete
	return (ADC0);
}

void Timer5_ISR (void) interrupt INTERRUPT_TIMER5
{
	SFRPAGE=0x10;
	TF5H = 0; // Clear Timer5 interrupt flag
	TMR5RL=RELOAD_10us;
	motor_counter++;
	if(motor_counter==100)
	{
		motor_counter=0;
	}
	if(pwm1>=motor_counter)
	{
		motorsignal1=1;
	}
	else
	{
		motorsignal1=0;
	}
	if(pwm2>=motor_counter)
	{
		motorsignal2=1;
	}
	else
	{
		motorsignal2=0;
	}

	servo_counter++;
	if(servo_counter==2000)
	{
		servo_counter=0;
	}
	if(baseservo>=servo_counter)
	{
		SERVO1=1;
	}
	else
	{
		SERVO1=0;
	}
	if(armservo>=servo_counter)
	{
		SERVO2=1;
	}
	else
	{
		SERVO2=0;
	}
	
	/*sonar trigger pulse
	if (sonar_counter ==2000)
	{
		sonar_counter = 0;
	}
	if (sonar >= sonar_counter)
	{
		TRIG = 1;
	}
	else
	{
		TRIG = 0;
	}
	*/
	
}

// Uses Timer3 to delay <us> micro-seconds. 
void Timer3us(unsigned char us)
{
	unsigned char i;               // usec counter
	
	// The input for Timer 3 is selected as SYSCLK by setting T3ML (bit 6) of CKCON0:
	CKCON0|=0b_0100_0000;
	
	TMR3RL = (-(SYSCLK)/1000000L); // Set Timer3 to overflow in 1us.
	TMR3 = TMR3RL;                 // Initialize Timer3 for first overflow
	
	TMR3CN0 = 0x04;                 // Sart Timer3 and clear overflow flag
	for (i = 0; i < us; i++)       // Count <us> overflows
	{
		while (!(TMR3CN0 & 0x80));  // Wait for overflow
		TMR3CN0 &= ~(0x80);         // Clear overflow indicator
	}
	TMR3CN0 = 0 ;                   // Stop Timer3 and clear overflow flag
}

void waitms (unsigned int ms)
{
	unsigned int j;
	for(j=ms; j!=0; j--)
	{
		Timer3us(249);
		Timer3us(249);
		Timer3us(249);
		Timer3us(250);
	}
}

// Measure the period of a square signal at PERIOD_PIN

unsigned long GetPeriod (int n)
{
	unsigned int overflow_count;
	unsigned char i;
	
	TR0=0; // Stop Timer/Counter 0
	TMOD&=0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD|=0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer

	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1;
	while(PERIOD_PIN!=0) // Wait for the signal to be zero
	{
		if(TF0==1) // Did the 16-bit timer overflow?
		{
			TF0=0;
			overflow_count++;
			if(overflow_count==10) // If it overflows too many times assume no signal is present
			{
				TR0=0;
				return 0; // No signal
			}
		}
	}
	
	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1;
	while(PERIOD_PIN!=1) // Wait for the signal to be one
	{
		if(TF0==1) // Did the 16-bit timer overflow?
		{
			TF0=0;
			overflow_count++;
			if(overflow_count==10) // If it overflows too many times assume no signal is present
			{
				TR0=0;
				return 0; // No signal
			}
		}
	}
	
	// Reset the counter
	TR0=0;
	TL0=0; TH0=0; TF0=0; overflow_count=0;
	TR0=1; // Start the timer
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while(PERIOD_PIN!=0) // Wait for the signal to be zero
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
			}
		}
		while(PERIOD_PIN!=1) // Wait for the signal to be one
		{
			if(TF0==1) // Did the 16-bit timer overflow?
			{
				TF0=0;
				overflow_count++;
			}
		}
	}
	TR0=0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period in clock cycles!
	
	return (overflow_count*65536+TH0*256+TL0);
}

/*SONAR
unsigned long Get_Sonar(void)
{
	volatile unsigned char overflow_count;
	sonar = 20;

	TR0 = 0; // Stop Timer/Counter 0
	TMOD &= 0b_1111_0000; // Set the bits of Timer/Counter 0 to zero
	TMOD |= 0b_0000_0001; // Timer/Counter 0 used as a 16-bit timer
	// Reset the counter
	TL0 = 0;
	TH0 = 0;
	TF0 = 0;
	overflow_count = 0;

	while (ECHO != 0); // Wait for the signal to be zero
	while (ECHO != 1); // Wait for the signal to be one
	TR0 = 1; // Start the timer
	while (ECHO != 0) // Wait for the signal to be zero
	{
		if (TF0 == 1) // Did the 16-bit timer overflow?
		{
			TF0 = 0;
			overflow_count++;
		}
	}
	//	while(ECHO!=1) // Wait for the signal to be one
	{
		if (TF0 == 1) // Did the 16-bit timer overflow?
		{
			TF0 = 0;
			overflow_count++;
		}
	}
	//
	TR0 = 0; // Stop timer 0, the 24-bit number [overflow_count-TH0-TL0] has the period!
	return 3.3 * (overflow_count * 65536.0 + TH0 * 256.0 + TL0) * (12.0 / SYSCLK);
}
*/

void eputs(char *String)
{	
	while(*String)
	{
		putchar(*String);
		String++;
	}
}

void PrintNumber(long int val, int Base, int digits)
{ 
	code const char HexDigit[]="0123456789ABCDEF";
	int j;
	#define NBITS 32
	xdata char buff[NBITS+1];
	buff[NBITS]=0;
	
	if(val<0)
	{
		putchar('-');
		val*=-1;
	}

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	eputs(&buff[j+1]);
}
//Takes position information from the master and uses it to control the robot.
void manual_control(int x, int y){
	if (x == 1 && y == 0) //left turn
		{
		LEFT1 = 0;
		LEFT2 = 0;
		RIGHT1 = 1;
		RIGHT2 = 0;
		}
		else if (x == 2 && y == 0) //right turn 
		{
		LEFT1 = 1;
		LEFT2 = 0;
		RIGHT1 = 0;
		RIGHT2 = 0;
		}
		else if (x == 1 && y == 1) //curve left
		{
		LEFT1 = motorsignal1;
		LEFT2 = 0;
		RIGHT1 =motorsignal2;
		RIGHT2 = 0;
		}
		else if (x == 2 && y == 1) //curve right
		{
		LEFT1 = motorsignal2;
		LEFT2 = 0;
		RIGHT1 = motorsignal1;
		RIGHT2 = 0;
		}
		else if (x == 1 && y == 2) //reverse curve left
		{
		LEFT1 = 0;
		LEFT2 = motorsignal1;
		RIGHT1 = 0;
		RIGHT2 = motorsignal2;
		}
		else if (x == 2 && y == 2) //reverse curve right
		{
		LEFT1 = 0;
		LEFT2 = motorsignal2;
		RIGHT1 = 0;
		RIGHT2 = motorsignal1;
		}
		else if (x == 0 && y == 1) //fwd
		{
		LEFT1 = 1;
		LEFT2 = 0;
		RIGHT1 = 1;
		RIGHT2 = 0;
		}
		else if (x == 0 && y == 2) //rev
		{
		LEFT1 = 0;
		LEFT2 = 1;
		RIGHT1 = 0;
		RIGHT2 = 1;
		}
		else //do nothing
		{
		LEFT1 = 0;
		LEFT2 = 0;
		RIGHT1 = 0;
		RIGHT2 = 0;
		}
}
//Takes in the ADC value at pin 2.2 and makes the robot reverse and turn if over a certain threshold.
void edge_detection_response(long int j){
	
		
		if (j > 5500)
        {
        
                LEFT1=0;
                LEFT2=1;
                RIGHT1=0;
                RIGHT2=1;
                
                waitms(250);
                waitms(250);
            
                LEFT1=0;
                LEFT2=0;
                RIGHT1=1;
                RIGHT2=0;
                
                waitms(250);
                waitms(250);
                waitms(250);
                waitms(250);     
        }
		
}

void switch_to_auto(int sec, char * buff){
	int i;

	sprintf(buff, "M%d\n", sec);
	sendstr1(buff);
	
	for (i = 0; i < sec; i++)
	{
		waitms(1000);
	}
}
void moveBaseServo(unsigned char start, unsigned char end, unsigned char step)
{	
	unsigned char j;
	eputs("base\n");
	
    if (start < end) {
        for (j = start; j < end; j += step) {
            baseservo = j;
            waitms(20);
        }
    } else {
        for (j = start; j > end; j -= step) {
            baseservo = j;
            waitms(20);
        }
    }
}


void moveArmServo(unsigned char start, unsigned char end, unsigned char step)
{
	unsigned char j;
	eputs("arm\n");
    
    if (start < end) {
        for (j = start; j < end; j += step) {
            armservo = j;
            waitms(20);
        }
    } else {
        for (j = start; j > end; j -= step) {
            armservo = j;
            waitms(20);
        }
    }
}
void controlElectromagnet(unsigned char state)
{
	eputs("mag\n");
    EMAGNET = state;
    waitms(1000); // Optional delay to simulate some action time
}
// WARNING: do not use printf().  It makes the program big and slow!
void pick_up_coin (void){
	eputs("Coin\n");
	controlElectromagnet(1);	// magnet on

	
    moveArmServo(240, 188, 1);	// move arm servo
	
	// sweep around
    moveBaseServo(175, 240, 1);
    moveBaseServo(240, 145, 1);	
    moveBaseServo(145, 175, 1);	// move base servo to original position
	
	// put into basket
    moveArmServo(188, 30, 1);	// move arm servo
    moveBaseServo(175, 130, 1);	// move base servo
    waitms(500);
    controlElectromagnet(0);	// magnet off
    waitms(500);
    // return to original position
    moveBaseServo(130, 175, 1);	// move base servo
    moveArmServo(30, 240, 1);	// move arm servo

	pickup_count++;
}

// Function to control the electromagnet



void main (void)
{
	int position[2]; //position of the joystick
	char buff[10]; //standard buffer for read/ write
    char c; // for the master codes
	int mode = 1; // for determining manual / automatic mode
	float m_period; // metal detection period.
	int temp = 0;
	int x =0; // discrete int for x rather than position[0]
	int y =0; // discrete int for y rather than position[1]
	long int p_param, p_voltage; //For the perimeter detection
    long int j, v; // Place holders, will likely be used for servo arm
	long int count, f; // used for housekeeping
	unsigned char LED_toggle=0; // Used to test the outputs
	unsigned char mcounter=0; // Used to test the outputs
	int i;
	long int total_period = 0;

	int startup = 1;
	int period_threshold = 0;
	UART1_Init(9600);
	ReceptionOff();

	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");

	// unique device ID.
	// number from 0x0000 to 0xFFFF. -BACC-
	SendATCommand("AT+DVID5005\r\n"); 
	
	InitPinADC(2, 2); // Configure P2.2 as analog input
	InitADC();
		
	LEFT1=0;
	LEFT2=0;
	RIGHT1=0;
	RIGHT2=0;
	controlElectromagnet(0);
	
	//pick_up_coin();
	
	while (1)
	{
		c = getchar1();
		if (c == 'C')
		{
			break;
		}
		
	}
	

	
	while (1)
	{
		if (pickup_count >= 20)
		{
			while (1)
			{
				c = getchar1();
				if (c == 'C')
				{
					break;
				}
				else{
					sprintf(buff, "X\n");
					sendstr1(buff);
				}
			}
			
		}
		
		if(RXU1()) // Something has arrived
		{
			c=getchar1();
			//P is the master code for updating position -> sent continously
			if (c == 'P') // Master wants to update position
			{
				//memset(buff, 0, sizeof(buff));
				getstr1(buff, sizeof(buff)-1);
				
				position[0] = buff[0] - '0'; //ascii convert
				position[1] = buff[1] - '0'; //ascii convert
				//sprintf(buff,"y%dx%d\n",position[1],position[0]);
				
				
			}
			//M is the master code for changing the mode -> keyed to button press
			else if (c == 'M')
			{
				getstr1(buff, sizeof(buff)-1);
				//eputs(buff);
				//We flip the mode from 0-> 1 or 1 -> 0
				mode ^= 1;
				if(!mode){
					int i;
					for (i = 0; i < 5; i++)//atoi(buff) Matches the delay of the animation on the remote. could use this to declare baselines.
					{
						waitms(1000);
					}
				}
				//We send a pulse of mode readings- Helps the signal not get lost
				//its important the mode stay the same between the master and slave
				//slave drives mode, as it always updates on the remote, but only sometimes on the bot.
				for (i = 0; i < 10; i++)
				{
					waitms(5); // The radio seems to need this delay...
					sprintf(buff, "M%d\n", mode);
					sendstr1(buff);
				}
			}
			//Sending an a is how the master requests info on the period
			else if(c=='A') // Master wants slave data
			{	
				//EA=0;
				//m_period = GetPeriod(10); //Getting the period with a sample rate of 10, to keep things quick.
				//EA=1;
				sprintf(buff, "P%d\n", (int)(100000000/m_period));
				waitms(5); // The radio seems to need this delay...
				sendstr1(buff);
			}
			else if(c == 'C'){
				if(mode){
					pick_up_coin();
				}
			}
		}
		
	
		m_period = GetPeriod(50); //Getting the period with a sample rate of 10, to keep things quick.
		
	
		if(mode){
			x = position[0];
			y = position[1];
			manual_control(x,y);
		}
		//Automatic control
		//will intiate forward move until the permiter is detected
		else{
			
			LEFT1 = 1;
			LEFT2 = 0;
			RIGHT1 = 1;
			RIGHT2 = 0;
			
			if(startup){
				
				
				period_threshold = (int)(100000000/GetPeriod(50));
				
				
				waitms(5);
				
				startup = 0;
				
			}
			
			
			p_param=ADC_at_Pin(QFP32_MUX_P2_2);
    		p_voltage=(j*33000)/0x3fff;
			edge_detection_response(p_param);
			m_period = GetPeriod(50); //Getting the period with a sample rate of 10, to keep things quick.
			//sprintf(buff, "P%d\n", (int)period_threshold);
			
			
			if ((int)(100000000/GetPeriod(50)) - period_threshold > 250 )
			{
				
				sprintf(buff, "B");
				sendstr1(buff);

				LEFT1=0;
				LEFT2=1;
				RIGHT1=0;
				RIGHT2=1;
				
				waitms(150);
				LEFT1=0;
				LEFT2=0;
				RIGHT1=0;
				RIGHT2=0;
				pick_up_coin();
				
				
			}
				
		}
		
	}
}
