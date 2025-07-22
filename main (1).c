#include "../Common/Include/stm32l051xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../Common/Include/serial.h"
#include "UART2.h"
#include "adc.h"
#include "lcd.h"
#include <math.h>
#define SYSCLK 32000000L
#define DEF_F 15000L

// LQFP32 pinout
//             ----------
//       VDD -|1       32|- VSS
//      PC14 -|2       31|- BOOT0
//      PC15 -|3       30|- PB7
//      NRST -|4       29|- PB6
//      VDDA -|5       28|- PB5
//       PA0 -|6       27|- PB4
//       PA1 -|7       26|- PB3
//       PA2 -|8       25|- PA15 (Used for RXD of UART2, connects to TXD of JDY40)
//       PA3 -|9       24|- PA14 (Used for TXD of UART2, connects to RXD of JDY40)
//       PA4 -|10      23|- PA13 (Used for SET of JDY40)
//       PA5 -|11      22|- PA12
//       PA6 -|12      21|- PA11
//       PA7 -|13      20|- PA10 (Reserved for RXD of UART1)
//       PB0 -|14      19|- PA9  (Reserved for TXD of UART1)
//       PB1 -|15      18|- PA8  (pushbutton)
//       VSS -|16      17|- VDD
//             ----------

#define F_CPU 32000000L
#define DEF_L 10000L // 1ms tick
volatile int timer_counter = 0;
volatile int enable_alarm = 0;
volatile int alarm_scalar = 1;
volatile int freq;
volatile int baseline = 0;
#define PA8_0 (GPIOA->ODR &= ~BIT8)
#define PA8_1 (GPIOA->ODR |= BIT8)

#define PA7 (GPIOA -> IDR & BIT7) //Button 1
#define PA6 (GPIOA -> IDR & BIT6) //joystick

void TIM2_Handler(void){
	TIM2->SR &= ~BIT0; // clear update interrupt flag
	timer_counter++;
	if(timer_counter > 2000){
		timer_counter = 0;
	}
	
	
	if(enable_alarm){
		//might need another timer running here 
		if (timer_counter > (int)(10000000/(5*freq)))
		{
			GPIOA->ODR ^= BIT8;
		}
	}else{
		PA8_0;
	}
	
	
	
}

void Hardware_Init(void)
{

	// Configure the pin used for analog input: PB0 and PB1 (pins 14 and 15)
	GPIOB->MODER |= (BIT0|BIT1);  // Select analog mode for PB0 (pin 14 of LQFP32 package)
	GPIOB->MODER |= (BIT2|BIT3);  // Select analog mode for PB1 (pin 15 of LQFP32 package)
	GPIOB->MODER |= (BIT6|BIT7);  // Select analog mode for PB3 (pin 26 of LQFP32 package)
	initADC();

	GPIOA->OSPEEDR=0xffffffff; // All pins of port A configured for very high speed! Page 201 of RM0451

	RCC->IOPENR |= BIT0; // peripheral clock enable for port A

    GPIOA->MODER = (GPIOA->MODER & ~(BIT27|BIT26)) | BIT26; // Make pin PA13 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.

	GPIOA->MODER = (GPIOA->MODER & ~(BIT17|BIT16)) | BIT16; // Make pin PA8 output (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0))

	/*
	GPIOA->MODER &= ~(BIT16 | BIT17); // Make pin PA8 input
	// Activate pull up for pin PA8:
	GPIOA->PUPDR |= BIT16; 
	GPIOA->PUPDR &= ~(BIT17);
	*/
	GPIOA->MODER &= ~(BIT14 | BIT15); // Make pin PA7 input
	// Activate pull up for pin PA7:
	GPIOA->PUPDR |= BIT14; 
	GPIOA->PUPDR &= ~(BIT15);

	GPIOA->MODER &= ~(BIT12 | BIT13); // Make pin PA6 input
	// Activate pull up for pin PA6:
	GPIOA->PUPDR |= BIT12; 
	GPIOA->PUPDR &= ~(BIT13);

	GPIOA->MODER = (GPIOA->MODER & ~(BIT10 | BIT11)) | BIT10; // Make pin PA5 output
	GPIOA->OTYPER &= ~BIT5;
	// Set up timer
	RCC->APB1ENR |= BIT0;  // turn on clock for timer2 (UM: page 177)
	TIM2->ARR = F_CPU/DEF_L-1;
	NVIC->ISER[0] |= BIT15; // enable timer 2 interrupts in the NVIC
	TIM2->CR1 |= BIT4;      // Downcounting    
	TIM2->CR1 |= BIT7;      // ARPE enable    
	TIM2->DIER |= BIT0;     // enable update event (reload event) interrupt 
	TIM2->CR1 |= BIT0;      // enable counting    
	
	__enable_irq();
	

}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2(s);
	egets2(buff, sizeof(buff)-1);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	waitms(10);
	printf("Response: %s", buff);
}

void Configure_Pins (void)
{
	RCC->IOPENR |= BIT0; // peripheral clock enable for port A
	
	// Make pins PA0 to PA5 outputs (page 200 of RM0451, two bits used to configure: bit0=1, bit1=0)
    GPIOA->MODER = (GPIOA->MODER & ~(BIT0|BIT1)) | BIT0; // PA0
	GPIOA->OTYPER &= ~BIT0; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT2|BIT3)) | BIT2; // PA1
	GPIOA->OTYPER &= ~BIT1; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT4|BIT5)) | BIT4; // PA2
	GPIOA->OTYPER &= ~BIT2; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT6|BIT7)) | BIT6; // PA3
	GPIOA->OTYPER &= ~BIT3; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT8|BIT9)) | BIT8; // PA4
	GPIOA->OTYPER &= ~BIT4; // Push-pull
    
    GPIOA->MODER = (GPIOA->MODER & ~(BIT10|BIT11)) | BIT10; // PA5
	GPIOA->OTYPER &= ~BIT5; // Push-pull
}

#define PA5_0 (GPIOA -> ODR &= ~ BIT5) //pa5 off
#define PA5_1 (GPIOA -> ODR |=   BIT5)//pa3 on
void ReceptionOff (void)
{
	GPIOA->ODR &= ~(BIT13); // 'set' pin to 0 is 'AT' mode.
	waitms(10);
	eputs2("AT+DVID0000\r\n"); // Some unused id, so that we get nothing in RXD1.
	waitms(10);
	GPIOA->ODR |= BIT13; // 'set' pin to 1 is normal operation mode.
	while (ReceivedBytes2()>0) egetc2(); // Clear FIFO
}
//this function will send an integer to the slave
void SendIntegerToSlave (int num){
	char buffer[80];
	sprintf(buffer, "%d\n", num);
	eputs2(buffer);
}
//this function will return an array of size 2 corresponding to the joystick position
//arr[0] is the x comp, arr[1] is the y comp
//for x: 1 -> left position, 2-> right position
//for y: 1 -> up position, 2-> down position
//for both: 0-> middle
void GetJoyStickPosition (int pos[2]){
	
	int read_x;
	int read_y;

	read_x = readADC(ADC_CHSELR_CHSEL9);
	read_y = readADC(ADC_CHSELR_CHSEL8);

	if (read_x < 10)
	{
		pos[0] = 2;
	}
	else if (read_x > 3000 && read_x<3150)
	{
		pos[0] = 0;
	}
	else if (read_x > 2730 && read_x < 2800)
	{
		pos[0] = 1;
	}
	
		
	
	
	if (read_y < 10)
	{
		pos[1] = 1;
	}
	else if (read_y > 3100 && read_y < 3200)
	{
		pos[1] = 0;
	}
	else if (read_y>2710 && read_y < 2780){
		pos[1] = 2;
	}
	
}
//this function will send an updated position to the slave with code information.
void SendJoyStickToSlave(int pos[2]){
	char buff[10];
	GetJoyStickPosition(pos);
	sprintf(buff,"%d%d\n",pos[0],pos[1]);
	  //sprintf(buff,"11");

	//eputc2('!'); // Send a message to the slave. First send the 'attention' character which is '!'
	// Wait a bit so the slave has a chance to get ready
	//waitms(5); // This may need adjustment depending on how busy is the slave
	eputc2('P'); //let slave know we are sending the position array.
	waitms(10);
	eputs2(buff);
}
void start_screen(void){

}
	
int main(void)
{
	printf("test\r\n");
	int j, v; //variables for ADC usage.
	int m_period; // Metal detection period
	int sec = 5; // Seconds Delay for the lcd animation (mode switch) linked to slave values
	int mode = 1; // mode tracker -> driven by slave
	int position[2]; //array to hold encoded postion of joystick
	char buff[80]; // character array for standard read / write
    int timeout_cnt=0; // for radio handeling
    int cont1=0, cont2=100; // For radio handeling
	PA5_0; // set the output pin PA5 to low
	int startup = 1;
	//------Configurations------
	Configure_Pins();
	LCD_4BIT();
	Hardware_Init();
	initUART2(9600);
	waitms(1000); // Give putty some time to start.
	ReceptionOff();
	//---------------------------

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	
	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xBACC
	SendATCommand("AT+DVID5005\r\n");
	sprintf(buff, "Press Joystick");
		LCDprint(buff, 1, 1);
		sprintf(buff, "To Start");
		LCDprint(buff, 2, 1);
		
	while (1)
	{
		if (!PA6)
		{
			eputc2('C');
			break;
		}
	}
		

		sprintf(buff, " ");
		LCDprint(buff, 2, 1);
		sprintf(buff, "  Manual  Mode");
		LCDprint(buff, 1, 1);
		sprintf(buff,"%d\n",sec);
		eputs2(buff);
	waitms(50);
	while(1)
	{	
		

		//Continously send encoded joystick info.
		SendJoyStickToSlave(position);
		
		if(baseline != 0){
			if(freq - baseline > 350){
				enable_alarm = 1;
			}
		}
		//Code for sending over radio
		if(++cont1>200) cont1=0; // Increment test counters for next message
		if(++cont2>200) cont2=0;
		waitms(5); // This may need adjustment depending on how busy the slave is

		
		//only enable metal detection feature during auto mode.
		if (!mode)
		{
			eputc2('A'); // Request period
			waitms(5);
		}
		
			
		
		if (!mode)
		{
			sprintf(buff, "  Auto  Mode");
			LCDprint(buff, 1, 1);
			sprintf(buff, " ");
			LCDprint(buff, 2, 1);

		}
		else{
			sprintf(buff, " ");
			LCDprint(buff, 2, 1);
			sprintf(buff, "  Manual  Mode");
			LCDprint(buff, 1, 1);
		}
		
		if (!PA7) {
			eputc2('M'); // ping mode change on button press
			
			printf("Button 1\r\n");
			if (mode)//checks if mode is one(in manual mode) and switches to auto.
			{
				//Display encoded joystick information and raw ADC on the lcd.
				sprintf(buff, " ");
				LCDprint(buff, 2, 1);
				sprintf(buff, "  Manual  Mode");
				LCDprint(buff, 1, 1);
				sprintf(buff,"%d\n",sec);
				eputs2(buff);

				//LCD animation based on the sec operator above.
				LCDprint(" ", 2, 1);
				LCDprint(" ", 1, 1);
				for (int i = 1; i <= sec; i++)
				{
					sprintf(buff, "Auto Start In: %d",(6 - i));
					LCDprint(buff, 1, 1);
					waitms(995); // Dont wait full second to account for transmission delay
					
				}
			}
		} 
		if (!PA6)
		{
			eputc2('C');
		}
		
	
		timeout_cnt=0;

		while(1)
		{
			if(ReceivedBytes2()>5) break; // Something has arrived
			if(++timeout_cnt>250) break; // Wait up to 25ms for the repply
			Delay_us(100); // 100us*250=25ms
		}
		
		if(ReceivedBytes2()>0) // Something has arrived from the slave
		{
			egets2(buff, sizeof(buff)-1);
			//The slave message will always have buff[0] as its slave code character.
			if(strlen(buff)<=10) // Check for valid message size (5 characters + new line '\n')
			{	
				
				//P is the slave code for sending the period measurment
				if(buff[0] == 'P'){
					printf("Slave says: %s\r", buff);
					freq = atoi(buff); //get the period using atoi
					if (startup)
					{
						baseline = atoi(buff);
						startup = 0;
					}
					
					
					
				}
				//M is the slave code for sending the mode state
				if (buff[0] == 'M')
				{
					mode = buff[1] - '0'; //get the mode using direct ascii convert
				}
				if (buff[0] == 'B')
				{
					
				}
				if(buff[0] == 'B'){

					enable_alarm = 1;

				}
				if (buff[0] == 'X')
				{
					sprintf(buff, "Auto Complete");
					LCDprint(buff, 1, 1);
					sprintf(buff, "Press Joystick");
					LCDprint(buff, 2, 1);
					while (1)
					{
						if (!PA6)
						{
							eputc2('C');
							break;
						}
					}
					

				}
				
				
			}
			else
			{
				while (ReceivedBytes2()) egetc2(); // Clear FIFO
				printf("*** BAD MESSAGE ***: %s\r", buff);
			}
		}
		else // Timed out waiting for reply
		{
			while (ReceivedBytes2()) egetc2(); // Clear FIFO
			//printf("NO RESPONSE\r\n", buff);
		}
		
		waitms(50);  // Set the information interchange pace: communicate about every 50ms
		
	}


}
