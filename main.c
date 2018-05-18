/*
 * Lab5.0.2
 *
 * Created: 4/5/2018 3:21:25 PM
 * Author : Jessica Brown and Skylar Chatman
 */ 
//pin 23 is ADC0 default ADC pin
#define F_CPU 8000000L // This should match the processor speed
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h> // Routine for FLASH (program memory)
#include <util/delay.h>
#include <string.h>
#include <stdio.h>
#include "i2cmaster.h"

//Baud rate and 8-bit voltage DAC definition
#define BAUD_RATE 9600 // Baud rate: the rate at which information is transferred in a communication channel
#define MAX518 0x58 // 8-bit voltage output digital-to-analog converters (DACs)

// Variables and #define for the RX ring buffer.
#define RX_BUFFER_SIZE 64
unsigned char rx_buffer[RX_BUFFER_SIZE]; 
volatile unsigned char rx_buffer_head; //read position of buffer
volatile unsigned char rx_buffer_tail; //write position of buffer

//variables used for obtaining voltage from micro controller and outputting to PC
float voltage; //holds the voltage

//variables used for user input commands
unsigned char command = NULL;
int n; //number of measurements (integer, 2<=n<=20)
char measurements[2]; //character array for number of measurements (because can be a 2 digit number)
int dt; //time between measurements (integer, 1,=dt<=60s)
char delaytime[2]; //char array for time between measurements
int c; //DAC channel number (integer, c<=={0,1})
char channel[2]; //char array for channel input
float outputvoltage; //output voltage (float, format: 'n.nn' V, needs to be converted into a decimal number that is sent to the DAC such that the quantization error is minimal)
char outVolt[4]; //char array for output voltage input
float finalVolt; //holds final voltage for the 'S' command

int f; //frequency of waveform (integer, f ? {1,5,10,15,20,25} Hz), the signal frequency must be within +/- 5%
int r; //number of consecutive waveform cycles to generate (integer, 1? r ? 1000)

//messages to print
const char msg1[] = "Enter command (options: G, M, S).\n"; 
const char msg2[] = "Enter number of measurements (integer, 2 ? n ? 20): \n";
const char msg3[] = "Enter time between measurements (integer, 1 ? dt ? 60 s): \n";
const char msg4[] = "Enter  DAC channel number (integer, c ? {0,1}): \n";
const char msg5[] = "Enter  output voltage (float, format: 'n.nn' V): \n";

// Function prototypes.
unsigned char uart_buffer_empty(void);
void usart_prints(const char *ptr);
void usart_printf(const char *ptr);
void usart_init();
void usart_putc(const char c);
unsigned char usart_getc(void);
void readADC();
extern unsigned char i2c_start(unsigned char addr);
float string2decimal(char ov[4]);
//main method
int main(void)
{
	/*while(1) 
	{
		i2c_init(); //initialize i2C
		i2c_start_wait(MAX518 + I2C_WRITE); //sends address and transfer direction
		i2c_write(0x01); //write address = 1
		i2c_write(255); //write output voltage to terminal
		i2c_stop(); //set stop condition = release bus
	}*/
	
	while(1){
		char str[25]; //holds the string which we want to print to SecureCRT
		int i,j; //counter
		//char voltageBuffer[8]; //will hold voltage
		sei(); // Enable interrupts
		
		usart_init(); // Initialize the USART
		adc_init();	// Initialize ADC conversions (analog to digital converter, 10-bit mode)
		i2c_init(); //initialize i2C
		
		usart_prints(msg1);
		for (i = 0;i<1;i++){
			command = usart_getc(); // Get character
		}
		//scan in command
		sscanf(command,"%s");
		
		//find voltage
		if(command == 'G'){
			readADC(&voltage);
			sprintf(str, "v = %.3f V\n\r", voltage);
			usart_prints(str);
		}
		//find multiple voltage
		else if(command == 'M')
		{
			usart_prints(msg2);
			for(i = 0; i <= 1; i++)
			{
				n = usart_getc();
				USART_Transmit(n);
				measurements[i] = n;
			}
			measurements[i] = '\0';
		
			usart_prints(msg3);
			for(j = 0; j <= 1; j++)
			{
				dt = usart_getc();
				USART_Transmit(dt);
				delaytime[j] = dt;
			}
			delaytime[j] = '\0';
		
			n = atoi(measurements); //convert string value to integer
			dt = atoi(delaytime); //convert string value to integer
			for(j = 0; j < n; j++)
			{
				readADC(&voltage);
				sprintf(str, "v = %.3f V\n\r", voltage);
				usart_prints(str);
			
				for(i = 0; i < dt; i++)
				{
					_delay_ms(1000);
				}
			}
		}
		//set DAC output voltage
		else if(command == 'S')
		{	
			i2c_init(); //initialize i2C
			i2c_start_wait(MAX518 + I2C_WRITE); //sends address and transfer direction
			
			usart_prints(msg4);
			for(j = 0; j <= 1; j++)
			{
				c = usart_getc();
				USART_Transmit(c);
				channel[j] = c;
			}
			channel[j] = '\0';

			//write to Ch0 or Ch1 based on user input
			if(channel[0] == 0){
				i2c_write(0x00); //write address = 0
			}
			else{
				i2c_write(0x01); //write address = 1
			}
			
			usart_prints(msg5);
			for(i = 0; i < 4; i++)
			{
				outputvoltage = usart_getc();
				USART_Transmit(outputvoltage);
				outVolt[i] = outputvoltage;
			}
			outVolt[i] = '\0';
			
			//now we have char array that holds the voltage we want to set
			//we need to convert this number into a floating point number
			outputvoltage = string2decimal(outVolt); //this should convert the string to a floating point number
			finalVolt = (outputvoltage*255.0)/5.0;
			i2c_write((int)finalVolt); //write output voltage to terminal
			i2c_stop(); //set stop condition = release bus
		}
	}

}

ISR(USART_RX_vect)
{
	// USART receive interrupt handler.
	// To do: check and warn if buffer overflows.
	
	char c = UDR0; //receive buffer UDRn
	rx_buffer[rx_buffer_head] = c; //first character of receive buffer
	if (rx_buffer_head == RX_BUFFER_SIZE - 1) //if buffer is at max
	rx_buffer_head = 0; //reset buffer
	else
	rx_buffer_head++; //count up in buffer
}

void usart_init(void)
{
	// Configures the USART for serial 8N1 with
	// the Baud rate controlled by a #define.
	unsigned short s;
	
	// Set Baud rate, controlled with #define above.
	s = (double)F_CPU / (BAUD_RATE*16.0) - 1.0;
	UBRR0H = (s & 0xFF00); //UBRRnH and UBRRnL are baud rate registers
	UBRR0L = (s & 0x00FF); //for high and low bits of baud rate

	// Receive complete interrupt enable: RXCIE0
	// Receiver & Transmitter enable: RXEN0,TXEN0
	UCSR0B = (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //USART control and status reg B

	// Along with UCSZ02 bit in UCSR0B, set 8 bits
	UCSR0C = (1<<UCSZ01)|(1<<UCSZ00);
	
	DDRD |= (1<< 1); // PD0 is output (Tx)
	DDRD &= ~(1<< 0); // PD1 is input (Rx)

	// Empty buffers
	rx_buffer_head = 0;
	rx_buffer_tail = 0;
}
void adc_init(void)
{
	// |= compound bitwise or --> 'set to 1'
	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC (analog power supply cleaned)
	// configures ADC
	ADCSRB |= (0 << ADTS0) | (0 << ADTS1) | (0 << ADTS2);  //Free-Running Mode

	ADCSRA |= (1 << ADEN);  // Enable ADC (turn on)
	ADCSRA |= (1 << ADSC);  // Start A/D Conversions
}

void usart_printf(const char *ptr){

	// Send NULL-terminated data from FLASH.
	// Uses polling (and it blocks).

	char c;

	while(pgm_read_byte_near(ptr)) { 
		c = pgm_read_byte_near(ptr++); //Read a byte from the program space with a 16-bit (near) address.
		usart_putc(c);
	}
}

float string2decimal(char ov[4]){
	float result = 0.00;
	result = result + ((float)ov[3]-48.0)*1.0;
	result = result + ((float)ov[2]-48.0)*.1;
	result = result + ((float)ov[0]-48.0)*.01;

	return result;
}

void usart_putc(const char c){

	// Send "c" via the USART.  Uses poling
	// (and it blocks). Wait for UDRE0 to become
	// set (=1), which indicates the UDR0 is empty
	// and can accept the next character.

	while (!(UCSR0A & (1<<UDRE0)))
	;
	UDR0 = c;
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;

	UDR0 = data;
}
void usart_prints(const char *ptr){
	
	// Send NULL-terminated data from SRAM.
	// Uses polling (and it blocks).

	while(*ptr) {
		while (!( UCSR0A & (1<<UDRE0)))
		;
		UDR0 = *(ptr++);
	}
}

int bcd2dec(int temp)
{
	return (((temp & 0xF0) >> 4) * 10) + (temp & 0x0F);
}

unsigned char usart_getc(void)
{
	// Get char from the receiver buffer.  This
	// function blocks until a character arrives.
	
	unsigned char c;
	
	// Wait for a character in the buffer.

	while (rx_buffer_tail == rx_buffer_head) //rx_buffer_head is the index of the location from which to write.
	;
	
	c = rx_buffer[rx_buffer_tail]; //rx_buffer_tail is the index of the location from which to read.
	if (rx_buffer_tail == RX_BUFFER_SIZE-1)
	rx_buffer_tail = 0;
	else
	rx_buffer_tail++; //move to next read location
	return c;
}

void readADC(float *v)
{
	ADCSRA |= (1<<ADSC);
	while(ADCSRA & (1<<ADSC)){
	}
	*v = (float)ADCW*((float)5/(float)(1023));
}


/****************************************************************************************
*    References:
*   (1) Author: Anton Kruger
*    Link: http://s-iihr64.iihr.uiowa.edu/MyWeb/Teaching/ECE3360_2016/Resources/SerialSample.c
*    Title: Sample code for serial communication
*	 Retrieved on: 4/2/2018
*
***************************************************************************************/