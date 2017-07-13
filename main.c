/*
 * TISO_TEST.c
 *
 * Created: 12.07.2017 18:08:52
 * Author : Ximerik
 */ 

#include <avr/io.h>
#include <ctype.h>
#include <avr/eeprom.h>
#include <string.h>
#include <avr/interrupt.h>

#define BUFER_SIZE 20
#define fCK 147456
#define BAUD 9600
#define MYUBRR (fCK/( BAUD * 16 ))-1

/************************************************************************/
/* FIFO bufer                                                           */
/************************************************************************/
struct ring_bufer{
	uint8_t mem[BUFER_SIZE];
	uint8_t write;
}bufer;

void rWrite(uint8_t* byte){
	uint8_t* pWrite = bufer.write; 
	bufer.mem[(*pWrite)++]=*byte;
	if (pWrite>19)
		*pWrite=0;
}

/************************************************************************/
/* TIMER 0 PWM                                                          */
/************************************************************************/
void TC0_init(void){
	PORTD |= (1<<PD6);
	DDRD  |= (1<<PD6);
	TCCR0A |=	(1<<COM0A0)|
				(1<<WGM00)|
				(1<<WGM01);

	TCCR0B |=	(1<<WGM02)|
				(1<<CS00)|
				(1<<CS01);
	OCR0A = 0x2E;
}

/************************************************************************/
/* TIMER 1 USART                                                        */
/************************************************************************/
void TC1_init(void){
	TCCR1B |= (1<<WGM12)|
			  (1<<CS12)|
			  (1<<CS10);
			  
	OCR1AH = 0x38;
	OCR1AL = 0x53;
	
	TIMSK1 |= (1<<ICIE1)|
			  (1<<OCIE0A);	
			  
}

/************************************************************************/
/*USART                                                                 */
/************************************************************************/
void USART_Init( unsigned int baud ){ // доработать
	/* Set baud rate */
	UBRR0H = (unsigned char)(baud>>8);
	UBRR0H = (unsigned char)baud;
	/* Enable receiver and transmitter */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	/* Set frame format: 8data, 2stop bit */
	UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}


/************************************************************************/
/* GLOBAL VARIABLE                                                      */
/************************************************************************/
uint8_t global1, 
		global2;


int main(void)
{	
	global1 = eeprom_read_byte(0x01);
	global2 = eeprom_read_byte(0x02);
	eeprom_write_byte(4,0xAA);
	eeprom_write_byte(5,0xEE);
	
	TC0_init();
	TC1_init();
	USART_Init(MYUBRR);
	
    /* Replace with your application code */
    while (1) 
    {
    }
}
ISR(TIMER0_COMPA_vect){
	while(!UCSR0A&(1<<UDRE0))
		UDR0 = global1;
	while(!UCSR0A&(1<<UDRE0))
		UDR0 = global2;
}
ISR(USART_RX_vect){
	while (!(UCSR0A & (1<<RXC0)))
		OCR0A = UDR0;
		rWrite(OCR0A);
}