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
#define fCK 14745600UL
#define BAUD 9600UL
#define MYUBRR 95

/************************************************************************/
/* FIFO bufer                                                           */
/************************************************************************/
struct ring_bufer{
    char memory[SIZE];
    int write_count,
        read_count;
}bufer;

int is_full(struct ring_bufer *this_bufer){
    if (((this_bufer->write_count) - (this_bufer->read_count)) != 0)
        return 0;
    else
        return 1;
}
int is_empty(struct ring_bufer *this_bufer){
    if ((((*this_bufer->write_count) - (*this_bufer->read_count)) == 0)
        return 1;
    else
        return 0;
}
void write(char data, struct ring_bufer *this_bufer){
    while(isfull(this_bufer) != 1){
        this_bufer -> memory[(*this_bufer -> write_count)++] = data;
        if (*this_bufer -> write_count > SIZE-1)
            *this_bufer -> write_count = 0;
    }
}
void* read(char* data, struct ring_bufer *this_bufer){
    while(isempty(this_bufer) != 1){
        *data++ = this_bufer -> memory[(*this_bufer -> read_count)++];
        if (*this_bufer -> read_count > SIZE-1)
            *this_bufer -> read_count = 0;
    }
}
void clear_bufer(struct ring_bufer *this_bufer){
    *this_bufer -> write_count = 0;
    *this_bufer -> read_count = 0;
}
void rRead(uint8_t* data){
	uint8_t* pRead = bufer.read; 
	*data++ = bufer.mem[(*pRead)++];
	if (*pRead>19)
		*pRead=0;
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
	TIMSK1 |= (1<<OCIE1A);	
			  
}

/************************************************************************/
/*USART                                                                 */
/************************************************************************/
void USART_Init(void){
	UBRR0L = MYUBRR;
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	UCSR0C |= (1<<UCSZ00)|()1<<UCSZ01);
	PORTC |= (1<<PC2);
}
/************************************************************************/
/*INT1 and ADC                                                          */
/************************************************************************/
void interupt_n_adc_init(void){
	PORTD |= (1<<PD3);
	EIMSK |=(1<<INT1);
	EICRA |= (1<<ISC10);
	ADMUX |=(1<<REFS0)|
		(1<<ADLAR)|
		(1<<MUX0);
	ADCSRA |=(1<<ADIE)|
		(1<<ADPS1)|
		 (1<<ADPS0);
}

/************************************************************************/
/* GLOBAL VARIABLE                                                      */
/************************************************************************/
uint8_t global1, 
	global2,
	adc_var;


int main(void)
{	
	global1 = eeprom_read_byte(0x01);
	global2 = eeprom_read_byte(0x02);
	eeprom_write_byte(4,0xAA);
	eeprom_write_byte(5,0xEE);
	
	interupt_n_adc_init();
	TC0_init();
	TC1_init();
	USART_Init();
	
    /* Replace with your application code */
    while (1) 
    {
	if (PC2==0)
		UCSR0C |=(1<<U2X0);
	else
		UCSR0C &=~(1<<U2X0);
    }
}
ISR(TIMER0_COMPA_vect){
	while(!(UCSR0A)&(1<<UDRE0))
		UDR0 = global1;
	while(!(UCSR0A)&(1<<UDRE0))
		UDR0 = global2;
}
ISR(USART_RX_vect){
	while (!(UCSR0A) & (1<<RXC0))
		OCR0A = UDR0;
		rWrite(OCR0A, &bufer);
}
ISR(INT1_vect){
	ADCSRA|=(1<<ADEN)|
		(1<<ADSC);
}
ISR(ADC_vect){
	adc_var = ADCH;
	ADCSRA&=~(1<<ADEN)&
		~(1<<ADSC);
}
