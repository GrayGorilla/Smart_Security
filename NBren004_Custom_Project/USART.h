#ifndef USART_atmega1284_H
#define USART_atmega1284_H

#define F_CPU 8000000UL // Assume uC operates at 8MHz
#define BAUD_RATE 9600
#define BAUD_PRESCALE (((F_CPU / (BAUD_RATE * 16UL))) - 1)
//#include <util/setbaud.h>

void USART_Init( unsigned int baud )
{
    // Set baud rate
    UBRR0H = (unsigned char)(BAUD_PRESCALE>>8);
    UBRR0L = (unsigned char)BAUD_PRESCALE;
    // Enable receiver and transmitter
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    //UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(3<<UCSZ20);
    // Set frame format: 8data, 2stop bit
    UCSR0C = (1<<USBS0)|(3<<UCSZ00);
}

unsigned char USART_HasReceived( void ){
    return (UCSR0A & (1<<RXC0)) ? 1 : 0;
}


unsigned char USART_Receive( void )
{
    // Wait for data to be received
    while ( !(UCSR0A & (1<<RXC0)));
    // Get and return received data from buffer
    return UDR0;
}

void USART_Transmit( unsigned char data )
{
    // Wait for empty transmit buffer
    while ( !( UCSR0A & (1<<UDRE0)));
    // Put data into buffer, sends the data
    UDR0 = data;
}


#endif