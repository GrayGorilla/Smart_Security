/*
 * NBren004_Custom_Project.c
 *
 * Created: 5/23/2019 1:51:25 AM
 * Author : Nathan Brennan
 */ 

#define F_CPU 8000000UL		/* Define CPU Frequency e.g. here its 8MHz */
#define HIDE 0
#define SHOW 1
#define TASKS_SIZE 2
#define TASKS_PERIOD 100



#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "nokia5110.h"
#include "bit.h"
#include "scheduler.h"
#include "timer.h"
#include "keypad.h"
#include "pwm.h"
#include "queue.h"



char* message;
unsigned char position;

unsigned char b_lock, b_reset, b_arm, sensor, keyEnable;
unsigned char* password;
Queue* keyInput;


/* ----------------------- STATE MACHINES ----------------------- */

enum lockStates { Start, Reset, Locked, Unlocked, Armed, SoundAlarm };
int place_holder(int state) { 
    // Transitions
    switch (state) {
        case Start:
            state = Reset;
            QueueClear(keyInput);
            keyEnable = 1;
            break;
        case Reset:
            if (b_lock && QueueFull(*keyInput)) {
                state = Locked;
                password = keyInput->buf;
                QueueClear(keyInput);
            }
            /*
            else if (b_arm) {
                state = Armed;
                password = keyInput.buf;
                QueueClear(keyInput);
                keyEnable = 0;
            }*/
            else {
                state = Reset;
            }
            break;
        case Locked:
            if (keyInput->buf == password) {
                state = Unlocked;
                keyEnable = 0;
            }
            else {
                state = Locked;
            }
            break;
        case Armed:
            break;
        case SoundAlarm:
            break;
        case Unlocked:
            if (b_reset) {
                state = Reset;
                QueueClear(keyInput);
                keyEnable = 1;
            }
            else if (b_lock) {
                state = Locked;
                QueueClear(keyInput);
                keyEnable = 1;
            }
            else if (b_arm) {
                state = Armed;
                QueueClear(keyInput);
                keyEnable = 1;
            }
    }
    return state; 
};


enum keyPadStates{KP_Start, Button};
int keyPad_tick(int state) {
    static char input;
    unsigned char mutate;
    // Transitions
    switch (state) {
        case KP_Start :
            state = Button;
            message = "Type over me!                   ";
            position = 0;
            mutate = 0;
            break;
        case Button :
            state = Button;
            break;
        default:
            break;
    }
    // Actions
    switch (state) {
        case Button :
            input = GetKeypadKey();
            if (input) mutate = 1;
            else mutate = 0;
        
            if (mutate) {
                message[position] = input;
                if (position >= 31) position = 0;
                else position++;
            }
            break;
        default:
            break;
    }
    return state;
}

enum LCD_States { LCD_Start, Display };
int LCD_tick(int state) {
    // Transitions
    switch (state) {
        case LCD_Start:
            state = Display;
            break;
        case Display:
            state = Display;
            break;
        default:
            state = LCD_Start;
            break;
    }
    // Actions
    switch (state) {
        case Display:
        
        /*
            nokia_lcd_write_string("IT'S WORKING!", 1);
            nokia_lcd_set_cursor(0, 15);
        */
            nokia_lcd_clear();
            nokia_lcd_write_string(message, 1);
            nokia_lcd_set_cursor(0, position + 1);
            nokia_lcd_render();

            //LCD_DisplayString(1, message);
            //LCD_Cursor(position + 1);

            break;
        default:
            break;
    }
    return state;
}


/* ----------------------- MAIN ----------------------- */

int dmain(void) {
    
    DDRA = 0x00; PORTA = 0x04;  // PWM
    DDRC = 0xF0; PORTC = 0x0F;  // Keypad input
    
    QueueInit(keyInput);
    
    unsigned char i = 0;
    tasks[i].state = KP_Start;
    tasks[i].period = TASKS_PERIOD;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &keyPad_tick;
    i++;
    tasks[i].state = LCD_Start;
    tasks[i].period = TASKS_PERIOD;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &LCD_tick;
    i++;
    
    TimerSet(TASKS_PERIOD);
    TimerOn();
    // Initializes the LCD display
//    LCD_init();

    nokia_lcd_init();
    nokia_lcd_clear();
    
    
    while (1) {}
}




/* ----------------------- TEST ----------------------- */

int maindd(void)
{
    DDRB = 0x40; PORTB = 0x00;  // PWM

    nokia_lcd_init();
    nokia_lcd_clear();
    nokia_lcd_write_string("IT'S WORKING!", 1);
    nokia_lcd_set_cursor(0, 15);
    nokia_lcd_write_string("Nice!", 3);
    /*
    nokia_lcd_set_pixel(5, 0, SHOW);
    nokia_lcd_set_pixel(5, 1, SHOW);
    nokia_lcd_set_pixel(5, 2, SHOW);
    nokia_lcd_set_pixel(5, 3, SHOW);
    nokia_lcd_set_pixel(5, 0, HIDE);
    */
    nokia_lcd_render();
    
    PWM_on();
    
    nokia_lcd_set_cursor(0, 15);
    nokia_lcd_write_string("Zero", 3);
    nokia_lcd_render();
    
    /*
    200: 0 degrees
    325: 90 degrees
    800: 180 degrees
    */
    
    short value = 200;

    for(short i = 0; i < 13; i++) {
        
        set_PWM(value);  
        char s[4];
        itoa(value, s, 10);
            
        nokia_lcd_clear();
        nokia_lcd_set_cursor(0, 15);
        nokia_lcd_write_string(s, 3);
        nokia_lcd_render();
        
        if (value <= 200) value = 325;
        else value = 200;
            
        _delay_ms(20000);       
    }
    
    nokia_lcd_clear();
    nokia_lcd_set_cursor(0, 15);
    nokia_lcd_write_string("Done.", 3);
    nokia_lcd_render();

        
    while (1) {
        _delay_ms(1000);
    }
}

int mainz(void)
{
    DDRD |= (1<<PD5);	/* Make OC1A pin as output */
    TCNT1 = 0;		/* Set timer1 count zero */
    ICR1 = 2499;		/* Set TOP count for timer1 in ICR1 register */

    /* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
    TCCR1A = (1<<WGM11)|(1<<COM1A1);
    TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
    
    
    DDRB = 0x40; PORTB = 0x00;  // PWM
    PWM_on();
    nokia_lcd_init();
    
    
    while(1)
    {
        OCR1A = 65;	/* Set servo shaft at -90° position */
        _delay_ms(1500);
        OCR1A = 175;	/* Set servo shaft at 0° position */
        _delay_ms(1500);
        OCR1A = 300;	/* Set servo at +90° position */
        _delay_ms(1000);
            

        
        set_PWM(174.61);
        nokia_lcd_clear();
        nokia_lcd_write_string("On", 3);
        nokia_lcd_render();
        _delay_ms(500);
        set_PWM(0);
        nokia_lcd_clear();
        nokia_lcd_write_string("Off", 3);
        nokia_lcd_render();
        _delay_ms(500);        
        set_PWM(174.61);
        nokia_lcd_clear();
        nokia_lcd_write_string("On", 3);
        nokia_lcd_render();
        _delay_ms(500);
        set_PWM(0);
        nokia_lcd_clear();
        nokia_lcd_write_string("Off", 3);
        nokia_lcd_render();
        _delay_ms(500);
        set_PWM(174.61);
        nokia_lcd_clear();
        nokia_lcd_write_string("On", 3);
        nokia_lcd_render();
        _delay_ms(500);
        set_PWM(0);
        nokia_lcd_clear();
        nokia_lcd_write_string("Off", 3);
        nokia_lcd_render();
    }
}

#define LED_OUTPUT		PORTB
#define PIR_Input		PINC


int main(void)
{
    DDRC = 0x00;	PIR_Input = 0xFF;   /* Set the PIR port as input port */
    DDRB = 0x41;	LED_OUTPUT = 0x00;  /* Set the LED port as output port */   // and PWM
    
    DDRD |= (1<<PD5);	/* Make OC1A pin as output */
    TCNT1 = 0;		/* Set timer1 count zero */
    ICR1 = 2499;		/* Set TOP count for timer1 in ICR1 register */

    /* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
    TCCR1A = (1<<WGM11)|(1<<COM1A1);
    TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
        
        
    PWM_on();

    
    nokia_lcd_init();


    while(1)
    {
        //LED_OUTPUT = ~PIR_Input & 0x01;
        nokia_lcd_clear();

        if (PIR_Input & 0x01) {
            LED_OUTPUT = 0x01;
            nokia_lcd_clear();
            nokia_lcd_write_string("Motion!", 2);
            nokia_lcd_render();
            OCR1A = 175;	/* Set servo shaft at 0° position */
            while (PIR_Input & 0x01) {
                set_PWM(174.61);
                _delay_ms(500);
                set_PWM(0);
                _delay_ms(500);
            }            
        }            
        else { 
            LED_OUTPUT = 0x00;
            nokia_lcd_write_string("Clear", 2);
            nokia_lcd_render();
            OCR1A = 65;	/* Set servo shaft at -90° position */
            set_PWM(0);
        }            
    }
}
