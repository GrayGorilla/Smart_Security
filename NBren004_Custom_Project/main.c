/*
 * NBren004_Custom_Project.c
 *
 * Created: 5/23/2019 1:51:25 AM
 * Author : Nathan Brennan
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "nokia5110.h"

#define HIDE 0
#define SHOW 1



#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>
#define TASKS_SIZE 2
#define TASKS_PERIOD 100
#include "bit.h"
#include "scheduler.h"
#include "timer.h"
#include "keypad.h"

char* message;
unsigned char position;


/* ----------------------- STATE MACHINES ----------------------- */

enum lockStates { Start, Reset, Locked, Unlocked, Armed, SoundAlarm };
int place_holder(){};


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

int main(void) {
    
    DDRC = 0xF0; PORTC = 0x0F;  // Keypad input
    
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




int main1(void)
{
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

    while (1) {
        _delay_ms(1000);
    }
}