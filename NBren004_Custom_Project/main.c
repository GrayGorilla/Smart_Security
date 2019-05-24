/*
 * NBren004_Custom_Project.c
 *
 * Created: 5/23/2019 1:51:25 AM
 * Author : Nathan Brennan
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include "nokia5110.h"


int main(void)
{
    nokia_lcd_init();
    nokia_lcd_clear();
    nokia_lcd_write_string("IT'S WORKING!",1);
    nokia_lcd_set_cursor(0, 15);
    nokia_lcd_write_string("Nice!", 3);
    nokia_lcd_render();

    while (1) {
        _delay_ms(1000);
    }
}