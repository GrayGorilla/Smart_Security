/*
 * NBren004_Custom_Project.c
 *
 * Created: 5/23/2019 1:51:25 AM
 * Author : Nathan Brennan
 */ 

#define F_CPU 8000000UL		/* Define CPU Frequency e.g. here its 8MHz */
#define HIDE 0
#define SHOW 1
#define TASKS_SIZE 7
#define TASKS_PERIOD 10

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
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
#include "USART.h"

void test_1();
void test_2();
void test_3();
void test_4();
void itoa(int, char*, int);
void resetEEPROM();


char* message;
unsigned char position;

unsigned char finishedPass, b_arm, b_reset, b_lock, p_sensor, u_bluetooth, l_reset, l_unlocked, l_locked, l_armed, l_soundAlarm;
unsigned char password[4];
Queue* keyInput;


/* ----------------------- STATE MACHINES ----------------------- */

enum ButtonStates { B_Start, B_NoButton, B_Arm, B_Reset, B_Lock/*, B_Read*/ };
int button_tick(int state) {
    unsigned char arm, reset, lock;
    arm = ~PINB & 0x01;
    reset = ~PINB & 0x02;
    lock = ~PINB & 0x04;
    // Transitions
    switch (state) {
        case B_Start:
            b_arm = 0;
            b_reset = 0;
            b_lock = 0;
            state = B_NoButton;
            break;
        case B_NoButton:
        case B_Arm:
        case B_Reset:
        case B_Lock:
            if (arm) {
                state = B_Arm;
            } else if (reset) {
                state = B_Reset;
            } else if (lock) {
                state = B_Lock;
            } else {
                state = B_NoButton;
            }
            break;
        default:
            state = B_Start;
            break;
    }
    // Actions
    switch (state) {
        case B_NoButton:
            b_arm = 0;
            b_reset = 0;
            b_lock = 0;
            break;
        case B_Arm:
            b_arm = 1;
            b_reset = 0;
            b_lock = 0;
            break;
        case B_Reset:
            b_arm = 0;
            b_reset = 1;
            b_lock = 0;
            break;
        case B_Lock:
            b_arm = 0;
            b_reset = 0;
            b_lock = 1;
            break;
        default:
            break;
    }
    return state;
}

enum PIRsensorState { P_Start };
int pirSensor_tick(int state) {
    unsigned char sensor;
    sensor = PINC & 0x01;
    if (sensor) {
        p_sensor = 1;
    } else {
        p_sensor = 0;
    }
    return state;
}

enum BlueToothStates { U_Start, U_Bluetooth };
int blueTooth_tick(int state) {
    // Transitions & Actions
    switch (state) {
        case U_Start:
            u_bluetooth = 0;
            state = U_Bluetooth;
            break;
        case U_Bluetooth:
            if (USART_HasReceived()) {
                switch (USART_Receive()) {
                    case '0':
                        u_bluetooth = 0;
                        USART_Transmit('o');
                        break;
                    case '1':
                        u_bluetooth = 1;
                        USART_Transmit('i');
                        break;
                    default:
                        USART_Transmit('N');
                        break;
                }
            }
            state = U_Bluetooth;
            break;
        default:
            state = U_Start;
            break;         
    }
    return state;
}

enum LockStates { L_Start, L_Set, L_Reset, L_Unlocked, L_Locked, L_Armed, L_SoundAlarm };
int lock_tick(int state) { 
    unsigned char savedState;
    // Transitions
    switch (state) {
        case L_Start:
            l_reset = 1;
            l_unlocked = 0;
            l_locked = 0;
            l_armed = 0;
            l_soundAlarm = 0;
            finishedPass = 0;
            savedState = eeprom_read_byte((uint8_t*) 0);
            if (savedState) {
                state = savedState;
            } else {
                state = L_Set;
            }
            break;
        case L_Set:
            if (finishedPass) {
                state = L_Unlocked;
                finishedPass = 0;
            } else {
                /* PASSWORD LOGIC */
            }                
            break;
        case L_Reset:
        case L_Unlocked:
        case L_Locked:
        case L_Armed:
        case L_SoundAlarm:
            if ((p_sensor && l_armed) || l_soundAlarm) {
                if (!u_bluetooth) {
                    state = L_SoundAlarm;
                } else {
                    state = L_Locked;
                }             
            } else if (b_reset) {       // Temporary; omitting L_Reset state
            state = L_Unlocked;
            } else if (b_lock) {
                state = L_Locked;
            } else if (b_arm) {
                if (!u_bluetooth) {
                    state = L_Armed;
                } else {
                    nokia_lcd_clear();
                    nokia_lcd_write_string("To arm, please disable override", 1);
                    nokia_lcd_render();
                    state = L_Locked;
                }
            }
            break;
        default:
            state = L_Start;
            break;
    }        
    // Actions
    switch (state) {
        case L_Unlocked:
            l_reset = 0;
            l_unlocked = 1;
            l_locked = 0;
            l_armed = 0;
            l_soundAlarm = 0;
            PORTA = 0x00;
            break;
        case L_Locked:
            l_reset = 0;
            l_unlocked = 0;
            l_locked = 1;
            l_armed = 0;
            l_soundAlarm = 0;
            PORTA = 0x00;
            break;
        case L_Armed:
            l_reset = 0;
            l_unlocked = 0;
            l_locked = 0;
            l_armed = 1;
            l_soundAlarm = 0;
            PORTA = 0x01;
            break;
        case L_SoundAlarm:
            l_reset = 0;
            l_unlocked = 0;
            l_locked = 0;
            l_armed = 0;
            l_soundAlarm = 1;
            PORTA = 0x00;
            break;
        default:
            break;
    }
    

    // Transitions
    /*
    switch (state) {
        case L_Start:
            savedState = eeprom_read_byte((uint8_t*) 0);
            if (savedState) {
                state = savedState;
            } else {
                state = L_Reset;
            }
            break;
            QueueClear(keyInput);
            keyEnable = 1;
            break;
        case L_Reset:
            if (b_lock && QueueFull(*keyInput)) {
                state = L_Locked;
                password = keyInput->buf;
                QueueClear(keyInput);
            }
            else if (b_arm) {
                state = L_Armed;
                password = keyInput.buf;
                QueueClear(keyInput);
                keyEnable = 0;
            }
            else {
                state = L_Reset;
            }
            break;
        case L_Locked:
            if (keyInput->buf == password) {
                state = L_Unlocked;
                keyEnable = 0;
            }
            else {
                state = L_Locked;
            }
            break;
        case L_Armed:
            break;
        case L_SoundAlarm:
            break;
        case L_Unlocked:
            if (b_reset) {
                state = L_Reset;
                QueueClear(keyInput);
                keyEnable = 1;
            }
            else if (b_lock) {
                state = L_Locked;
                QueueClear(keyInput);
                keyEnable = 1;
            }
            else if (b_arm) {
                state = L_Armed;
                QueueClear(keyInput);
                keyEnable = 1;
            }
    } */
    eeprom_write_byte((uint8_t*) 0, state);
    return state; 
};

enum ServoStates { S_Start, S_Unlock, S_Lock };
int servo_tick(int state) {
    unsigned char savedState;
    // Transitions
    switch (state) {
        case S_Start:
            savedState = eeprom_read_byte((uint8_t*) 1);
            if (savedState) {
                state = savedState;
            } else {
                state = S_Unlock;
            }
            break;
        case S_Unlock:
            if (l_locked || l_armed) {
                state = S_Lock;
            } else {
                state = S_Unlock;
            }
            break;
        case S_Lock:
            if (l_unlocked) {
                state = S_Unlock;
            } else {
                state = S_Lock;
            }
            break;
        default:
            state = S_Start;
            break;
    }
    // Actions
    switch (state) {
        case S_Unlock:
            OCR1A = 75;	    // Set servo shaft at 0° position
            break;
        case S_Lock:
            OCR1A = 190;    // Set servo shaft at 90° position
            break;
        default:
            break;
    }
    eeprom_write_byte((uint8_t*) 1, state);
    return state;
};

enum AlarmStates { A_Start, A_Disable, A_High, A_Low };
int alarm_tick(int state) {
    unsigned char savedState;
    // Transitions
    switch (state) {
        case A_Start:
            savedState = eeprom_read_byte((uint8_t*) 2);
            if (savedState) {
                state = savedState;
            } else {
                state = A_Disable;
            }
            break;
        case A_Disable:
            if (l_soundAlarm) {
                state = A_High;
            } else {
                state = A_Disable;
            }
            break;
        case A_High:
            if (l_soundAlarm) {
                state = A_Low;
            } else {
                state = A_Disable;
            }
            break;
        case A_Low:
            if (l_soundAlarm) {
                state = A_High;
                } else {
                state = A_Disable;
            }
            break;
        default:
            state = A_Start;
            break;
    }
    // Actions
    switch (state) {
        case A_Disable:
        case A_Low:
            set_PWM(0);             // No sound
            PORTD = PIND & 0x7F;    // LED Off
            break;
        case A_High:
            set_PWM(174.61);        // Low C-note
            PORTD = PIND | 0x80;    // LED On
            break;
        default:
            break;
    }
    eeprom_write_byte((uint8_t*) 2, state);
    return state;
};

enum NokiaStates { N_Start, N_Reset, N_Unlocked, N_Locked, N_Armed, N_SoundAlarm };
int nokia_tick(int state) {
    unsigned char savedState, stateHasChanged;
    // Transitions
    switch (state) {
        case N_Start:
            savedState = eeprom_read_byte((uint8_t*) 3);
            stateHasChanged = 1;
            if (savedState) {
                state = savedState;
            } else {
                state = S_Unlock;
            }
            break;
        case N_Reset:
            if (l_unlocked) {
                state = N_Unlocked;
                stateHasChanged = 1;
            } else if (l_locked) {
                state = N_Locked;
                stateHasChanged = 1;
            } else if (l_armed) {
                state = N_Armed;
                stateHasChanged = 1;
            } else if (l_soundAlarm) {
                state = N_SoundAlarm;
                stateHasChanged = 1;
            } else {
                state = N_Reset;
                stateHasChanged = 0;
            }
            break;
        case N_Unlocked:
            if (l_locked) {
                state = N_Locked;
                stateHasChanged = 1;
            } else if (l_armed) {
                state = N_Armed;
                stateHasChanged = 1;
            } else if (l_soundAlarm) {
                state = N_SoundAlarm;
                stateHasChanged = 1;
            } else if (l_reset) {
                state = N_Reset;
                stateHasChanged = 1;
            } else {
                state = N_Unlocked;
                stateHasChanged = 0;
            }                
            break;
        case N_Locked:
            if (l_unlocked) {
                state = N_Unlocked;
                stateHasChanged = 1;
            } else if (l_armed) {
                state = N_Armed;
                stateHasChanged = 1;
            } else if (l_soundAlarm) {
                state = N_SoundAlarm;
                stateHasChanged = 1;
            } else if (l_reset) {
                state = N_Reset;
                stateHasChanged = 1;
            } else {
                state = N_Locked;
                stateHasChanged = 0;
            }
            break;
        case N_Armed:
            if (l_unlocked) {
                state = N_Unlocked;
                stateHasChanged = 1;
            } else if (l_locked) {
                state = N_Locked;
                stateHasChanged = 1;
            } else if (l_soundAlarm) {
                state = N_SoundAlarm;
                stateHasChanged = 1;
            } else if (l_reset) {
                state = N_Reset;
                stateHasChanged = 1;
            } else {
                state = N_Armed;
                stateHasChanged = 0;
            }
            break;
        case N_SoundAlarm:
            if (l_unlocked) {
                state = N_Unlocked;
                stateHasChanged = 1;
            } else if (l_locked) {
                state = N_Locked;
                stateHasChanged = 1;
            } else if (l_armed) {
                state = N_Armed;
                stateHasChanged = 1;
            } else if (l_reset) {
                state = N_Reset;
                stateHasChanged = 1;
            } else {
                state = N_SoundAlarm;
                stateHasChanged = 0;
            }
            break;
        default:
            state = N_Start;
            break;
    }
    // Actions
    switch (state) {
        case N_Reset:
            if (stateHasChanged) {
                nokia_lcd_clear();
                nokia_lcd_write_string("New Passcode: ", 1);
                nokia_lcd_render();
            }
            /* PUT STUFF HERE */
            break;
        case N_Unlocked:
            if (stateHasChanged) {
                nokia_lcd_clear();
                nokia_lcd_write_string("Unlocked", 1);
                nokia_lcd_render();
            }
            break;
        case N_Locked:
            if (stateHasChanged) {
                nokia_lcd_clear();
                nokia_lcd_write_string("Locked", 2);
                nokia_lcd_render();
            }
            /* PUT STUFF HERE */
            break;
        case N_Armed:
            if (stateHasChanged) {
                nokia_lcd_clear();
                nokia_lcd_write_string("Armed", 2);
                nokia_lcd_render();
            }
            /* PUT STUFF HERE */
            break;
        case N_SoundAlarm:
            if (stateHasChanged) {
                nokia_lcd_clear();
                nokia_lcd_write_string(" !!!", 3);
                nokia_lcd_render();
            }
            /* PUT STUFF HERE */
            break;
        default:
            break;
    }
    eeprom_write_byte((uint8_t*) 3, state);
    return state;
}


/* ----------------------- MAIN ----------------------- */

int main() {
    
    resetEEPROM();
    
    //test_3();
    
    /* Initialize Ports */
    DDRA = 0x01;    PORTA = 0x00;       // Testing LED (temp)
    //DDRA = 0xF0;    PORTA = 0x0F;       /* Keypad */
    DDRB = 0x40;    PORTB = 0x07;       /* PWM & Buttons */
    DDRC = 0x3E;	PORTC = 0x01;       /* Nokia LCD Screen & PIR sensor */
    DDRD = 0x80;    PORTD = 0x00;       /* LED & Servo */
    
    /* Servo Motor Initialization */
    DDRD |= (1<<PD5);	/* Make OC1A pin as output */
    TCNT1 = 0;		    /* Set timer1 count zero */
    ICR1 = 2499;		/* Set TOP count for timer1 in ICR1 register */
    /* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
    TCCR1A = (1<<WGM11)|(1<<COM1A1);
    TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);


    /* Initialize State Machines */
    unsigned char i = 0;
    tasks[i].state = B_Start;
    tasks[i].period = 10;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &button_tick;
    i++;
    tasks[i].state = P_Start;
    tasks[i].period = 200;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &pirSensor_tick;
    i++;
    tasks[i].state = U_Start;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &blueTooth_tick;
    i++;
    tasks[i].state = L_Start;
    tasks[i].period = 100;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &lock_tick;
    i++;
    tasks[i].state = S_Start;
    tasks[i].period = 150;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &servo_tick;
    i++;
    tasks[i].state = A_Start;
    tasks[i].period = 130;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &alarm_tick;
    i++;
    tasks[i].state = N_Start;
    tasks[i].period = 150;
    tasks[i].elapsedTime = tasks[i].period;
    tasks[i].TickFct = &nokia_tick;
    i++;
    
    /* Initialize Utilities */
    PWM_on();
    USART_Init(BAUD_RATE);
    QueueInit(keyInput);
    nokia_lcd_init();
    nokia_lcd_clear();
    TimerSet(TASKS_PERIOD);
    TimerOn();    
    
    while (1) {}
}

/* ------------------ AUX Functions ------------------- */

void resetEEPROM() {
    for (int i = 0; i < 8; i++) {
        eeprom_write_byte((uint8_t*) i, 0);
    }
}    


/* ----------------------- TEST ----------------------- */

void test_1(void)
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

void test_2(void)
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

#define LED_OUTPUT		PORTD
#define PIR_Input		PINC


void test_3(void)
{
    DDRB = 0x40;    PORTB = 0x00;       /* PWM */
    DDRC = 0x3E;	PIR_Input = 0x01;   /* Set the PIR port as input port */
    DDRD = 0x80;	LED_OUTPUT = 0x00;  /* Set the LED port as output port */
    DDRD = 0x80;    PORTD = 0x00;
    
    DDRD |= (1<<PD5);	/* Make OC1A pin as output */
    TCNT1 = 0;		    /* Set timer1 count zero */
    ICR1 = 2499;		/* Set TOP count for timer1 in ICR1 register */

    /* Set Fast PWM, TOP in ICR1, Clear OC1A on compare match, clk/64 */
    TCCR1A = (1<<WGM11)|(1<<COM1A1);
    TCCR1B = (1<<WGM12)|(1<<WGM13)|(1<<CS10)|(1<<CS11);
        
        
    PWM_on();
    nokia_lcd_init();
    USART_Init(BAUD_RATE);

    
    unsigned char count = 0, bluetoothInput = 0;
    char s_count[3];
    if (!eeprom_read_byte((uint8_t*) 0)) eeprom_write_byte((uint8_t*) 0, count);
    
    nokia_lcd_clear();
    nokia_lcd_write_string("Wait..", 2);
    nokia_lcd_render();
    _delay_ms(4000);

    while(1)
    {
        count = eeprom_read_byte((uint8_t*) 0);
        itoa(count, s_count, 10);
        
        if (USART_HasReceived()) {
            switch (USART_Receive()) {
                case '0':
                    bluetoothInput = 0;
                    USART_Transmit('o');
                    break;
                case '1':
                    bluetoothInput = 1;
                    USART_Transmit('i');
                    break;
                default:
                    USART_Transmit('N');
                    break;
            }                    
        }
        if ((PIR_Input & 0x01) && !bluetoothInput) {
            nokia_lcd_clear();
            nokia_lcd_write_string("Motion!", 2);
            nokia_lcd_set_cursor(30, 20);
            nokia_lcd_write_string(s_count, 2);
            nokia_lcd_set_cursor(20, 40);
            if (bluetoothInput) {
                nokia_lcd_write_string("Disabled", 1);
                } else {
                nokia_lcd_write_string("Armed", 1);
            }
            nokia_lcd_render();
            nokia_lcd_set_cursor(0, 0);
            OCR1A = 175;	/* Set servo shaft at 0° position */
                // Sound Alarm
                LED_OUTPUT = 0x80;
                set_PWM(174.61);
                _delay_ms(500);
                LED_OUTPUT = 0x00;
                set_PWM(0);
                _delay_ms(500);
            count += 1;
            eeprom_write_byte((uint8_t*) 0, count);
        }            
        else { 
            LED_OUTPUT = 0x00;
            nokia_lcd_clear();
            nokia_lcd_write_string("Clear", 2);
            nokia_lcd_set_cursor(20, 40);
            if (bluetoothInput) {
                nokia_lcd_write_string("Disabled", 1);
                } else {
                nokia_lcd_write_string("Armed", 1);
            }
            nokia_lcd_render();
            OCR1A = 65;	/* Set servo shaft at -90° position */
            set_PWM(0);
        }            
        nokia_lcd_render();
        nokia_lcd_set_cursor(0, 0);
    }
}


void test_4() {
    
    DDRD = 0x80; PORTD = 0x00;
    USART_Init(BAUD_RATE);
    
    while (1) {
        if (USART_HasReceived()) {
            switch (USART_Receive()) {
                case '0':
                    PORTD = 0x00;
                    USART_Transmit('o');
                    break;
                case '1':
                    PORTD = 0x80;
                    USART_Transmit('i');
                    break;
                default:
                    USART_Transmit('N');
                    break;
            }
        }
    }
}