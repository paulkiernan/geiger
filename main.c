/* Name: main.c
 * Author: Paul Kiernan <paulkiernan1@gmail.com>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

// This pgm just blinks D.2 for testing the protoboard.

//***********************************************************************
// Includes
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>          // Facilitate UART file type redefinition
#include "uart.h"           // UART driver (serial communication library)
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

#include <util/delay.h>     // Required by lcd_lib for LCD timing
#include <avr/pgmspace.h>   // Another requirement of (static) LCD printing
#include "lcd_lib.h"        // LCD driver

//timeout values for each task
#define t1 250

// task subroutines
void task1(void);       //blink at 2 or 8 Hz
void initialize(void);  //all the usual mcu stuff
void init_lcd(void);    //initalize the LCD

int rad_counter;
int8_t lcd_buffer[17];          // LCD display buffer
volatile unsigned char time1;   //timeout counter
unsigned char led;              //light states
const int8_t LCD_initialize[] PROGMEM = "LCD Initialized\0";
const int8_t LCD_chartbeat[] PROGMEM = "CHARTBEAT    CPM\0"; // LCD init msg
const int8_t LCD_hackweek[]  PROGMEM = "HACKWEEK!\0";  // LCD init msg

//**********************************************************
//timer 0 compare ISR
ISR (TIMER0_COMPA_vect){
    //Decrement the  time if they are not already zero
    if (time1>0)    --time1;
}


// Handle external interrupt 0
ISR (INT0_vect){
    rad_counter += 1;
}


//**********************************************************
//Entry point and task scheduler loop
int main(void){
    initialize();
    init_lcd();

    // init the UART -- uart_init() is in uart.c
    uart_init();
    stdout = stdin = stderr = &uart_str;
    fprintf(stdout, "Starting Geiger Counter...\n\r");

    rad_counter = 0;

    //main task scheduler loop
    int temp_clicks = 0;
    while(1){
        if (time1==0){
            time1=t1;
            task1();
        }
        if (temp_clicks != rad_counter){
            sprintf(lcd_buffer,"Clicks: %i", rad_counter);
            LCDGotoXY(0, 1);
            LCDstring(lcd_buffer, strlen(lcd_buffer));
            temp_clicks = rad_counter;
        }
    }
}

//**********************************************************
//Task 1
void task1(void){
    //toggle the second bit
    led = led ^ 1 ;
    PORTD = (led<<PORTD2) ;
}


//**********************************************************
//Set it all up
void initialize(void){

    DDRD = (1<<PORTD2);    // PORT D.2 is an output
    DDRD &= ~(1<<PORTD0);  // PORT D.0 is an input

    //set up timer 0 for 1 mSec timebase
    TIMSK0= (1<<OCIE0A);    //turn on timer 0 cmp match ISR
    OCR0A = 249;          //set the compare register to 250 time ticks
    //set prescalar to divide by 64
    TCCR0B= 3;
    // turn on clear-on-match
    TCCR0A= (1<<WGM01) ;

    //init the LED status
    led=0x00;

    //init the task timer
    time1=t1;

    /*GICR = 1<<INT0;                 // Enable INT0*/
    MCUCR = 1<<ISC01 | 1<<ISC00;    // Trigger INT0 on rising edge

    //crank up the ISRs
    sei();
}


// Initalize the LCD stuff
/*void init_lcd(void){*/
    /*_delay_ms(500);*/
    /*LCDinit();          //initialize the display*/
    /*LCDcursorOFF();     //turn off the LCD cursor*/
    /*LCDclr();           //clear the display*/
    /*LCDGotoXY(0,0);*/
/*}*/

void init_lcd(void){
    _delay_ms(500);
    LCDinit();   //initialize the display
    LCDcursorOFF();
    LCDclr();            //clear the display
    LCDGotoXY(0,0);
    CopyStringtoLCD(LCD_initialize, 0, 0);
    _delay_ms(1000);

    LCDclr();            //clear the display
    LCDGotoXY(0,0);
    CopyStringtoLCD(LCD_chartbeat, 0, 0);
    CopyStringtoLCD(LCD_hackweek, 0, 1);

}
