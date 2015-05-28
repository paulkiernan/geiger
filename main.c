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
void led_heartbeat(void);       //blink at 2 or 8 Hz
void initialize(void);  //all the usual mcu stuff
void init_lcd(void);    //initalize the LCD
uint16_t  ReadADC(uint8_t);    //initalize the LCD

long Ain;                       // raw A to D number
int rad_counter = 0;            // CPM
int8_t lcd_buffer[17];          // LCD display buffer
volatile unsigned char time1;   // timeout counter
unsigned char led;              // light states
const uint8_t LCD_initialize[] PROGMEM = "LCD Initialized\0";
const uint8_t LCD_chartbeat[] PROGMEM = "CHARTBEAT    CPM\0"; // LCD init msg
const uint8_t LCD_hackweek[]  PROGMEM = "HACKWEEK!\0";  // LCD init msg

//**********************************************************
//timer 0 compare ISR
ISR (TIMER0_COMPA_vect){
    //Decrement the  time if they are not already zero
    if (time1>0)    --time1;
}


// Handle external interrupt 0
ISR (INT1_vect){
    /*fprintf(stdout, "Dropped into the interrupt routine!");*/
    rad_counter++;
}

int main(void){
    initialize();
    uart_init();    // init the UART -- uart_init() is in uart.c
    stdout = stdin = stderr = &uart_str;
    fprintf(stdout, "\033[2J");
    fprintf(stdout, "##########################\n\r");
    fprintf(stdout, "Starting Geiger Counter...\n\r");
    fprintf(stdout, "##########################\n\r");

    /*init_lcd();     // init the LCD screen*/

    //main task scheduler loop
    while(1){
        if (time1==0){
            time1=t1;
            led_heartbeat();
        }

        /*sprintf(lcd_buffer, "SAin: %ld", Ain);*/
        fprintf(stdout, "FAin: %i\n\r", rad_counter);
        /*LCDGotoXY(0, 1);*/
        /*LCDstring(lcd_buffer, strlen(lcd_buffer));*/
        _delay_ms(50);
    }
}


void led_heartbeat(void){
    //toggle the second bit
    led = led ^ 1 ;
    PORTD = (led<<PORTD2) ;
}


void initialize(void){

    DDRD = (1<<PORTD2);    // PORT D.2 is an output

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
    /*GICR |= ( 1 << INT1);*/
    /*//falling edge triggers interrupt0*/
    /*MCUCR |= ( 0 << ISC00);*/
    /*MCUCR |= ( 1 << ISC01);*/
    //falling edge interrupt 1
    EIMSK |= (1 << INT1);     // Turns on INT1
    /*MCUCR = 1<<ISC11 | 1<<ISC10;    // Trigger INT0 on rising edge*/

    //crank up the ISRs
    sei();
}


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
