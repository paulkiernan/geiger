/* Name: main.c
 * Author: Paul Kiernan <paulkiernan1@gmail.com>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include <stdio.h>          // Facilitate UART file type redefinition
#include "uart.h"           // UART driver (serial communication library)
FILE uart_str = FDEV_SETUP_STREAM(uart_putchar, uart_getchar, _FDEV_SETUP_RW);

#include <util/delay.h>     // Required by lcd_lib for LCD timing
#include <avr/pgmspace.h>   // Another requirement of (static) LCD printing
#include "lcd_lib.h"        // LCD driver

// Program Definitions
#define T250MILLISECONDS 250
#define T1MINUTE (1000 * 60)
#define CPM2SIEVERT 0.0057f

// Function Prototypes
void initialize(void);      // All the usual mcu stuff
void init_lcd(void);        // Initalize the LCD
void toggle_led(void);      // Blink the LED on PORTD2
void clear_array(uint8_t a[], int num_elements);
int  sum_array(uint8_t a[], int num_elements);
void print_array(uint8_t a[], int num_elements);
void print_vals_to_lcd(int cpm);

// Global program variables
int rad_counter = 0;                // Event accumulator
uint8_t click_counter[60];          // CPM circular array
volatile unsigned char time1;       // LED blink counter
volatile unsigned int  time2;       // Minute counter
volatile unsigned int time2_index;  // In
unsigned char led;                  // LED state
uint8_t lcd_buffer[17];             // LCD display buffer
float micro_sieverts_per_hour;      // uSv/hr calculation
char usv_string[10];                // String buffer for uSv calculation

// Default LCD messages
const uint8_t LCD_initialize[] PROGMEM = "LCD Initialized \0";
const uint8_t LCD_header[]     PROGMEM = "CPM  | uSv/hr   \0";


//-----------------------------------------------------------------------------+
// Interrupt Service Routines                                                  +
//-----------------------------------------------------------------------------+

// Timer 0 compare ISR
ISR (TIMER0_COMPA_vect){
    if (time1>0) time1--;
    if (time2>0) time2--;
}

// Handle trigger
ISR (INT1_vect){
    rad_counter++;                  // Accumulate total rad counter
    click_counter[time2/1000]++;    // Accumulate in a cpm circular array

    // Ping the serial port that we detected an ionization event
    fprintf(stdout, "detected ionization event!\n\r");
}


//-----------------------------------------------------------------------------+
// Main Program function                                                       +
//-----------------------------------------------------------------------------+
int main(void){

    initialize();

    // Main Program loop
    int clicks_per_minute = 0;
    while(1){

        // Time-dependent tasks
        if (time1==0){
            time1=T250MILLISECONDS;     // Reset time counter
            toggle_led();               // Blink LED

            // Clear adjacent element in circular array
            // Timer counts down from 60 so we -- for the next element
            time2_index = time2/1000;
            if (time2_index == 0) time2_index = 61;
            click_counter[time2_index-1] = 0;
        }
        if (time2==0){
            time2=T1MINUTE;       // Reset time counter
        }

        // Calculate CPM and print to LCD
        clicks_per_minute = sum_array(click_counter, 60);
        print_vals_to_lcd(clicks_per_minute);
    }
}


void print_vals_to_lcd(int cpm){

    micro_sieverts_per_hour = (float)cpm * CPM2SIEVERT;
    dtostrf(micro_sieverts_per_hour , 4, 6, usv_string);

    // Write CPM
    sprintf(lcd_buffer, "%i   ", cpm);
    LCDGotoXY(0, 1);
    LCDstring(lcd_buffer, strlen(lcd_buffer));

    // Write uSv
    sprintf(lcd_buffer, "| %s", usv_string);
    LCDGotoXY(5, 1);
    LCDstring(lcd_buffer, strlen(lcd_buffer));
}


void toggle_led(void){
    led = led ^ 1 ;
    PORTD = (led<<PORTD2) ;
}


void clear_array(uint8_t a[], int num_elements){
    for(int i=0; i<num_elements; i++){
        a[i] = 0;
    }
}


int sum_array(uint8_t a[], int num_elements){
   int sum = 0;
   for (int i=0; i<num_elements; i++){
        sum = sum + a[i];
   }
   return(sum);
}


void print_array(uint8_t a[], int num_elements){
    for(int i=0; i<num_elements; i++){
         fprintf(stdout, "%d ", a[i]);
    }
    fprintf(stdout, "\n");
}


//-----------------------------------------------------------------------------+
// Initializations                                                             +
//-----------------------------------------------------------------------------+
void initialize(void){

    // Initialize microcontroller ports and registers
    DDRD = (1<<PORTD2);    // PORT D.2 is an output

    //set up timer 0 for 1 mSec timebase
    TIMSK0= (1<<OCIE0A);    // Turn on timer 0 compare match ISR
    OCR0A = 249;            // Set the compare register to 250 time ticks
    TCCR0B= 3;              // Set prescalar to divide by 64
    TCCR0A= (1<<WGM01);     // Turn on clear-on-match

    // Init the LED state and task timer
    led=0x00;
    time1=T250MILLISECONDS;
    time2=T1MINUTE;
    clear_array(click_counter, 60);

    // Set up the INT1 External Interrupt pin
    EIMSK |= (1 << INT1);           // Turns on INT1
    EICRA = 1<<ISC11 | 1<<ISC10;    // Trigger INT1 on rising edge

    //crank up the ISRs
    sei();

    // Initialize UART comm
    uart_init();
    stdout = stdin = stderr = &uart_str;
    fprintf(stdout, "\033[2J");
    fprintf(stdout, "Starting Geiger Counter...\n\r");

    // Initialize the LCD
    init_lcd();
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
    CopyStringtoLCD(LCD_header, 0, 0);
}
