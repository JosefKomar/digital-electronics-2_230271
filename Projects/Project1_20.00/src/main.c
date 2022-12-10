/***********************************************************************
 * 
 * Stopwatch by Timer/Counter2 on the Liquid Crystal Display (LCD)
 *
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2017 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 * Components:
 *   16x2 character LCD with parallel interface
 *     VSS  - GND (Power supply ground)
 *     VDD  - +5V (Positive power supply)
 *     Vo   - (Contrast)
 *     RS   - PB0 (Register Select: High for Data transfer, Low for Instruction transfer)
 *     RW   - GND (Read/Write signal: High for Read mode, Low for Write mode)
 *     E    - PB1 (Read/Write Enable: High for Read, falling edge writes data to LCD)
 *     D3:0 - NC (Data bits 3..0, Not Connected)
 *     D4   - PD4 (Data bit 4)
 *     D5   - PD5 (Data bit 5)
 *     D6   - PD6 (Data bit 6)
 *     D7   - PD7 (Data bit 7)
 *     A+K  - Back-light enabled/disabled by PB2
 * 
 **********************************************************************/

// ENCODER
#define OutputCLK PD2
#define OutputDT PD3
#define OutputSW PB2

// JOYSTICK
//#define OutputVRX PC1
//#define OutputVRY PC0
#define OutputSWJ PB3

// LEDS
#define LED_1 PC6
#define LED_2 PC5
#define LED_3 PC4
#define LED_4 PC3
#define LED_5 PC2
#define LED_6 PB5
#define LED_7 PB4


/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions
#include <Servo.h>

static int8_t counter = 0; 
static uint8_t aState;
static uint8_t aLastState;
static uint8_t button = 1;
int8_t nullX, nullY;
 

/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Update stopwatch value on LCD screen when 8-bit 
 *           Timer/Counter2 overflows.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    GPIO_mode_input_nopull(&DDRD,OutputCLK);
    GPIO_mode_input_nopull(&DDRD,OutputDT);
    GPIO_mode_input_pullup(&DDRB,OutputSW);

    aLastState = GPIO_read(&PIND,OutputCLK);

    // Initialize display
    lcd_init(LCD_DISP_ON);
    lcd_gotoxy(0, 0);


    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX |= (1<<REFS0);
    ADMUX &= ~(1<<REFS1); 
    // Select input channel ADC0 (voltage divider pin)
    ADMUX &= ~((1<<MUX3) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0) );
    // Enable ADC module
    ADCSRA |= (1<<ADEN);
    // Enable conversion complete interrupt
    
    ADCSRA |= (1<<ADIE);
    // Set clock prescaler to 128
    ADCSRA |= (1<<ADPS2);
    ADCSRA |= (1<<ADPS1);
    ADCSRA |= (1<<ADPS0);



    // Configuration of 8-bit Timer/Counter2 for Stopwatch update
    // Set the overflow prescaler to 16 ms and enable interrupt
    TIM1_overflow_33ms();
    TIM1_overflow_interrupt_enable();
    TIM0_overflow_1ms();
    TIM0_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Infinite loop
    while (1)
    {

    }
    // Will never reach this
    return 0;
}



/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter2 overflow interrupt
 * Purpose:  Update the stopwatch on LCD screen every sixth overflow,
 *           ie approximately every 100 ms (6 x 16 ms = 100 ms).
 **********************************************************************/
ISR(TIMER0_OVF_vect)
{
    char string[4];
    aState = GPIO_read(&PIND,OutputCLK);
    lcd_gotoxy(0, 0);
    lcd_puts("Counter:");

    if (aState != aLastState && aState == 1){

        if (GPIO_read(&PIND,OutputDT) != aState) {
        counter ++;
        }
        
        else {
        counter --;
        }

        itoa(counter, string, 10);
        lcd_gotoxy(8, 0);
        lcd_puts("    ");
        lcd_gotoxy(8, 0);
        lcd_puts(string);
    }

aLastState = aState;

button = GPIO_read(&PINB,OutputSW);

    if (button == 0){

        counter = 0;

        itoa(counter, string, 10);
        lcd_gotoxy(8, 0);
        lcd_puts("      ");
        lcd_gotoxy(8, 0);
        lcd_puts(string);
    }
}


ISR(TIMER1_OVF_vect)
{
   // Start ADC conversion
    ADCSRA |= (1<<ADSC);

}


ISR(ADC_vect)
{
    uint16_t value;
    char string[4];  // String for converted numbers by itoa()
    //char buttonName[6];

    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    value = ADC;
    // Convert "value" to "string" and display it
    //lcd_clrscr();
    
    itoa(value, string, 10);   
    
    lcd_gotoxy(0, 1); lcd_puts(" t   ");
    lcd_gotoxy(0, 1); lcd_puts(string);
   
    
}