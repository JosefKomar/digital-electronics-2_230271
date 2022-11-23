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

#define OutputCLK PD2
#define OutputDT PD3
#define OutputSW PB2

/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Update stopwatch value on LCD screen when 8-bit 
 *           Timer/Counter2 overflows.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    static int8_t counter = 0; 
    static int8_t aState;
    static int8_t aLastState;
    static uint8_t RST = 1;
    char string[2];
    
  
    
    GPIO_mode_input_nopull(&DDRD,OutputCLK);
    GPIO_mode_input_nopull(&DDRD,OutputDT);
    GPIO_mode_input_pullup(&DDRB,OutputSW);

    aLastState = GPIO_read(&PIND,OutputCLK);

    

    
    // Initialize display
    lcd_init(LCD_DISP_ON_CURSOR);

    // Put string(s) on LCD screen
    //lcd_gotoxy(1, 0);
    //lcd_puts("00:00.0");

    lcd_gotoxy(1, 1);
    //lcd_puts("jarek je DOG");

    // Configuration of 8-bit Timer/Counter2 for Stopwatch update
    // Set the overflow prescaler to 16 ms and enable interrupt
    TIM2_overflow_16ms();
    TIM2_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Infinite loop
    while (1)
    {
        


        aState = GPIO_read(&PIND,OutputCLK); // Reads the "current" state of the outputA
        // If the previous and the current state of the outputA are different, that means a Pulse has occured
        if (aState != aLastState){
        // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
        if (GPIO_read(&PIND,OutputDT) != aState) {
        counter ++;
        lcd_gotoxy(1, 1);
        lcd_puts("    ");
        } else {
        counter --;
        lcd_gotoxy(1, 1);
        lcd_puts("    ");
        }
        itoa(counter, string, 10);
        lcd_gotoxy(1, 1);
        lcd_puts(string);

        

        }
        aLastState = aState;
        RST = GPIO_read(&PINB,OutputSW);
        if (RST == 0){
            counter = 0;
        }
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
ISR(TIMER2_OVF_vect)
{
    
    
    // Else do nothing and exit the ISR
}