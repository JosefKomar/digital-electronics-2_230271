/***********************************************************************
 * 
 * Use USART unit and transmit data between ATmega328P and computer.
 * 
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * Copyright (c) 2018 Tomas Fryza
 * Dept. of Radio Electronics, Brno University of Technology, Czechia
 * This work is licensed under the terms of the MIT license.
 * 
 **********************************************************************/


/* Defines -----------------------------------------------------------*/
#ifndef F_CPU
# define F_CPU 16000000  // CPU frequency in Hz required for UART_BAUD_SELECT
#endif


/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <uart.h>           // Peter Fleury's UART library
#include <stdlib.h>         // C library. Needed for number conversions


/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and transmit UART data four times 
 *           per second.
 * Returns:  none
 **********************************************************************/
int main(void)
{
    // Initialize USART to asynchronous, 8N1, 9600
    uart_init(UART_BAUD_SELECT(9600, F_CPU));
    
    // Configure 16-bit Timer/Counter1 to transmit UART data
    // Set prescaler to 262 ms and enable overflow interrupt
    TIM1_overflow_4s();
    TIM1_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();

    // Put strings to ringbuffer for transmitting via UART
    uart_puts("Print one line... ");
    uart_puts("done\r\n");
    uart_puts("--- Interactive UART console ---\n ");
    uart_puts("1: read current Timer/counter1 value\n ");
    uart_puts("2: reset Timer/counter1\n ");
    



    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
        uint8_t value;
        uint8_t timer;
        char string[8];

        value = uart_getc();
        if (value != '\0') {     // Data available from UART
            if (value == '1') {  // Key `1` received
                timer = TCNT1;
                itoa(timer,string,10);
                uart_puts(string);
                uart_putc('\n');
                
        }
             if (value == '2') {
                TCNT1 = 0;
                uart_puts("Reseted");
                uart_putc('\n');
        }
    }
    }

    // Will never reach this
    return 0;
}

/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Transmit UART data four times per second.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
    // Transmit UART string(s)
    uint8_t value;
    char string[8];  // String for converted numbers by itoa()

    
    /* value = uart_getc();
    if (value != '\0') {  // Data available from UART
        // Display ASCII code of received character
        // WRITE YOUR CODE HERE
        uart_puts("Pressed key: ");
        uart_putc(value);
        uart_putc('\t');

        uart_puts("DEC value: ");
        itoa(value,string,10);
        uart_puts(string);
        uart_putc('\t');

        uart_puts("HEX value: ");
        itoa(value,string,16);
        uart_puts(string);
        uart_putc('\t');

        uart_puts("BIN value: ");
        itoa(value,string,2);
        uart_puts(string);
        uart_putc('\t\n');
    }
    uart_puts("\x1b[4;32m");  // 4: underline style; 32: green foreground
    uart_puts("This is all Green and Underlined\r\n");
    //uart_puts("\x1b[0m");     // 0: reset all attributes
    //uart_puts("This is Normal text again\r\n");*/

    
}