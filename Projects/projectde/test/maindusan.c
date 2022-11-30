/***********************************************************************
 * 
 * 
 * ATmega328P (Arduino Uno), 16 MHz, PlatformIO
 *
 * 
 **********************************************************************/


/* Includes ----------------------------------------------------------*/
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <lcd.h>            // Peter Fleury's LCD library
#include <stdlib.h>         // C library. Needed for number conversions
#include <gpio.h>           // Use our gpio library
/*Define -------------------------------------------------------------*/
#define Js_button PC2      //PD2 is AVR pin where joystick button is connected

#define enc_SW_2  PD3      //second encoder SW to pin PD3
#define enc_DT_2  PD2
#define enc_clk_2 PD1
/* Function definitions ----------------------------------------------*/
/**********************************************************************
 * Function: Main function where the program execution begins
 * Purpose:  Use Timer/Counter1 and start ADC conversion every 100 ms.
 *           When AD conversion ends, send converted value to LCD screen.
 * Returns:  none
 **********************************************************************/

/*Global Para.--------------------------------------------------------*/
static uint8_t LastState,current_state;
static int8_t counter = 0;



int main(void)
{
    // Initialize display
    lcd_init(LCD_DISP_ON);
    lcd_gotoxy(1, 0); lcd_puts("x:");
    lcd_gotoxy(1, 1); lcd_puts("y:");
   
   
    
    GPIO_mode_input_pullup(&DDRB,Js_button);    //declaration our Js_button port as input
    


    
    GPIO_mode_input_pullup(&DDRD,enc_SW_2);
    
    GPIO_mode_input_nopull(&DDRD,enc_DT_2);
    GPIO_mode_input_nopull(&DDRD,enc_clk_2);

    LastState = GPIO_read(&PIND,enc_clk_2);


    // Configure Analog-to-Digital Convertion unit
    // Select ADC voltage reference to "AVcc with external capacitor at AREF pin"
    ADMUX = ADMUX | (1<<REFS0);
    // Select input channel ADC0 (voltage divider pin)
    ADMUX = ADMUX & ~(1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0);
    // Enable ADC module
    ADCSRA = ADCSRA | (1<<ADEN);
    // Enable conversion complete interrupt
    ADCSRA = ADCSRA | (1<<ADIE);
    // Set clock prescaler to 128
    ADCSRA = ADCSRA | (1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0);

    // Configure 16-bit Timer/Counter1 to start ADC conversion
    // Set prescaler to 33 ms and enable overflow interrupt
    TIM1_overflow_33ms();
    TIM1_overflow_interrupt_enable();

    // Enables interrupts by setting the global interrupt mask
    sei();



    // Infinite loop
    while (1)
    {
        /* Empty loop. All subsequent operations are performed exclusively 
         * inside interrupt service routines ISRs */
    }

    // Will never reach this
    return 0;
}


/* Interrupt service routines ----------------------------------------*/
/**********************************************************************
 * Function: Timer/Counter1 overflow interrupt
 * Purpose:  Use Single Conversion mode and start conversion every 100 ms.
 **********************************************************************/
ISR(TIMER1_OVF_vect)
{
    // Start ADC conversion
    ADCSRA = ADCSRA | (1<<ADSC);

uint16_t b,r;

char string[4];
    
    b= GPIO_read(&PIND,Js_button);
    if (b ==0)
    {
        
        r=1;
        itoa(r, string, 10);
        lcd_gotoxy(13, 0);
        lcd_puts("    ");
        lcd_gotoxy(13, 0);
        lcd_puts(string);
    }

    if (b ==1)
    {
        
        r=0;
        itoa(r, string, 10);
        lcd_gotoxy(13, 0);
        lcd_puts("    ");
        lcd_gotoxy(13, 0);
        lcd_puts(string);
    }



    //Encoder 2
    current_state = GPIO_read(&PIND,enc_clk_2);

    if (current_state != LastState && current_state==1)
    {
        if(GPIO_read(&PIND,enc_DT_2) != current_state)
        {
            counter ++;
        }

        else
        {
            counter --;
        }

        itoa(counter, string, 10);
        lcd_gotoxy(9, 1);
        lcd_puts("    ");    
        lcd_gotoxy(9, 1);
        lcd_puts(string);
    }
    LastState = current_state;
}


/**********************************************************************
 * Function: ADC complete interrupt
 * Purpose:  Display converted value on LCD screen.
 **********************************************************************/
ISR(ADC_vect)
{
    static uint8_t channel = 0;
    uint16_t x, y;
    char string[4];  // String for converted numbers by itoa()


    // Read converted value
    // Note that, register pair ADCH and ADCL can be read as a 16-bit value ADC
    if (channel == 0)
    {
        ADMUX = ADMUX & ~(1<<MUX3 | 1<<MUX2 | 1<<MUX1 | 1<<MUX0);
        ADCSRA = ADCSRA | (1<<ADSC);
        x = ADC;
        // Convert "value" to "string" and display it
        itoa(x, string, 10);
        lcd_gotoxy(8, 0);
        lcd_puts("    ");
        lcd_gotoxy(8, 0);
        lcd_puts(string);
        channel++;
    }
    
    if (channel == 1)
    {
        ADMUX = ADMUX & ~(1<<MUX3 | 1<<MUX2 | 1<<MUX1); ADMUX = ADMUX | (1<<MUX0);
        ADCSRA = ADCSRA | (1<<ADSC);
        y = ADC;
        // Convert "value" to "string" and display it
        itoa(y, string, 10);
        lcd_gotoxy(5, 1);
        lcd_puts("    ");
        lcd_gotoxy(5, 1);
        lcd_puts(string);
        channel =0;
    }
}