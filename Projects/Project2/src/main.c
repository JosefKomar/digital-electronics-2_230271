#include <Arduino.h>
#include <avr/io.h>         // AVR device-specific IO definitions
#include <avr/interrupt.h>  // Interrupts standard C library for AVR-GCC
#include <gpio.h>           // GPIO library for AVR-GCC
#include "timer.h"          // Timer library for AVR-GCC
#include <stdlib.h>
#include "adc.h"
#include <lcd.h>            // Peter Fleury's LCD library

// JOYSTICK
// OutputVRX PC1
// OutputVRY PC0
#define OutputSWJ PD2

#define servo_1 PB2
#define servo_2 PB3

#define SERVO_DEFAULT 1500
#define SERVO_MIN 600
#define SERVO_MAX 2400
#define SERVO_STEP 25

volatile uint32_t servo1_position = SERVO_DEFAULT;
volatile uint32_t servo2_position = SERVO_DEFAULT;

uint8_t overflow = 0;

int main(void) 
{
  GPIO_mode_output(&DDRB, servo_1);
  GPIO_mode_output(&DDRB, servo_2);
  GPIO_mode_input_pullup(&DDRD,OutputSWJ);
  
  // Initialize display
    lcd_init(LCD_DISP_ON);
    lcd_gotoxy(0, 0);
    lcd_puts("Counter:");

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


  TCCR1A |= (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B |= (1 << WGM13);
  ICR1 = 20000;
  OCR1A = servo1_position;
  OCR1B = servo2_position;
  TCCR1B |= (1 << CS11);


  // Set encoder and joystick button pins (INT0/1) as inputs with pull-up resistor
  //DDRD &= ~(1 << PD2);
  //PORTD |= (1 << PD2);
  // Set activation of both external interrupts to any logical change
  EICRA |= (1 << ISC00);
  // Enable external interrupts INT0 and INT1 
  EIMSK |= (1 << INT0);
  
  
  
  /*TCCR1B |= (1 << WGM12);
  TCCR1B &= ~(1 << WGM13);
  TCCR1A |= (1 << WGM10) | (1 << WGM11);

  TCCR1A |=  (1<<COM1A1);
  TCCR1A &= ~(1<<COM1A0);

  //TCCR1A=0b10100010;//COM1A0,COM1B0 are 0, COM1A1, COM1B1 are 0 0
  //also WGM10 are 0 and WGM11=1
  
  //TCCR1B &=~ ((1<<ICNC1)|(1<<ICES1));  
  TCCR1B |= (1<<CS10);
  TCCR1B &= ~(1<<CS11);
  TCCR1B &= ~(1<<CS12);
  // ICR1=20000;
 // DDRB |= (1<<DDB1);
*/
  TIM1_overflow_33ms();
  TIM1_overflow_interrupt_enable();
  sei();
  
  while(1)
  {

  }
  return 0;
}

ISR(TIMER1_OVF_vect)
{

  
  static int8_t nooverflow = 0;
  nooverflow++;
  if(nooverflow > 1)
  {
    nooverflow = 0;
    ADCSRA |= (1 << ADSC);
  }
 
}
ISR(ADC_vect)
{
    /*char charX[4];
    char charY[4];
    uint16_t valueX;
    uint16_t valueY;
    uint8_t channel = 0;
    */
    /* address shifting ADMUX -----------------------------------------*/
    /*
  if (channel == 0){
    // Select input channel ADC0 (voltage divider pin)
    ADMUX = ADMUX & ~( 1<<MUX3 | 1<<MUX2 | 1<<MUX0 | 1<<MUX1);
    ADCSRA |= (1<<ADSC);
    valueX = ADC;

    if (valueX > 750 && servo1_position <= SERVO_MAX) 
    {
        servo1_position += SERVO_STEP;
    }
    else if (valueX < 250 && servo1_position >= SERVO_MIN)
    {
        servo1_position -= SERVO_STEP;
    }
    itoa(valueX, charX, 10);
    lcd_gotoxy(0,1);
    lcd_puts("       ");
    lcd_gotoxy(0,1);
    lcd_puts("X:");
    lcd_gotoxy(2,1);
    lcd_puts(charX);

    channel = 1;
    
  }
  if (channel == 1){
    // Select input channel ADC1 (voltage divider pin)
    ADMUX &= ~((1<<MUX3 | 1<<MUX2 | 1<<MUX1)); ADMUX |= (1<<MUX0);
    ADCSRA |= (1<<ADSC);
    valueY = ADC;
    
     if (valueY < 250 && servo2_position <= SERVO_MAX) 
    {
        servo2_position += SERVO_STEP;
    }
    else if (valueY > 750 && servo2_position >= SERVO_MIN)
    {
        servo2_position -= SERVO_STEP;
    }
    itoa(valueY, charY, 10);
    lcd_gotoxy(7,1);
    lcd_puts("       ");
    lcd_gotoxy(7,1);
    lcd_puts("Y:");
    lcd_gotoxy(9,1);
    lcd_puts(charY);

    channel = 0;
    
  } 
    OCR1A = servo1_position;
    OCR1B = servo2_position;
    */
    uint16_t value;
    value = ADC;

    uint8_t channel = adc_get_current_channel();
    // Move the player according to sensed value from the current channel 
    switch (channel)
    {
        case 0:
            if (value > 750 && servo1_position <= SERVO_MAX) 
            {
                servo1_position += SERVO_STEP;
            }
            else if (value < 250 && servo1_position >= SERVO_MIN)
            {
                servo1_position -= SERVO_STEP;
            }

            // Switch ADC to the channel 1 and start the conversion again
            adc_select_channel(1);
            adc_start_conversion();
            break;
        
        case 1:
            if (value < 250 && servo2_position <= SERVO_MAX) 
            {
                servo2_position += SERVO_STEP;
            }
            else if (value > 750 && servo2_position >= SERVO_MIN)
            {
                servo2_position -= SERVO_STEP;
            }
            // Switch the ADC back to the channel 0
            adc_select_channel(0);
            break;
    }
    OCR1A = servo1_position;
    OCR1B = servo1_position;
}