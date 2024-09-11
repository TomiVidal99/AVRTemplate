/*
 * main.c
 *
 * Created: 14/7/2024 22:03:25
 * Author : Tomas Vidal
 * Control básico de planta
 *
 * Que hace?
 * Con el Timer 1 (16 bits) se genera una interrupcion cada 1ms y un PWM de esa frecuencia
 * el callback de la interrupcion ajusta el ciclo de trabajo del PWM (accion de control)
 * a partir de leer el ADC5 y con calcular ajusta el PWM.
 */

#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "../include/definitions.h"

#define GAIN_FACTOR 10
#define MIN_GAIN_FACTOR 80

uint16_t system_output_mv;

int main(void)
{
  init_adc5();
  init_timer1();
  sei();

  DDRD |= (1 << PD1);

  // set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Modo de bajo consumo: power-down

  set_pwm_duty_cycle(50);

  while (1)
  {
    // sleep_mode();
  }
  return 0;
}

void init_timer1()
{
  // Set PB1 (OC1A) as output
  DDRB |= (1 << PB1);

  // Configure Timer 1 for Fast PWM mode with ICR1 as TOP
  // WGM13:0 = 1110 for Fast PWM with ICR1 as top
  TCCR1A = (1 << COM1A1) | (1 << WGM11);              // Clear OC1A on Compare Match, set OC1A at BOTTOM (non-inverting mode)
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10); // Fast PWM, no prescaler

  // Set ICR1 for 1 kHz frequency (1 ms period)
  ICR1 = 15999; // (F_CPU / (Prescaler * Frequency)) - 1 = 16000000 / (1 * 1000) - 1

  // Enable Timer 1 overflow interrupt
  TIMSK1 = (1 << TOIE1);

  // Enable global interrupts
  sei();
}

void set_pwm_duty_cycle(uint8_t duty_cycle)
{
  // Limit duty cycle to 0-100%
  if (duty_cycle > 100)
  {
    duty_cycle = 100;
  }

  // Calculate OCR1A value based on duty cycle percentage
  OCR1A = (uint16_t)(((uint32_t)duty_cycle * (ICR1 + 1)) / 100);
}

// Timer1 Overflow ISR, called every 1 ms
uint16_t control_action = 0;
ISR(TIMER1_OVF_vect)
{
  // Code to run every 1 ms
  // Example: Toggle an LED or perform some task

  PORTD ^= (1 << PD1);

  read_adc5();

  control_action = GAIN_FACTOR * (5000 / (system_output_mv * 100)) + MIN_GAIN_FACTOR;
  set_pwm_duty_cycle(control_action);
}

void init_adc5()
{
  // Select Vref = AVcc with external capacitor at AREF pin
  ADMUX = (1 << REFS0);

  // Select ADC5 (PC5) as input channel (MUX bits = 0101)
  ADMUX |= (1 << MUX2) | (1 << MUX0);

  // Enable ADC and set prescaler to 128 for 125 kHz ADC clock (16 MHz / 128)
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void read_adc5()
{
  // Start an ADC conversion by setting ADSC bit
  ADCSRA |= (1 << ADSC);

  // Wait for the conversion to complete (ADSC becomes '0' again)
  while (ADCSRA & (1 << ADSC))
    ;

  // Read the ADC value (10-bit result)

  system_output_mv = (uint16_t)((5000 * ADC) / 1023);
}