/*
 * TP3_problema_2.c
 *
 * Created: 14/7/2024 22:03:25
 * Author : Tomas Vidal
 * La precision de la lectura es horrible
 */

#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "../include/definitions.h"

uint16_t adc_read;
uint8_t pwm_duty_cycle = 128;

int main(void)
{
  init_ports();
  init_adc5();
  init_timer1();
  init_timer0();

  // set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Modo de bajo consumo: power-down

  while (1)
  {
    // sleep_mode();
  }
  return 0;
}

void init_timer1()
{
  TCCR1B |= (1 << WGM12);
  OCR1A = 15625;
  TIMSK1 |= (1 << OCIE1A);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  sei();
}

void init_adc5()
{
  ADMUX = (1 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (0 << MUX3) | (1 << MUX2) | (0 << MUX1) | (1 << MUX0);
  ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADEN);
  ADCSRA |= (1 << ADSC);
  while ((ADCSRA & (1 << ADSC)) != 0)
  {
  }
}

ISR(TIMER1_COMPA_vect)
{
  ADCSRA |= (1 << ADSC);
  while ((ADCSRA & (1 << ADSC)) != 0)
  {
  }
  adc_read = ADC; // TODO: escalar correctamente

  update_pwm();
}

void init_ports()
{
  DDRD |= (1 << PD2);
}

void init_timer0()
{
  TCCR0A |= (1 << WGM00) | (1 << WGM01);
  TCCR0A |= (1 << COM0B1);
  TCCR0B |= (1 << CS00);
  OCR0B = pwm_duty_cycle;
}

void update_pwm()
{
  PORTD ^= (1 << PD2);
  // pwm_duty_cycle = (uint8_t)((adc_read * 255) / 1023);
  // OCR0B = pwm_duty_cycle;
}