/*
 * main.c
 * TPA1
 *
 * La salida de la señal está en el PB1,
 * Osea el pin 9 en el Arduino UNO
 *
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define ADC_INTERNAL_REFERENCE 1100

#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "definitions.h"
#include "uart.h"

#define MAX_SAMPLES 200

// este es el buffer de 8 bits que contiene mi
// señal pseudo aleatoria
volatile uint8_t prbs_buffer = 127;

volatile uint16_t timer0_counter;
uint16_t system_output_mv = 0;
char *debug_output = "TEST\n";

uint16_t sample_counter = 0;
uint16_t MUESTRAS[MAX_SAMPLES] = {};
volatile uint8_t val;

int main(void)
{
  init_timer0();
  // USART_init();
  sei();

  DDRB |= (1 << PB1); // salida

  // sprintf(debug_output, "prbs_buffer: '%d'\r\n", prbs_buffer);
  // USART_putstring(debug_output);

  while (1)
  {
  }
  return 0;
}

// dispara cada 1ms
void init_timer0()
{
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);
}

void init_adc5()
{
  ADMUX = (1 << REFS0);
  ADMUX |= (1 << MUX2) | (1 << MUX0);
  ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);

  // se hace una lectura para finalizar el seteo del registro
  ADCSRA |= (1 << ADSC);
  while ((ADCSRA & (1 << ADSC)) != 0)
    ;
}

void read_adc5()
{
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC))
  {
  };
  system_output_mv = (uint16_t)((4.88 * ADC) + 1);
}

// se dispara cada 1ms
ISR(TIMER0_COMPA_vect)
{
  timer0_counter++;
  if (timer0_counter < 200)
  {
    return;
  }
  timer0_counter = 0;

  // Se aplica la señal pseudo aleatoria
  val = update_prbs();
  if (val)
  {
    PORTB |= (1 << PB1);
  }
  else
  {
    PORTB &= ~(1 << PB1);
  }

  // PORTB ^= (1 << PB4);
}

uint8_t update_prbs()
{
  prbs_buffer = (prbs_buffer << 1) + (((prbs_buffer >> 3) ^ (prbs_buffer >> 6)) & (0x1));
  return (prbs_buffer >> 7);
}