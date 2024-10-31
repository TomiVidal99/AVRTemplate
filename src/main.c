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
 *
 *
 * ENTRADA (LECTURA DE ADC): ADC5
 * SALIDA (FAST PWM DE TIMER1): PB1
 *
 * OTRAS SALIDAS/ENTRADAS (para debugear):
 * RX, TX, PB4, PB6
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

volatile uint16_t timer0_counter;
uint16_t system_output_mv = 0;
char *debug_output = "TEST\n";

int main(void)
{
  init_adc5();
  init_timer1();
  init_timer0();
  USART_init();
  sei();

  DDRB |= (1 << PB4);
  DDRB |= (1 << PB6);

  // set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Modo de bajo consumo: power-down

  set_pwm_duty_cycle(90);

  while (1)
  {
    // sleep_mode();
  }
  return 0;
}

void init_timer0()
{
  TCCR0A = (1 << WGM01); // CTC mode
  TCCR0B = (1 << CS01) | (1 << CS00);
  OCR0A = 249;
  TIMSK0 = (1 << OCIE0A);
}

void init_timer1()
{
  DDRB |= (1 << PB1);
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
  ICR1 = 15999;

  TIMSK1 = (1 << TOIE1);
}

void set_pwm_duty_cycle(uint8_t duty_cycle)
{
  if (duty_cycle > 100)
  {
    duty_cycle = 100;
  }

  OCR1A = (uint16_t)(((uint32_t)duty_cycle * (ICR1 + 1)) / 100);
}

ISR(TIMER1_OVF_vect)
{
  // esta es una señal de referencia para saber cuando
  // se hace la interrupcion cada 1ms
  PORTB ^= (1 << PB6);

  read_adc5();

  // esta es la accion de control (ahora hice una logica muy simple)
  // acá iría el diseño del controlador
  if (system_output_mv < 500)
  {
    set_pwm_duty_cycle(90);
  }
  else if (system_output_mv > 3500)
  {
    set_pwm_duty_cycle(1);
  }
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

ISR(TIMER0_COMPA_vect)
{
  timer0_counter++;
  if (timer0_counter < 1000)
  {
    return;
  }
  timer0_counter = 0;
  sprintf(debug_output, "lectura: %d\n\r", system_output_mv);
  USART_putstring(debug_output);
  PORTB ^= (1 << PB4);
}