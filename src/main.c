/*
 * TP3_problema_2.c
 *
 * Created: 14/7/2024 22:03:25
 * Author : Tomas Vidal
 * Control basico
 */

#define F_CPU 16000000UL
#define SCALE_FACTOR 1

#include <stdio.h>

#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <util/delay.h>

#include "UART.h"

void PWM_update(void);        // actualiza la salida PWM en base al duty_cycle
void PWM_init(void);          // inicializa el timer del pwm en fast mode
void interrupt_init();        // habilita la interrupcion que es disparada por la lectura
void adc_init();              // inicializa el adc para la medición
void controller_update(void); // actualiza la salida PWM en base a una logica de control

uint8_t temperature = 0;
uint8_t duty_cycle = 0;
char message[] = "Temp: 26°C \n";

int main(void)
{
  USART_init();     // Llama la rutina de inicializaci?n de la USART
  interrupt_init(); // habilita y configura las interrupciones
  adc_init();       // habilita el ADC para la medicion de la temperatura

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Modo de bajo consumo: power-down

  while (1)
  {
    sleep_mode();
  }
  return 0;
}

void interrupt_init()
{
  TCCR0A |= (1 << WGM01);
  TCCR0B |= (1 << WGM02);

  OCR0A = 249;

  TIMSK0 |= (1 << OCIE0A);

  TCCR0B |= (1 << CS01) | (1 << CS00);

  sei();
}

void adc_init()
{
  ADMUX = (1 << REFS1) | (1 << REFS0) | (0 << ADLAR) | (1 << MUX3) | (0 << MUX2) | (0 << MUX1) | (0 << MUX0);
  ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADEN);
  ADCSRA |= (1 << ADSC);
  while ((ADCSRA & (1 << ADSC)) != 0)
  {
  }
}

ISR(TIMER1_COMPA_vect)
{
  ADCSRA |= (1 << ADSC); // se empieza a leer la temperatura con el conversor
  while ((ADCSRA & (1 << ADSC)) != 0)
  {
  } // se espera a la conversion
  temperature = ADC - 273;
  // sprintf(message, "Temp: %d°C \n", temperature);

  // USART_putstring(message);
  //
  //  generar PWM acorde a una logica de control
  //  proporcional a la lectura de la temperatura
  controller_update();
}

void PWM_update(void)
{
  OCR1A = (uint16_t)((duty_cycle * ICR1) / 100);
}

void PWM_init(void)
{
  TCCR1A |= (1 << COM1A1) | (1 << WGM11);
  TCCR1B |= (1 << WGM12) | (1 << WGM13) | (1 << CS11);

  ICR1 = 19999;

  DDRB |= (1 << PB1);
}

void controller_update(void)
{
  duty_cycle = SCALE_FACTOR * temperature;

  if (duty_cycle > 100)
  {
    duty_cycle = 100;
  }
  else if (duty_cycle < 0)
  {
    duty_cycle = 0;
  }

  PWM_update();
}