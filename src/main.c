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

#include "UART.h"

void INTERRUPT_INIT(); // habilita la interrupcion que es disparada por la lectura
void TEMP_INIT();      // inicializa el adc para la medición

unsigned int temperature = 0;
char message[] = "Temp: 26°C \n";

int main(void)
{
  USART_init();     // Llama la rutina de inicializaci�n de la USART
  INTERRUPT_INIT(); // habilita y configura las interrupciones
  TEMP_INIT();      // habilita el ADC para la medicion de la temperatura

  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Modo de bajo consumo: power-down

  while (1)
  {
    sleep_mode();
  }
  return 0;
}

void INTERRUPT_INIT()
{
  TCCR1B |= (1 << WGM12);
  OCR1A = 15625;
  TIMSK1 |= (1 << OCIE1A);
  TCCR1B |= (1 << CS12) | (1 << CS10);
  sei();
}

void TEMP_INIT()
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
  sprintf(message, "Temp: %d°C \n", temperature);

  USART_putstring(message);
}