#include <stdint.h>

void init_adc5();   // hago la lectura de la muestra con el ADC5
void init_ports();  // accion de control con el puerto PD2
void init_timer1(); // este timer es el 16 bits, hace el PWM (a 1KHz)
void init_timer0(); // este timer es de 8 bits, dispara las lecturas
void update_pwm();  // ajusta el ciclo de trabajo del PWM y lo muestra en la salida (PD2)