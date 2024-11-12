#include <stdint.h>

void init_adc5();
void init_timer0();
void init_timer1();
void read_adc5();
void set_pwm_duty_cycle(uint8_t);

uint8_t update_prbs();