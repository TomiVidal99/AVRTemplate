#include <avr/io.h>
#include <util/delay.h>

int main(void) {

    while (1) {
        PORTB ^= (1 << PB7);
        _delay_ms(1000);
    }

    return 0;
}