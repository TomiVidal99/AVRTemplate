#include <avr/io.h>

int main(void) {

    while (1) {
        PORTB ^= (1 << PB7);
    }

    return 0;
}