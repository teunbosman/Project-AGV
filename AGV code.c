#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
int main(void)
{
    DDRB |= (1<<PB7); //output led 1
    
    PORTB &= ~(1<<PB7); //led 1 uit
	while (1)
	{
	  PORTB |= (1<<PB7); //led 1 aan
		_delay_ms(200);
		PORTB &=~(1 << PB7);
		_delay_ms(200);
	}

  return 0;

}
