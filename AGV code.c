#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
int main(void)
{
    DDRB |= (1<<PB7); //output led 1
    PORTF &=~(1<<PF2);
    PORTB &= ~(1<<PB7); //led 1 uit
	while (1)
	{
		if ((PINF &(1<<PF2))==0)
		{
		PORTB |= (1<<PB7); //led 1 aan
		_delay_ms(200);
		PORTB &=~(1 << PB7);
		_delay_ms(200);
		}
	}

  return 0;

}
