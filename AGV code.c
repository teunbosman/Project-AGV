#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define KNOP1 PF2
#define KNOP2 PF1
#define LED1 PB7
#define LED2 PB6

void init()
{
    //led
    DDRB |= (1 << LED1); //output led 1
    DDRB |= (1 << LED2); //output led 2
    PORTB &= ~(1 << LED1); //led 1 uit
    PORTB &= ~(1 << LED2); //led 2 uit
	
    //knoppen
    PORTF &= ~(1 << KNOP1); //knop 1
    PORTF &= ~(1 << KNOP2); //knop 2
    
	
}

int main(void)
{
	while (1)
	{
		if ((PINF & (1 << KNOP1)) !=0)
		{
		PORTB |= (1 << LED1); //led aan
		}
			
		else {
		PORTB &=~(1 << LED1); //led uit
		}

		if((PINF & (1 << KNOP1) && (PINF & (1 << KNOP2)) != 0)
		{
			PORTB ^= (1 << LED2); //led toggle
		}
	}
  return 0;
}
