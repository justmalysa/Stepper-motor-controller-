/*
 * stepper_motor.c
 */ 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define BAUD_RATE 4800
#define BAUD_PRESCALE ((F_CPU / (BAUD_RATE * 16UL)) - 1)
#define SEQ_SIZE 8

static char m_received[7];
static uint8_t m_rx_index;
static volatile uint16_t m_stepper_delay;
static volatile uint16_t m_stepper_position;

/* UART interrupt service routine:
   This function collect subsequent bytes received from UART interface.
   When the new line character is encountered, received bytes are interpreted as stepper motor control command.
   Supported commands are:
   - p%u - stepper motor position, denoted in pulses,
   - s%u - stepper motor speed, denoted in 10us delay between subsequent pulses.
*/
ISR(USART_RXC_vect)
{
	uint8_t byte = UDR;
	if( '\n' == byte)
	{
		m_received[m_rx_index] = '\0';
		if ('s' == m_received[0])
		{
			m_stepper_delay = (uint16_t)atoi(&m_received[1]);
			if(m_stepper_delay == INT16_MAX || m_stepper_delay == INT16_MIN)
			{
				m_stepper_delay = 0;
			}
		}
		else if ('p' == m_received[0])
		{
			m_stepper_position = (uint16_t)atoi(&m_received[1]);
		}
		m_rx_index = 0;
	}
	else
	{
		m_received[m_rx_index++] = byte;		
	}
}

/* UART configuration:
- baud rate: 4800
- data bits: 8
- parity: none
- stop bits: 1
*/
void init_USART(void)
{
	UBRRH = ( BAUD_PRESCALE >> 8);
	UBRRL = BAUD_PRESCALE;
	UCSRB = (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
	UCSRC = (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
}

/* Function for performing time delay, specified in the 10us units. */
void my_delay_us(uint16_t us)
{
	while(us--)
	{
		_delay_us(10);
	}
}

/* Function for sending impulses to the stepper motor coils with the specified parameters:
   - seq - array of coils sequence,
   - delay - delay between subsequent pulses in 10us units,
   - dif_position - number of impulses to send.
*/ 
void stepper_rotate(const uint8_t * seq, uint16_t delay, uint16_t dif_position)
{
	uint16_t count = 0;
	while( count < dif_position)
	{
		for (uint8_t i = 0; i<SEQ_SIZE; i++)
		{
			PORTC = seq[i];
			my_delay_us(delay);
		}
		count++;
	}
}


int main(void)
{
	init_USART();
	DDRC = 0x0F;
	
	/* Coils sequence described clock-wise rotation*/
	const uint8_t seq_cw[SEQ_SIZE] = {0x1, 0x3, 0x2, 0x6, 0x4, 0xc, 0x8, 0x9};
		
	/* Coils sequence described counter clock-wise rotation*/
	const uint8_t seq_ccw[SEQ_SIZE] = {0x9, 0x8, 0xc, 0x4, 0x6, 0x2, 0x3, 0x1};
	
	int16_t previous_position = 0;
    while (1) 
    {
		/*Copy values of global variables to local variables.
		As the global variables may be updated in-between assignments by UART ISR,
		temporarily disable interrupts.*/
		cli();
		uint16_t delay = m_stepper_delay;
		int16_t position = m_stepper_position;
		int16_t dif_position =  position - previous_position;
		sei();
		
		/*Choose correct sequence given position difference sign.*/
		if (dif_position > 0)
		{
			stepper_rotate(seq_cw, delay, (uint16_t)dif_position);
		}
		else if(dif_position < 0)
		{
			dif_position = -dif_position;
			stepper_rotate(seq_ccw, delay, (uint16_t)dif_position);
		}
		
		/*Save last position.*/
		previous_position = position;
    }
}

