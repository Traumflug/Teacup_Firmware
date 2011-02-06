#include	"intercom.h"

#include	<avr/interrupt.h>

#include	"config.h"
#include	"delay.h"

#ifdef	TEMP_INTERCOM
#define		INTERCOM_BAUD			57600

#define enable_transmit()			do { WRITE(TX_ENABLE_PIN,1);  WRITE(RX_ENABLE_PIN,0); } while(0)
#define disable_transmit()			do { WRITE(TX_ENABLE_PIN,0);  WRITE(RX_ENABLE_PIN,0); } while(0)

/*
 Defines a super simple intercom interface using the RS485 modules

 Host will say: START1 START2 PWM_CMD PWM_CHK 
 Extruder will reply: START1 START2 TMP_CMD TMP_CHK 

 CHK = 255-CMD, if they match do the work, if not, ignore this packet

 in a loop
*/


#define		START1	0xAA
#define		START2	0x55

typedef enum {
	SEND_START1,
	SEND_START2,
	SEND_CMD,
	SEND_CHK,
	SEND_DONE,

	READ_START1,
	READ_START2,
	READ_CMD,
	READ_CHK,
} intercom_state_e;


intercom_state_e state = READ_START1;
uint8_t cmd, chk, send_cmd, read_cmd;

void intercom_init(void)
{
#ifdef HOST
	#if INTERCOM_BAUD > 38401
		UCSR1A = MASK(U2X1);
		UBRR1 = (((F_CPU / 8) / INTERCOM_BAUD) - 0.5);
	#else
		UCSR1A = 0;
		UBRR1 = (((F_CPU / 16) / INTERCOM_BAUD) - 0.5);
	#endif
	UCSR1B = MASK(RXEN1) | MASK(TXEN1);
	UCSR1C = MASK(UCSZ11) | MASK(UCSZ10);

	UCSR1B |= MASK(RXCIE1) | MASK(TXCIE1);
#else
	#if INTERCOM_BAUD > 38401
		UCSR0A = MASK(U2X0);
		UBRR0 = (((F_CPU / 8) / INTERCOM_BAUD) - 0.5);
	#else
		UCSR0A = 0;
		UBRR0 = (((F_CPU / 16) / INTERCOM_BAUD) - 0.5);
	#endif
	UCSR0B = MASK(RXEN0) | MASK(TXEN0);
	UCSR0C = MASK(UCSZ01) | MASK(UCSZ00);

	UCSR0B |= MASK(RXCIE0) | MASK(TXCIE0);
#endif
}

void update_send_cmd(uint8_t new_send_cmd) {
	send_cmd = new_send_cmd;
}

uint8_t get_read_cmd(void) {
	return read_cmd;
}

static void write_byte(uint8_t val) {
#ifdef HOST
	UDR1 = val;
#else
	UDR0 = val;
#endif
}


void start_send(void) {
	state = SEND_START1;
	enable_transmit();
	delay_us(15);
	//Enable interrupts so we can send next characters
#ifdef HOST
	UCSR1B |= MASK(UDRIE1);
#else
	UCSR0B |= MASK(UDRIE0);
#endif
}

static void finish_send(void) {
	state = READ_START1;
	disable_transmit();
}


/*
	Interrupts, UART 0 for mendel
*/
#ifdef HOST
ISR(USART1_RX_vect)
#else
ISR(USART_RX_vect)
#endif
{
	static uint8_t c;

#ifdef HOST
	c = UDR1;
	UCSR1A &= ~MASK(FE1) & ~MASK(DOR1) & ~MASK(UPE1);
#else
	c = UDR0;
	UCSR0A &= ~MASK(FE0) & ~MASK(DOR0) & ~MASK(UPE0);
#endif
	
	if (state >= READ_START1) {
		
		switch(state) {
		case READ_START1:
			if (c == START1) state = READ_START2;
			break;
		case READ_START2:
			if (c == START2) state = READ_CMD;
			else			 state = READ_START1;
			break;
		case READ_CMD:
			cmd = c;
			state = READ_CHK;
			break;
		case READ_CHK:
			chk = c;
					
			if (chk == 255 - cmd) {	
				//Values are correct, do something useful
			WRITE(DEBUG_LED,1);	
				read_cmd = cmd;
#ifdef EXTRUDER
				start_send();
#endif
			}
			else
			{
				state = READ_START1;
			}
			break;
		default:
			break;
		}
	}

}

#ifdef HOST
ISR(USART1_TX_vect)
#else
ISR(USART_TX_vect)
#endif
{
	if (state == SEND_DONE) {
		finish_send();
		
					
#ifdef HOST
	UCSR1B &= ~MASK(TXCIE1);
#else
	UCSR0B &= ~MASK(TXCIE0);
#endif
	}
}

#ifdef HOST
ISR(USART1_UDRE_vect)
#else
ISR(USART_UDRE_vect)
#endif
{
	switch(state) {
	case SEND_START1:
		write_byte(START1);
		state = SEND_START2;
		break;
	case SEND_START2:
		write_byte(START2);
		state = SEND_CMD;
		break;
	case SEND_CMD:
		write_byte(send_cmd);
		state = SEND_CHK;
		break;
	case SEND_CHK:
		write_byte(255 - send_cmd);
		state = SEND_DONE;
#ifdef HOST
	UCSR1B &= ~MASK(UDRIE1);
	UCSR1B |= MASK(TXCIE1);
#else
	UCSR0B &= ~MASK(UDRIE0);
	UCSR0B |= MASK(TXCIE0);
#endif
		break;
	default:
		break;
	}
}

#endif	/* TEMP_INTERCOM */
