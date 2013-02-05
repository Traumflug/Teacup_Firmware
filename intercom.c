#include	"intercom.h"

/** \file
	\brief motherboard <-> extruder board protocol
*/

#include	<avr/io.h>
#include	<avr/interrupt.h>
#include	"memory_barrier.h"

#include	"config.h"
#include	"delay.h"

#if	 (defined TEMP_INTERCOM) || (defined EXTRUDER)
#define		INTERCOM_BAUD			57600

#define	START	0x55

intercom_packet tx;		///< this packet will be sent
intercom_packet rx;		///< the last received packet with correct checksum
intercom_packet _tx;	///< current packet in transmission
intercom_packet _rx;	///< current packet being received

/// pointer to location in transferring packet.
/// since we run half duplex, we can share the pointer between rx and tx
uint8_t packet_pointer;
/// crc for currently transferring packet
/// since we run half duplex, we can share the crc between rx and tx
uint8_t	rxcrc;

volatile uint8_t	intercom_flags;

void intercom_init(void)
{
#ifdef MOTHERBOARD
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

	intercom_flags = 0;
}

void send_temperature(uint8_t index, uint16_t temperature) {
	tx.packet.temp[index] = temperature;
}

uint16_t read_temperature(uint8_t index) {
	return rx.packet.temp[index];
}

#ifdef MOTHERBOARD
void set_dio(uint8_t index, uint8_t value) {
	if (value)
		tx.packet.dio |= (1 << index);
	else
		tx.packet.dio &= ~(1 << index);
}
#else
uint8_t	get_dio(uint8_t index) {
	return rx.packet.dio & (1 << index);
}
#endif

void set_err(uint8_t err) {
	tx.packet.err = err;
}

uint8_t get_err() {
	return rx.packet.err;
}

void start_send(void) {
	uint8_t txcrc = 0, i;

	// atomically update flags
	uint8_t sreg = SREG;
	cli();
	intercom_flags = (intercom_flags & ~FLAG_TX_FINISHED) | FLAG_TX_IN_PROGRESS;
	SREG = sreg;

	// enable transmit pin
	enable_transmit();

	// set start byte
	tx.packet.start = START;

	// set packet type
	tx.packet.control_word = 105;
	tx.packet.control_index = 0;

	// calculate CRC for outgoing packet
	for (i = 0; i < (sizeof(intercom_packet_t) - 1); i++) {
		txcrc ^= tx.data[i];
	}
	tx.packet.crc = txcrc;

	for (i = 0; i < (sizeof(intercom_packet_t) ); i++) {
		_tx.data[i] = tx.data[i];
	}

	packet_pointer = 0;

	// actually start sending the packet
	#ifdef MOTHERBOARD
		UCSR1B |= MASK(UDRIE1);
	#else
		UCSR0B |= MASK(UDRIE0);
	#endif
}

/*
	Interrupts, UART 0 for mendel
*/

// receive data interrupt- stuff into rx
#ifdef MOTHERBOARD
ISR(USART1_RX_vect)
#else
ISR(USART_RX_vect)
#endif
{
	// save status register
	uint8_t sreg_save = SREG;

	// pull character
	static uint8_t c;

	#ifdef MOTHERBOARD
		c = UDR1;
		UCSR1A &= ~MASK(FE1) & ~MASK(DOR1) & ~MASK(UPE1);
	#else
		c = UDR0;
		UCSR0A &= ~MASK(FE0) & ~MASK(DOR0) & ~MASK(UPE0);
	#endif

	// are we waiting for a start byte? is this one?
	if ((packet_pointer == 0) && (c == START)) {
		rxcrc = _rx.packet.start = START;
		packet_pointer = 1;
		intercom_flags |= FLAG_RX_IN_PROGRESS;
	}
	else if (packet_pointer > 0) {
		// we're receiving a packet
		// calculate CRC (except CRC character!)
		if (packet_pointer < (sizeof(intercom_packet_t) - 1))
			rxcrc ^= c;
		// stuff byte into structure
		_rx.data[packet_pointer++] = c;
		// last byte?
		if (packet_pointer >= sizeof(intercom_packet_t)) {
			// reset pointer
			packet_pointer = 0;

			#ifndef MOTHERBOARD
			if (rxcrc == _rx.packet.crc &&
			    _rx.packet.controller_num == THIS_CONTROLLER_NUM){
			#else
			if (rxcrc == _rx.packet.crc){
			#endif
				// correct crc copy packet
				static uint8_t i;
				for (i = 0; i < (sizeof(intercom_packet_t) ); i++) {
					rx.data[i] = _rx.data[i];
				}
			}

			#ifndef MOTHERBOARD
				if (rx.packet.controller_num == THIS_CONTROLLER_NUM) {
					if (rxcrc != _rx.packet.crc)
						tx.packet.err = ERROR_BAD_CRC;
					else
						intercom_flags = (intercom_flags & ~FLAG_RX_IN_PROGRESS) | FLAG_NEW_RX;
					// not sure why exactly this delay is needed, but wihtout it first byte never arrives.
// 					delay_us(150);
// 					start_send();
				}
			#else
				intercom_flags = (intercom_flags & ~FLAG_RX_IN_PROGRESS) | FLAG_NEW_RX;
			#endif
		}
	}

	// restore status register
	MEMORY_BARRIER();
	SREG = sreg_save;
}

// finished transmitting interrupt- only enabled at end of packet
#ifdef MOTHERBOARD
ISR(USART1_TX_vect)
#else
ISR(USART_TX_vect)
#endif
{
	// save status register
	uint8_t sreg_save = SREG;

	if (packet_pointer >= sizeof(intercom_packet_t)) {
		disable_transmit();
		packet_pointer = 0;
		intercom_flags = (intercom_flags & ~FLAG_TX_IN_PROGRESS) | FLAG_TX_FINISHED;
		#ifdef MOTHERBOARD
			UCSR1B &= ~MASK(TXCIE1);
		#else
			UCSR0B &= ~MASK(TXCIE0);
		#endif
	}

	// restore status register
	MEMORY_BARRIER();
	SREG = sreg_save;
}

// tx queue empty interrupt- send next byte
#ifdef MOTHERBOARD
ISR(USART1_UDRE_vect)
#else
ISR(USART_UDRE_vect)
#endif
{
	// save status register
	uint8_t sreg_save = SREG;

	#ifdef	MOTHERBOARD
	UDR1 = _tx.data[packet_pointer++];
	#else
	UDR0 = _tx.data[packet_pointer++];
	#endif

	if (packet_pointer >= sizeof(intercom_packet_t)) {
		#ifdef MOTHERBOARD
			UCSR1B &= ~MASK(UDRIE1);
			UCSR1B |= MASK(TXCIE1);
		#else
			UCSR0B &= ~MASK(UDRIE0);
			UCSR0B |= MASK(TXCIE0);
		#endif
	}

	// restore status register
	MEMORY_BARRIER();
	SREG = sreg_save;
}

#endif	/* TEMP_INTERCOM */
