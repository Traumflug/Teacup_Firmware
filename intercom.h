#ifndef	_INTERCOM_H
#define	_INTERCOM_H

#include	<stdint.h>
#include	"config.h"

#ifdef MOTHERBOARD
	#define enable_transmit()			do { WRITE(TX_ENABLE_PIN,1);  UCSR1B &=~MASK(RXEN1); } while(0)
	#define disable_transmit()			do { WRITE(TX_ENABLE_PIN,0);  UCSR1B &= ~(MASK(TXCIE1) | MASK(UDRIE1)); UCSR1B |= MASK(RXEN1); } while(0)
#else
	#define enable_transmit()			do { WRITE(TX_ENABLE_PIN,1);  UCSR0B &= ~MASK(RXEN0); } while(0)
	#define disable_transmit()			do { WRITE(TX_ENABLE_PIN,0);  UCSR0B &= ~(MASK(TXCIE0) | MASK(UDRIE0)); UCSR0B |= MASK(RXEN0); } while(0)
#endif

/// list of error codes, not many so far...
enum {
	ERROR_BAD_CRC
} err_codes;

/** \brief intercom packet structure, both tx and rx
*/
typedef struct {
	uint8_t		start; ///< start byte, must be 0x55
	uint8_t		dio;	///< gpio
	uint8_t		controller_num; ///< controller index
	uint8_t		control_word;		///< what to do
	uint8_t		control_index;	///< what to do it to
	/// data with which to do it
	union {
		int32_t		control_data_int32;
		uint32_t	control_data_uint32;
		float			control_data_float;
		uint16_t	temp[2];
	};
	uint8_t		err;	///< error code, if any
	uint8_t		crc;	///< crc for packet verification. packets with bad crc are simply ignored
} intercom_packet_t;

/// this allows us to fill the packet struct, then pass it to something that expects an array of bytes
typedef union {
	intercom_packet_t packet;
	uint8_t						data[sizeof(intercom_packet_t)];
} intercom_packet;

extern intercom_packet tx;
extern intercom_packet rx;

/// initialise serial subsystem
void intercom_init(void);

/// if host, send target temperature to extruder
/// if extruder, send actual temperature to host
void send_temperature(uint8_t index, uint16_t temperature);

/// if host, read actual temperature from extruder
/// if extruder, read target temperature from host
uint16_t read_temperature(uint8_t index);

/// if host, set DIOs on extruder controller
/// if extruder, report DIO state
void set_dio(uint8_t index, uint8_t value);

/// if host, read extruder DIO inputs
/// if extruder, set DIO outputs
uint8_t	get_dio(uint8_t index);

/// set error code to send to other end
void set_err(uint8_t err);

/// get error code sent from other end
uint8_t get_err(void);

/// if host, send packet to extruder
/// if extruder, return packet to host
void start_send(void);

#define	FLAG_RX_IN_PROGRESS	1
#define	FLAG_TX_IN_PROGRESS	2
#define FLAG_NEW_RX					4
#define	FLAG_TX_FINISHED		8
extern volatile uint8_t	intercom_flags;



#endif	/* _INTERCOM_H */
