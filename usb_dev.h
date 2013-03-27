#ifndef _usb_dev_h_
#define _usb_dev_h_

// This header is NOT meant to be included when compiling
// user sketches in Arduino.  The low-level functions
// provided by usb_dev.c are meant to be called only by
// code which provides higher-level interfaces to the user.

#include "usb_mem.h"
#include "usb_desc.h"

#ifdef __cplusplus
extern "C" {
#endif

void usb_init(void);
void usb_isr(void);
usb_packet_t *usb_rx(uint32_t endpoint);
uint32_t usb_rx_byte_count(uint32_t endpoint);
uint32_t usb_tx_byte_count(uint32_t endpoint);
uint32_t usb_tx_packet_count(uint32_t endpoint);
void usb_tx(uint32_t endpoint, usb_packet_t *packet);
void usb_tx_isr(uint32_t endpoint, usb_packet_t *packet);

extern volatile uint8_t usb_configuration;

#ifdef CDC_DATA_INTERFACE
extern uint32_t usb_cdc_line_coding[2];
extern volatile uint8_t usb_cdc_line_rtsdtr;
extern volatile uint8_t usb_cdc_transmit_flush_timer;
extern void usb_serial_flush_callback(void);
#endif

#ifdef SEREMU_INTERFACE
extern volatile uint8_t usb_seremu_transmit_flush_timer;
extern void usb_seremu_flush_callback(void);
#endif

#ifdef KEYBOARD_INTERFACE
extern uint8_t keyboard_modifier_keys;
extern uint8_t keyboard_keys[6];
extern uint8_t keyboard_protocol;
extern uint8_t keyboard_idle_config;
extern uint8_t keyboard_idle_count;
extern volatile uint8_t keyboard_leds;
#endif

#ifdef MIDI_INTERFACE
extern void usb_midi_flush_output(void);
#endif

#ifdef FLIGHTSIM_INTERFACE
extern void usb_flightsim_flush_callback(void);
#endif





#ifdef __cplusplus
}
#endif



#endif
