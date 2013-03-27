#ifndef USBserial_h_
#define USBserial_h_

#if defined(USB_SERIAL) || defined(USB_SERIAL_HID)

#include <inttypes.h>

// C language implementation
#ifdef __cplusplus
extern "C" {
#endif
int usb_serial_getchar(void);
int usb_serial_peekchar(void);
int usb_serial_available(void);
void usb_serial_flush_input(void);
int usb_serial_putchar(uint8_t c);
int usb_serial_write(const void *buffer, uint32_t size);
void usb_serial_flush_output(void);
extern uint32_t usb_cdc_line_coding[2];
extern volatile uint8_t usb_cdc_line_rtsdtr;
extern volatile uint8_t usb_cdc_transmit_flush_timer;
extern volatile uint8_t usb_configuration;
#ifdef __cplusplus
}
#endif

#define USB_SERIAL_DTR  0x01
#define USB_SERIAL_RTS  0x02


// C++ interface
#ifdef __cplusplus
#include "Stream.h"
class usb_serial_class : public Stream
{
public:
        void begin(long) { /* TODO: call a function that tries to wait for enumeration */ };
        void end() { /* TODO: flush output and shut down USB port */ };
        virtual int available() { return usb_serial_available(); }
        virtual int read() { return usb_serial_getchar(); }
        virtual int peek() { return usb_serial_peekchar(); }
        virtual void flush() { usb_serial_flush_output(); }  // TODO: actually wait for data to leave USB...
        virtual size_t write(uint8_t c) { return usb_serial_putchar(c); }
        virtual size_t write(const uint8_t *buffer, size_t size) { return usb_serial_write(buffer, size); }
	using Print::write;
        void send_now(void) { usb_serial_flush_output(); };
        uint32_t baud(void) { return usb_cdc_line_coding[0]; }
        uint8_t stopbits(void) { uint8_t b = usb_cdc_line_coding[1]; if (!b) b = 1; return b; }
        uint8_t paritytype(void) { return usb_cdc_line_coding[1] >> 8; } // 0=none, 1=odd, 2=even
        uint8_t numbits(void) { return usb_cdc_line_coding[1] >> 16; }
        uint8_t dtr(void) { return (usb_cdc_line_rtsdtr & USB_SERIAL_DTR) ? 1 : 0; }
        uint8_t rts(void) { return (usb_cdc_line_rtsdtr & USB_SERIAL_RTS) ? 1 : 0; }
        operator bool() { return usb_configuration && (usb_cdc_line_rtsdtr & (USB_SERIAL_DTR | USB_SERIAL_RTS)); }
};

extern usb_serial_class Serial;

#endif // __cplusplus

#endif // USB_SERIAL || USB_SERIAL_HID
#endif // USBserial_h_
