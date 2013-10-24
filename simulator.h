#if !defined _SIMULATOR_H && defined SIMULATOR
#define _SIMULATOR_H

#include <stdint.h>
#include <stdbool.h>

#define PROGMEM
#define PGM_P const char *
#define PSTR(x) (x)
#define pgm_read_byte(x) (*((uint8_t *)(x)))
#define pgm_read_word(x) (*((uint16_t *)(x)))

#define MASK(PIN) (1 << PIN)
#define ACD     7
#define OCIE1A  1

// TODO: Implement simulated EEMEM persistence
#define EEMEM
#define eeprom_read_dword(ptr32) (*(ptr32))
#define eeprom_read_word(ptr16) (*(ptr16))
#define eeprom_write_dword(ptr32, i32) (*(ptr32)=i32)
#define eeprom_write_word(ptr16, i16) (*(ptr16)=i16)

#undef X_STEP_PIN
#undef X_DIR_PIN
#undef X_MIN_PIN
#undef X_ENABLE_PIN
#undef Y_STEP_PIN
#undef Y_DIR_PIN
#undef Y_MIN_PIN
#undef Y_ENABLE_PIN
#undef Z_STEP_PIN
#undef Z_DIR_PIN
#undef Z_MIN_PIN
#undef Z_ENABLE_PIN
#undef E_STEP_PIN
#undef E_DIR_PIN
#undef E_ENABLE_PIN
#undef STEPPER_ENABLE_PIN

#undef PS_MOSFET_PIN
#undef PS_ON_PIN
#undef RX_ENABLE_PIN
#undef TX_ENABLE_PIN
#undef X_MAX_PIN
#undef Y_MAX_PIN
#undef Z_MAX_PIN

typedef enum {
        // Define pins used
        X_STEP_PIN,
        X_DIR_PIN,
        X_MIN_PIN,
        X_ENABLE_PIN,
        Y_STEP_PIN,
        Y_DIR_PIN,
        Y_MIN_PIN,
        Y_ENABLE_PIN,
        Z_STEP_PIN,
        Z_DIR_PIN,
        Z_MIN_PIN,
        Z_ENABLE_PIN,
        E_STEP_PIN,
        E_DIR_PIN,
        E_ENABLE_PIN,

        STEPPER_ENABLE_PIN,

        SCK,
        MOSI,
        MISO,
        SS,

/*
 * Not used in the simulator.  Add them to this list to enable them if needed.
  PS_MOSFET_PIN,
  PS_ON_PIN,
  RX_ENABLE_PIN,
  TX_ENABLE_PIN,
  X_MAX_PIN,
  Y_MAX_PIN,
  Z_MAX_PIN,
  */
    PIN_NB     /* End of PINS marker; Put all new pins before this one */
} pin_t;

// AVR stand-ins
typedef enum {
    WGM00 = 0,
    WGM01,
    WGM20,
    WGM21,
    CS00 = 0,
    CS02,
    CS20,
    CS21,
    CS22,
} masks_t;

#undef TEMP_PIN_CHANNEL
#define TEMP_PIN_CHANNEL 0

extern uint8_t ACSR;
extern uint8_t TIMSK1;
extern volatile bool sim_interrupts;

void WRITE(pin_t pin, bool on);
void SET_OUTPUT(pin_t pin);
void SET_INPUT(pin_t pin);

#define READ(n) 0

void sei(void);

#ifdef USE_WATCHDOG
#define wd_init()
#define wd_reset()
#endif

void sim_info(const char fmt[], ...);
void sim_error(const char msg[]);
void sim_assert(bool cond, const char msg[]);
inline void cli(void);
inline void cli() { }

#endif /* _SIMULATOR_H */
