/** \file
 \brief I/O primitives - step, enable, direction, endstops etc
*/

#ifndef	_PINIO_H
#define	_PINIO_H

#include	"config_wrapper.h"

// Configuration tests.
#ifdef USE_INTERNAL_PULLDOWNS
  #ifdef USE_INTERNAL_PULLUPS
    #error Cant use USE_INTERNAL_PULLUPS and ..._PULLDOWNS at the same time.
  #endif
#endif

#ifndef MASK
  /// MASKING- returns \f$2^PIN\f$
  #define MASK(PIN) (1 << PIN)
#endif

/** Magic I/O routines, also known as "FastIO".

  Now you can simply SET_OUTPUT(STEP); WRITE(STEP, 1); WRITE(STEP, 0);.

  The point here is to move any pin/port mapping calculations into the
  preprocessor. This way there is no longer math at runtime neccessary, all
  instructions melt into a single one with fixed numbers.

  This makes code for setting a pin small, smaller than calling a subroutine.
  It also make code fast, on AVR a pin can be turned on and off in just two
  clock cycles.
*/
#if defined __AVR__

  #include <avr/io.h>

  /// Read a pin.
  #define _READ(IO)        (IO ## _RPORT & MASK(IO ## _PIN))
  /// Write to a pin.
  #define _WRITE(IO, v) \
    do { \
      if (v) { IO ## _WPORT |= MASK(IO ## _PIN); } \
      else { IO ## _WPORT &= ~MASK(IO ## _PIN); } \
    } while (0)

  /**
    Setting pins as input/output: other than with ARMs, function of a pin
    on AVR isn't given by a dedicated function register, but solely by the
    on-chip peripheral connected to it. With the peripheral (e.g. UART, SPI,
    ...) connected, a pin automatically serves with this function. With the
    peripheral disconnected, it automatically returns to general I/O function.
  */
  /// Set pin as input.
  #define _SET_INPUT(IO)   do { IO ## _DDR &= ~MASK(IO ## _PIN); } while (0)
  /// Set pin as output.
  #define _SET_OUTPUT(IO)  do { IO ## _DDR |=  MASK(IO ## _PIN); } while (0)

  /// Enable pullup resistor.
  #define _PULLUP_ON(IO)   _WRITE(IO, 1)
  /// No such feature on ATmegas.
  #define _PULLDOWN_ON(IO) atmegas_dont_support_pulldown_resistors()
  /// Disable pullup resistor.
  #define _PULL_OFF(IO)    _WRITE(IO, 0)

#elif defined __ARM_LPC1114__

  /**
    The LPC1114 supports bit-banding by mapping the bit mask to the address.
    See chapter 12 in the LPC111x User Manual. A read-modify-write cycle like
    on AVR costs 5 clock cycles, this implementation works with 3 clock cycles.
  */
  /// Read a pin.
  #define _READ(IO)        (IO ## _PORT->MASKED_ACCESS[MASK(IO ## _PIN)])
  /// Write to a pin.
  #define _WRITE(IO, v) \
    do { \
      IO ## _PORT->MASKED_ACCESS[MASK(IO ## _PIN)] = \
        (v) ? MASK(IO ## _PIN) : 0; \
    } while (0)

  /**
    Set pins as input/output. On ARM, each pin has its own IOCON register,
    which allows to set its function and mode. We always set here standard
    GPIO behavior. Peripherals using these pins may have to change this and
    should do so in their own context.
  */
  /// Set pin as input.
  #define _SET_INPUT(IO) \
    do { \
      LPC_IOCON->IO ## _CMSIS = (IO ## _OUTPUT | IO_MODEMASK_REPEATER); \
      IO ## _PORT->DIR &= ~MASK(IO ## _PIN); \
    } while (0)
  /// Set pin as output.
  #define _SET_OUTPUT(IO) \
    do { \
      LPC_IOCON->IO ## _CMSIS = IO ## _OUTPUT; \
      IO ## _PORT->DIR |= MASK(IO ## _PIN); \
    } while (0)

  /// Enable pullup resistor or switch from pulldown to pullup.
  #define _PULLUP_ON(IO) \
    do { \
      LPC_IOCON->IO ## _CMSIS = (IO ## _OUTPUT | IO_MODEMASK_PULLUP); \
    } while (0)
  /// Enable pulldown resistor or switch from pullup to pulldown.
  #define _PULLDOWN_ON(IO) \
    do { \
      LPC_IOCON->IO ## _CMSIS = (IO ## _OUTPUT | IO_MODEMASK_PULLDOWN); \
    } while (0)
  /// Disable pull resistor.
  #define _PULL_OFF(IO) \
    do { \
      LPC_IOCON->IO ## _CMSIS = (IO ## _OUTPUT | IO_MODEMASK_INACTIVE); \
    } while (0)

#elif defined __ARM_STM32__
  /// Read a pin.
  #define _READ(IO) 		(IO ## _PORT->IDR & MASK(IO ## _PIN))
  /// Write to a pin.
  #define _WRITE(IO, v) \
    do { \
      if (v) \
      	IO ## _PORT->BSRR = MASK(IO ## _PIN); \
      else \
      	IO ## _PORT->BSRR = MASK((IO ## _PIN + 16)); \
    } while (0)

  /** 
    possible modes are:
      0: Input (reset state)
      1: General purpose output mode
      2: Alternate function mode
      3: Analog mode
  */
  #define _SET_MODE(IO, v) \
    do { \
      IO ## _PORT->MODER &= ~(0x3 << ((IO ## _PIN) * 2)); \
      IO ## _PORT->MODER |= (v << ((IO ## _PIN) * 2)); \
    } while (0)
  #define SET_MODE(IO, v) _SET_MODE(IO, v)

  /// Enable pullup resistor.
  #define _PULLUP_ON(IO) \
    do { \
      IO ## _PORT->PUPDR &= ~(0x3 << ((IO ## _PIN) * 2)); \
      IO ## _PORT->PUPDR |= (0x1 << ((IO ## _PIN) * 2)); \
    } while (0)
  /// Enable pulldown resistor.
  #define _PULLDOWN_ON(IO) \
    do { \
      IO ## _PORT->PUPDR &= ~(0x3 << ((IO ## _PIN) * 2)); \
      IO ## _PORT->PUPDR |= (0x2 << ((IO ## _PIN) * 2)); \
    } while (0)
  /// Disable pullup resistor.
  #define _PULL_OFF(IO) \
    do { \
      IO ## _PORT->PUPDR &= ~(0x3 << ((IO ## _PIN) * 2)); \
    } while (0)

  /// Set alternate function register
  #define _SET_AFR(IO, v) \
    do { \
      uint8_t subst; \
      subst = (IO ## _PIN >= 8); \
      IO ## _PORT->AFR[subst] &= ~(0xF << ((IO ## _PIN - (subst * 8)) * 4)); \
      IO ## _PORT->AFR[subst] |= ((v) << ((IO ## _PIN - (subst * 8)) * 4)); \
    } while (0)
  #define SET_AFR(IO, v) _SET_AFR(IO, v)

  /// Set output speed register
  #define _SET_OSPEED(IO, v) \
    do { \
      IO ## _PORT->OSPEEDR &= ~(0x3 << ((IO ## _PIN) * 2)); \
      IO ## _PORT->OSPEEDR |= ~(v << ((IO ## _PIN) * 2)); \
    } while (0)
  #define SET_OSPEED(IO, v) _SET_OSPEED(IO, v)

  /** 
    Set pins as input/output. Standard is input. MODER ist 32bit. 2bits for every pin.
  */
  /** Set pin as input.
      - reset pullup
      - set input mode
  */
  #define _SET_INPUT(IO) \
    do { \
      _PULL_OFF(IO); \
      _SET_MODE(IO, 0); \
    } while (0)
  
  /** Set pin as output.
      - reset Pullup
      - reset direction mode
      - set output
      - set speed
  */
  #define _SET_OUTPUT(IO) \
    do { \
      _PULL_OFF(IO); \
      _SET_MODE(IO, 1); \
      _SET_OSPEED(IO, 3); \
    } while (0)

#elif defined SIMULATOR

  #include "simulator.h"

  bool _READ(pin_t pin);
  void _WRITE(pin_t pin, bool on);
  void _SET_OUTPUT(pin_t pin);
  void _SET_INPUT(pin_t pin);
  #define _PULLUP_ON(IO)   _WRITE(IO, 1)
  #define _PULL_OFF(IO)    _WRITE(IO, 0)

#endif /* __AVR__, __ARM_LPC1114__, __ARM_STM32__, SIMULATOR */

/**
  Why double up on these macros?
  See http://gcc.gnu.org/onlinedocs/cpp/Stringification.html
*/
/// Read a pin wrapper.
#define READ(IO)        _READ(IO)
/// Write to a pin wrapper.
#define WRITE(IO, v)    _WRITE(IO, v)

/// Set pin as input wrapper.
#define SET_INPUT(IO)   _SET_INPUT(IO)
/// Set pin as output wrapper.
#define SET_OUTPUT(IO)  _SET_OUTPUT(IO)

/// Enable pullup resistor or switch from pulldown to pullup.
#define PULLUP_ON(IO)   _PULLUP_ON(IO)
/// Enable pulldown resistor or switch from pullup to pulldown.
#define PULLDOWN_ON(IO) _PULLDOWN_ON(IO)
/// Disable pull resistor.
#define PULL_OFF(IO)    _PULL_OFF(IO)

/*
Power
*/

/// psu_timeout is set to zero when we step, and increases over time so we can
/// turn the motors off when they've been idle for a while.
/// A second function is to guarantee a minimum on time of the PSU.
/// Timeout counting is done in clock.c.
/// It is used inside and outside of interrupts, which is why it has been made volatile
extern volatile uint8_t psu_timeout;

static void power_init(void);
inline void power_init(void) {
  #ifdef PS_MOSFET_PIN
    WRITE(PS_MOSFET_PIN, 0);
    SET_OUTPUT(PS_MOSFET_PIN);
  #endif
}

void pinio_init(void);

void power_on(void);
void power_off(void);

/*
X Stepper
*/

#define	_x_step(st)						WRITE(X_STEP_PIN, st)
#define x_step()              _x_step(1)
#ifndef	X_INVERT_DIR
	#define	x_direction(dir)		WRITE(X_DIR_PIN, dir)
#else
	#define	x_direction(dir)		WRITE(X_DIR_PIN, (dir)^1)
#endif
#ifdef	X_MIN_PIN
	#ifndef X_INVERT_MIN
		#define x_min()						(READ(X_MIN_PIN)?1:0)
	#else
		#define x_min()						(READ(X_MIN_PIN)?0:1)
	#endif
#else
	#define	x_min()							(0)
#endif
#ifdef	X_MAX_PIN
	#ifndef X_INVERT_MAX
		#define x_max()						(READ(X_MAX_PIN)?1:0)
	#else
		#define x_max()						(READ(X_MAX_PIN)?0:1)
	#endif
#else
	#define	x_max()							(0)
#endif

/*
Y Stepper
*/

#define	_y_step(st)						WRITE(Y_STEP_PIN, st)
#define y_step()              _y_step(1)
#ifndef	Y_INVERT_DIR
	#define	y_direction(dir)		WRITE(Y_DIR_PIN, dir)
#else
	#define	y_direction(dir)		WRITE(Y_DIR_PIN, (dir)^1)
#endif
#ifdef	Y_MIN_PIN
	#ifndef Y_INVERT_MIN
		#define y_min()						(READ(Y_MIN_PIN)?1:0)
	#else
		#define y_min()						(READ(Y_MIN_PIN)?0:1)
	#endif
#else
	#define	y_min()							(0)
#endif
#ifdef	Y_MAX_PIN
	#ifndef Y_INVERT_MAX
		#define y_max()						(READ(Y_MAX_PIN)?1:0)
	#else
		#define y_max()						(READ(Y_MAX_PIN)?0:1)
	#endif
#else
	#define	y_max()							(0)
#endif

/*
Z Stepper
*/

#if defined Z_STEP_PIN && defined Z_DIR_PIN
	#define	_z_step(st)					WRITE(Z_STEP_PIN, st)
  #define z_step()            _z_step(1)
	#ifndef	Z_INVERT_DIR
		#define	z_direction(dir)	WRITE(Z_DIR_PIN, dir)
	#else
		#define	z_direction(dir)	WRITE(Z_DIR_PIN, (dir)^1)
	#endif
#else
	#define	_z_step(x)					do { } while (0)
	#define	z_step()						do { } while (0)
	#define	z_direction(x)			do { } while (0)
#endif
#ifdef	Z_MIN_PIN
	#ifndef Z_INVERT_MIN
		#define z_min()						(READ(Z_MIN_PIN)?1:0)
	#else
		#define z_min()						(READ(Z_MIN_PIN)?0:1)
	#endif
#else
	#define	z_min()							(0)
#endif
#ifdef	Z_MAX_PIN
	#ifndef Z_INVERT_MAX
		#define z_max()						(READ(Z_MAX_PIN)?1:0)
	#else
		#define z_max()						(READ(Z_MAX_PIN)?0:1)
	#endif
#else
	#define	z_max()							(0)
#endif

/*
Extruder
*/

#if defined E_STEP_PIN && defined E_DIR_PIN
	#define	_e_step(st)					WRITE(E_STEP_PIN, st)
  #define e_step()            _e_step(1)
	#ifndef	E_INVERT_DIR
		#define	e_direction(dir)	WRITE(E_DIR_PIN, dir)
	#else
		#define	e_direction(dir)	WRITE(E_DIR_PIN, (dir)^1)
	#endif
#else
	#define	_e_step(st)					do { } while (0)
	#define	e_step()						do { } while (0)
	#define	e_direction(dir)		do { } while (0)
#endif

/*
End Step - All Steppers
(so we don't have to delay in interrupt context)
*/

#define unstep() 							do { _x_step(0); _y_step(0); _z_step(0); _e_step(0); } while (0)

/*
Stepper Enable Pins
*/

#ifdef	STEPPER_ENABLE_PIN
	#ifdef	STEPPER_INVERT_ENABLE
		#define stepper_enable()	do { WRITE(STEPPER_ENABLE_PIN, 0); } while (0)
		#define stepper_disable()	do { WRITE(STEPPER_ENABLE_PIN, 1); } while (0)
	#else
		#define stepper_enable()	do { WRITE(STEPPER_ENABLE_PIN, 1); } while (0)
		#define stepper_disable()	do { WRITE(STEPPER_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	stepper_enable()		do { } while (0)
	#define	stepper_disable()		do { } while (0)
#endif

#ifdef	X_ENABLE_PIN
	#ifdef	X_INVERT_ENABLE
		#define	x_enable()				do { WRITE(X_ENABLE_PIN, 0); } while (0)
		#define	x_disable()				do { WRITE(X_ENABLE_PIN, 1); } while (0)
	#else
		#define	x_enable()				do { WRITE(X_ENABLE_PIN, 1); } while (0)
		#define	x_disable()				do { WRITE(X_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	x_enable()					do { } while (0)
	#define	x_disable()					do { } while (0)
#endif

#ifdef	Y_ENABLE_PIN
	#ifdef	Y_INVERT_ENABLE
		#define	y_enable()				do { WRITE(Y_ENABLE_PIN, 0); } while (0)
		#define	y_disable()				do { WRITE(Y_ENABLE_PIN, 1); } while (0)
	#else
		#define	y_enable()				do { WRITE(Y_ENABLE_PIN, 1); } while (0)
		#define	y_disable()				do { WRITE(Y_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	y_enable()					do { } while (0)
	#define	y_disable()					do { } while (0)
#endif

#ifdef	Z_ENABLE_PIN
	#ifdef	Z_INVERT_ENABLE
		#define	z_enable()				do { WRITE(Z_ENABLE_PIN, 0); } while (0)
		#define	z_disable()				do { WRITE(Z_ENABLE_PIN, 1); } while (0)
	#else
		#define	z_enable()				do { WRITE(Z_ENABLE_PIN, 1); } while (0)
		#define	z_disable()				do { WRITE(Z_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	z_enable()					do { } while (0)
	#define	z_disable()					do { } while (0)
#endif

#ifdef	E_ENABLE_PIN
	#ifdef	E_INVERT_ENABLE
		#define	e_enable()				do { WRITE(E_ENABLE_PIN, 0); } while (0)
		#define	e_disable()				do { WRITE(E_ENABLE_PIN, 1); } while (0)
	#else
		#define	e_enable()				do { WRITE(E_ENABLE_PIN, 1); } while (0)
		#define	e_disable()				do { WRITE(E_ENABLE_PIN, 0); } while (0)
	#endif
#else
	#define	e_enable()					do { } while (0)
	#define	e_disable()					do { } while (0)
#endif

/*
Internal pullup resistors for endstops
*/
static void endstops_on(void) __attribute__ ((always_inline));
inline void endstops_on(void) {
	#ifdef USE_INTERNAL_PULLUPS
		#ifdef X_MIN_PIN
      PULLUP_ON(X_MIN_PIN);
		#endif
		#ifdef X_MAX_PIN
      PULLUP_ON(X_MAX_PIN);
		#endif
		#ifdef Y_MIN_PIN
      PULLUP_ON(Y_MIN_PIN);
		#endif
		#ifdef Y_MAX_PIN
      PULLUP_ON(Y_MAX_PIN);
		#endif
		#ifdef Z_MIN_PIN
      PULLUP_ON(Z_MIN_PIN);
		#endif
		#ifdef Z_MAX_PIN
      PULLUP_ON(Z_MAX_PIN);
		#endif
	#endif

  #ifdef USE_INTERNAL_PULLDOWNS
    #ifdef X_MIN_PIN
      PULLDOWN_ON(X_MIN_PIN);
    #endif
    #ifdef X_MAX_PIN
      PULLDOWN_ON(X_MAX_PIN);
    #endif
    #ifdef Y_MIN_PIN
      PULLDOWN_ON(Y_MIN_PIN);
    #endif
    #ifdef Y_MAX_PIN
      PULLDOWN_ON(Y_MAX_PIN);
    #endif
    #ifdef Z_MIN_PIN
      PULLDOWN_ON(Z_MIN_PIN);
    #endif
    #ifdef Z_MAX_PIN
      PULLDOWN_ON(Z_MAX_PIN);
    #endif
  #endif
}

static void endstops_off(void) __attribute__ ((always_inline));
inline void endstops_off(void) {
  #if (defined USE_INTERNAL_PULLUPS) || (defined USE_INTERNAL_PULLDOWNS)
		#ifdef X_MIN_PIN
      PULL_OFF(X_MIN_PIN);
		#endif
		#ifdef X_MAX_PIN
      PULL_OFF(X_MAX_PIN);
		#endif
		#ifdef Y_MIN_PIN
      PULL_OFF(Y_MIN_PIN);
		#endif
		#ifdef Y_MAX_PIN
      PULL_OFF(Y_MAX_PIN);
		#endif
		#ifdef Z_MIN_PIN
      PULL_OFF(Z_MIN_PIN);
		#endif
		#ifdef Z_MAX_PIN
      PULL_OFF(Z_MAX_PIN);
		#endif
	#endif
}


#endif	/* _PINIO_H */
