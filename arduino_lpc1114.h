
/** \file
  \brief MCU pin mappings.

  Here we map the pins required by Teacup to the names known by CMSIS.
*/

/** I/O pins.

  In MBED, I/O pin handling is rather complicated. Lots of enums, lots of
  functions, spread over various files, slow execution (pin toggling about
  15 times slower than what we have here).

  Accordingly, the strategy for FastIO is derived directly from the data sheet.
  There one can see that hardware is very similar to that of AVRs: pins grouped
  in ports (here: 12 pins per port) and a number of additional registers with
  bits for additional proterties: wether input or output, wether a pullup is
  attached, and so on. See chapter 12 of the LPC111x User Manual. Other than
  AVRs, many ARMs (including the LPC1114) support bit-banding, which is
  interrupt safe and also a few clock cycles faster. See macros in pinio.h.
*/

#include "cmsis-lpc11xx.h"

#define F_CPU __SYSTEM_CLOCK

/** Pins for UART, the serial port.
*/
#define RXD             PIO1_6
#define RXD_CMSIS       PIO1_6_CMSIS
#define TXD             PIO1_7
#define TXD_CMSIS       PIO1_7_CMSIS

/**
  Offsets to the various GPIO registers. See chapter 12.3 in LPC111x User
  Manual.
*/
#define IO_MASK_OFFSET  0x0000  // Bit-masked data registers.
#define IO_DATA_OFFSET  0x3FFC  // Data register.
#define IO_DIR_OFFSET   0x8000  // Data direction register.
#define IO_IS_OFFSET    0x8004  // Interrupt sense register.
#define IO_IBE_OFFSET   0x8008  // Interrupt both edges register.
#define IO_IEV_OFFSET   0x800C  // Interrupt event register.
#define IO_IE_OFFSET    0x8010  // Interrupt mask register.
#define IO_RIS_OFFSET   0x8014  // Raw interrupt status register.
#define IO_MIS_OFFSET   0x8018  // Masked interrupt status register.
#define IO_IC_OFFSET    0x801C  // Interrupt clear register.

/**
  Masks to handle the pulling function. All IOCON registers for pins do this on
  bits [4:3]. 'Or' them with xxx_OUTPUT (see below) to get the requested
  functionality.

  Note that PIO0_4 and PIO0_5 are always open drain, no pullup possible. Mask
  bits are "reserved" on these two.
*/
#define IO_MODEMASK_INACTIVE  (0x00 << 3)
#define IO_MODEMASK_PULLDOWN  (0x01 << 3)
#define IO_MODEMASK_PULLUP    (0x02 << 3)
#define IO_MODEMASK_REPEATER  (0x03 << 3)

/**
  We define only pins available on the DIP28/TSSOP28 package here, because
  the LPC1114FN28/102 is currently the only LPC1114 known to be used in a
  RepRap controller. We also use pin names as defined in the User Manual, for
  (hopefully) least disambiguation.

  Description:

    xxx_CMSIS   Stupid MBED uses an inconsistent naming scheme in
                LPC_IOCON_TypeDef, so we have to map these names to the
                standard naming pattern.

    xxx_PIN     Pin number. Can be used to calculate the port mask and also
                the bitbanding address.

    xxx_PORT    Base address of the corresponding GPIO register. For offsets
                see above.

    xxx_OUTPUT  Bits to set/clear to set this pin as GPIO output in the IOCON
                register. See chapter 7.4 of the User Manual. FUNC as I/O pin,
                MODE inactive (no pullup or pulldown), no hysteresis, no
                analog function, no open drain.

                To achieve appropriate pulling behaviour for inputs, 'or'
                this value with one of the IO_MODEMASKs.

    xxx_TIMER   The timer a pin can be connected to, or NO_TIMER, if this
                pin can't be connected to a timer for some reason. If a pin
                can be connected to a timer, it can output PWM.

    xxx_MATCH   Number of the match register of PWM-able pins. Must be defined
                for non-PWM-able pins, too, to allow macros to expand. Value
                doesn't matter in the latter case.

    xxx_PWM     Bits to set/clear to set this pin as PWM output pin in the
                IOCON register. See chapter 7.4 of the User Manual. Must be
                defined, but value doesn't matter for non-PWM-able pins.
*/
#define NO_TIMER        ((LPC_TMR_TypeDef *)0)

// Reset pin. Don't use.
//#define PIO0_0_CMSIS    RESET_PIO0_0
//#define PIO0_0_PIN      0
//#define PIO0_0_PORT     LPC_GPIO0
//#define PIO0_0_OUTPUT   (0x01 << 0)
//#define PIO0_0_TIMER    NO_TIMER
//#define PIO0_0_MATCH    0
//#define PIO0_0_PWM      0

#define PIO0_1_CMSIS    PIO0_1
#define PIO0_1_PIN      1
#define PIO0_1_PORT     LPC_GPIO0
#define PIO0_1_OUTPUT   0x00
// Timer pin CT32B0_MAT2, but timer used for Step interrupt.
#define PIO0_1_TIMER    NO_TIMER
#define PIO0_1_MATCH    0
#define PIO0_1_PWM      0

#define PIO0_2_CMSIS    PIO0_2
#define PIO0_2_PIN      2
#define PIO0_2_PORT     LPC_GPIO0
#define PIO0_2_OUTPUT   0x00
#define PIO0_2_TIMER    NO_TIMER
#define PIO0_2_MATCH    0
#define PIO0_2_PWM      0

#define PIO0_3_CMSIS    PIO0_3
#define PIO0_3_PIN      3
#define PIO0_3_PORT     LPC_GPIO0
#define PIO0_3_OUTPUT   0x00
#define PIO0_3_TIMER    NO_TIMER
#define PIO0_3_MATCH    0
#define PIO0_3_PWM      0

#define PIO0_4_CMSIS    PIO0_4
#define PIO0_4_PIN      4
#define PIO0_4_PORT     LPC_GPIO0
#define PIO0_4_OUTPUT   (0x01 << 8)
#define PIO0_4_TIMER    NO_TIMER
#define PIO0_4_MATCH    0
#define PIO0_4_PWM      0

#define PIO0_5_CMSIS    PIO0_5
#define PIO0_5_PIN      5
#define PIO0_5_PORT     LPC_GPIO0
#define PIO0_5_OUTPUT   (0x01 << 8)
#define PIO0_5_TIMER    NO_TIMER
#define PIO0_5_MATCH    0
#define PIO0_5_PWM      0

#define PIO0_6_CMSIS    PIO0_6
#define PIO0_6_PIN      6
#define PIO0_6_PORT     LPC_GPIO0
#define PIO0_6_OUTPUT   0x00
#define PIO0_6_TIMER    NO_TIMER
#define PIO0_6_MATCH    0
#define PIO0_6_PWM      0

#define PIO0_7_CMSIS    PIO0_7
#define PIO0_7_PIN      7
#define PIO0_7_PORT     LPC_GPIO0
#define PIO0_7_OUTPUT   0x00
#define PIO0_7_TIMER    NO_TIMER
#define PIO0_7_MATCH    0
#define PIO0_7_PWM      0

#define PIO0_8_CMSIS    PIO0_8
#define PIO0_8_PIN      8
#define PIO0_8_PORT     LPC_GPIO0
#define PIO0_8_OUTPUT   0x00
#define PIO0_8_TIMER    LPC_TMR16B0
#define PIO0_8_MATCH    1
#define PIO0_8_PWM      (0x02 << 0)

#define PIO0_9_CMSIS    PIO0_9
#define PIO0_9_PIN      9
#define PIO0_9_PORT     LPC_GPIO0
#define PIO0_9_OUTPUT   0x00
#define PIO0_9_TIMER    LPC_TMR16B0
#define PIO0_9_MATCH    1
#define PIO0_9_PWM      (0x02 << 0)

#define PIO0_10_CMSIS   SWCLK_PIO0_10
#define PIO0_10_PIN     10
#define PIO0_10_PORT    LPC_GPIO0
#define PIO0_10_OUTPUT  (0x01 << 0)
#define PIO0_10_TIMER   LPC_TMR16B0
#define PIO0_10_MATCH   2
#define PIO0_10_PWM     (0x03 << 0)

#define PIO0_11_CMSIS   R_PIO0_11
#define PIO0_11_PIN     11
#define PIO0_11_PORT    LPC_GPIO0
#define PIO0_11_OUTPUT  ((0x01 << 0) | (0x01 << 7))
#define PIO0_11_ADC     0
// Timer pin CT32B0_MAT3, but timer used for Step interrupt.
#define PIO0_11_TIMER   NO_TIMER
#define PIO0_11_MATCH   0
#define PIO0_11_PWM     0

#define PIO1_0_CMSIS    R_PIO1_0
#define PIO1_0_PIN      0
#define PIO1_0_PORT     LPC_GPIO1
#define PIO1_0_OUTPUT   ((0x01 << 0) | (0x01 << 7))
#define PIO1_0_ADC      1
#define PIO1_0_TIMER    NO_TIMER
#define PIO1_0_MATCH    0
#define PIO1_0_PWM      0

#define PIO1_1_CMSIS    R_PIO1_1
#define PIO1_1_PIN      1
#define PIO1_1_PORT     LPC_GPIO1
#define PIO1_1_OUTPUT   ((0x01 << 0) | (0x01 << 7))
#define PIO1_1_ADC      2
#define PIO1_1_TIMER    LPC_TMR32B1
#define PIO1_1_MATCH    0
#define PIO1_1_PWM      (0x03 << 0)

#define PIO1_2_CMSIS    R_PIO1_2
#define PIO1_2_PIN      2
#define PIO1_2_PORT     LPC_GPIO1
#define PIO1_2_OUTPUT   ((0x01 << 0) | (0x01 << 7))
#define PIO1_2_ADC      3
#define PIO1_2_TIMER    LPC_TMR32B1
#define PIO1_2_MATCH    1
#define PIO1_2_PWM      (0x03 << 0)

#define PIO1_3_CMSIS    SWDIO_PIO1_3
#define PIO1_3_PIN      3
#define PIO1_3_PORT     LPC_GPIO1
#define PIO1_3_OUTPUT   ((0x01 << 0) | (0x01 << 7))
#define PIO1_3_ADC      4
#define PIO1_3_TIMER    LPC_TMR32B1
#define PIO1_3_MATCH    2
#define PIO1_3_PWM      (0x03 << 0)

#define PIO1_4_CMSIS    PIO1_4
#define PIO1_4_PIN      4
#define PIO1_4_PORT     LPC_GPIO1
#define PIO1_4_OUTPUT   (0x01 << 7)
#define PIO1_4_ADC      5
// Timer pin CT32B1_MAT3, but match used for PWM reset.
#define PIO1_4_TIMER    NO_TIMER
#define PIO1_4_MATCH    0
#define PIO1_4_PWM      0

#define PIO1_5_CMSIS    PIO1_5
#define PIO1_5_PIN      5
#define PIO1_5_PORT     LPC_GPIO1
#define PIO1_5_OUTPUT   0x00
#define PIO1_5_TIMER    NO_TIMER
#define PIO1_5_MATCH    0
#define PIO1_5_PWM      0

#define PIO1_6_CMSIS    PIO1_6
#define PIO1_6_PIN      6
#define PIO1_6_PORT     LPC_GPIO1
#define PIO1_6_OUTPUT   0x00
// Timer pin CT32B0_MAT0, but timer used for Step interrupt.
#define PIO1_6_TIMER    NO_TIMER
#define PIO1_6_MATCH    0
#define PIO1_6_PWM      0

#define PIO1_7_CMSIS    PIO1_7
#define PIO1_7_PIN      7
#define PIO1_7_PORT     LPC_GPIO1
#define PIO1_7_OUTPUT   0x00
// Timer pin CT32B0_MAT1, but timer used for Step interrupt.
#define PIO1_7_TIMER    NO_TIMER
#define PIO1_7_MATCH    0
#define PIO1_7_PWM      0

#define PIO1_8_CMSIS    PIO1_8
#define PIO1_8_PIN      8
#define PIO1_8_PORT     LPC_GPIO1
#define PIO1_8_OUTPUT   0x00
#define PIO1_8_TIMER    NO_TIMER
#define PIO1_8_MATCH    0
#define PIO1_8_PWM      0

#define PIO1_9_CMSIS    PIO1_9
#define PIO1_9_PIN      9
#define PIO1_9_PORT     LPC_GPIO1
#define PIO1_9_OUTPUT   0x00
#define PIO1_9_TIMER    LPC_TMR16B1
#define PIO1_9_MATCH    0
#define PIO1_9_PWM      (0x01 << 0)
