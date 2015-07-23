
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

#include "mbed-LPC11xx.h"

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

    xxx_PIN     Pin number. Can be used to calculate the port mask and also
                the bitbanding address.

    xxx_PORT    Base address of the corresponding GPIO register. For offsets
                see above.

    xxx_IOREG   Address of the corresponding IOCON register. See chapter 7 in
                LPC111x User Manual.

    xxx_OUTPUT  Bits to set/clear to set this pin as GPIO output in the IOCON
                register. See chapter 7.4 of the User Manual. FUNC as I/O pin,
                MODE inactive (no pullup or pulldown), no hysteresis, no
                analog function, no open drain.

                To achieve appropriate pulling behaviour for inputs, 'or'
                this value with one of the IO_MODEMASKs.
*/
// Reset pin. Don't use.
//#define PIO0_0_PIN      0
//#define PIO0_0_PORT     LPC_GPIO0_BASE
//#define PIO0_0_IOREG    (LPC_IOCON_BASE + 0x0C)
//#define PIO0_0_OUTPUT   (0x01 << 0)

#define PIO0_1_PIN      1
#define PIO0_1_PORT     LPC_GPIO0_BASE
#define PIO0_1_IOREG    (LPC_IOCON_BASE + 0x10)
#define PIO0_1_OUTPUT   0x00

#define PIO0_2_PIN      2
#define PIO0_2_PORT     LPC_GPIO0_BASE
#define PIO0_2_IOREG    (LPC_IOCON_BASE + 0x1c)
#define PIO0_2_OUTPUT   0x00

#define PIO0_3_PIN      3
#define PIO0_3_PORT     LPC_GPIO0_BASE
#define PIO0_3_IOREG    (LPC_IOCON_BASE + 0x2c)
#define PIO0_3_OUTPUT   0x00

#define PIO0_4_PIN      4
#define PIO0_4_PORT     LPC_GPIO0_BASE
#define PIO0_4_IOREG    (LPC_IOCON_BASE + 0x30)
#define PIO0_4_OUTPUT   (0x01 << 8)

#define PIO0_5_PIN      5
#define PIO0_5_PORT     LPC_GPIO0_BASE
#define PIO0_5_IOREG    (LPC_IOCON_BASE + 0x34)
#define PIO0_5_OUTPUT   (0x01 << 8)

#define PIO0_6_PIN      6
#define PIO0_6_PORT     LPC_GPIO0_BASE
#define PIO0_6_IOREG    (LPC_IOCON_BASE + 0x4c)
#define PIO0_6_OUTPUT   0x00

#define PIO0_7_PIN      7
#define PIO0_7_PORT     LPC_GPIO0_BASE
#define PIO0_7_IOREG    (LPC_IOCON_BASE + 0x50)
#define PIO0_7_OUTPUT   0x00

#define PIO0_8_PIN      8
#define PIO0_8_PORT     LPC_GPIO0_BASE
#define PIO0_8_IOREG    (LPC_IOCON_BASE + 0x60)
#define PIO0_8_OUTPUT   0x00

#define PIO0_9_PIN      9
#define PIO0_9_PORT     LPC_GPIO0_BASE
#define PIO0_9_IOREG    (LPC_IOCON_BASE + 0x64)
#define PIO0_9_OUTPUT   0x00

#define PIO0_10_PIN     10
#define PIO0_10_PORT    LPC_GPIO0_BASE
#define PIO0_10_IOREG   (LPC_IOCON_BASE + 0x68)
#define PIO0_10_OUTPUT  (0x01 << 0)

#define PIO0_11_PIN     11
#define PIO0_11_PORT    LPC_GPIO0_BASE
#define PIO0_11_IOREG   (LPC_IOCON_BASE + 0x74)
#define PIO0_11_OUTPUT  ((0x01 << 0) | (0x01 << 7))

#define PIO1_0_PIN      0
#define PIO1_0_PORT     LPC_GPIO1_BASE
#define PIO1_0_IOREG    (LPC_IOCON_BASE + 0x78)
#define PIO1_0_OUTPUT   ((0x01 << 0) | (0x01 << 7))

#define PIO1_1_PIN      1
#define PIO1_1_PORT     LPC_GPIO1_BASE
#define PIO1_1_IOREG    (LPC_IOCON_BASE + 0x7c)
#define PIO1_1_OUTPUT   ((0x01 << 0) | (0x01 << 7))

#define PIO1_2_PIN      2
#define PIO1_2_PORT     LPC_GPIO1_BASE
#define PIO1_2_IOREG    (LPC_IOCON_BASE + 0x80)
#define PIO1_2_OUTPUT   ((0x01 << 0) | (0x01 << 7))

#define PIO1_3_PIN      3
#define PIO1_3_PORT     LPC_GPIO1_BASE
#define PIO1_3_IOREG    (LPC_IOCON_BASE + 0x90)
#define PIO1_3_OUTPUT   ((0x01 << 0) | (0x01 << 7))

#define PIO1_4_PIN      4
#define PIO1_4_PORT     LPC_GPIO1_BASE
#define PIO1_4_IOREG    (LPC_IOCON_BASE + 0x94)
#define PIO1_4_OUTPUT   (0x01 << 7)

#define PIO1_5_PIN      5
#define PIO1_5_PORT     LPC_GPIO1_BASE
#define PIO1_5_IOREG    (LPC_IOCON_BASE + 0xa0)
#define PIO1_5_OUTPUT   0x00

#define PIO1_6_PIN      6
#define PIO1_6_PORT     LPC_GPIO1_BASE
#define PIO1_6_IOREG    (LPC_IOCON_BASE + 0xa4)
#define PIO1_6_OUTPUT   0x00

#define PIO1_7_PIN      7
#define PIO1_7_PORT     LPC_GPIO1_BASE
#define PIO1_7_IOREG    (LPC_IOCON_BASE + 0xa8)
#define PIO1_7_OUTPUT   0x00

#define PIO1_8_PIN      8
#define PIO1_8_PORT     LPC_GPIO1_BASE
#define PIO1_8_IOREG    (LPC_IOCON_BASE + 0x14)
#define PIO1_8_OUTPUT   0x00

#define PIO1_9_PIN      9
#define PIO1_9_PORT     LPC_GPIO1_BASE
#define PIO1_9_IOREG    (LPC_IOCON_BASE + 0x38)
#define PIO1_9_OUTPUT   0x00
