/*
	CONTENTS
	
	1. Mechanical/Hardware
	2. Acceleration settings
	3. Pinouts
	4. Temperature sensors
	5. Heaters
	6. Communication options
	7. Miscellaneous
	8. Appendix A - PWMable pins and mappings
*/

#error this config is not tested, and may be incorrect! please post in forum or via git any corrections

/***************************************************************************\
*                                                                           *
* 1. MECHANICAL/HARDWARE                                                    *
*                                                                           *
\***************************************************************************/

/*
	Set your microcontroller type in Makefile! atmega168/atmega328p/atmega644p/atmega1280

	If you want to port this to a new chip, start off with arduino.h and see how you go.
*/
#if ! ( defined (__AVR_ATmega644P__) || defined (__AVR_ATmega644PA__) )
	#error GEN3 has a 644P/644PA! set your cpu type in Makefile!
#endif

/*
	CPU clock rate
*/
#ifndef	F_CPU
	#define	F_CPU	16000000L
#endif

/*
	This is the motherboard, as opposed to the extruder. See extruder/ directory for GEN3 extruder firmware
*/
#define	HOST

/*
	Values reflecting the gearing of your machine.
		All numbers are fixed point integers, so no more than 3 digits to the right of the decimal point, please :-)
*/

// calculate these values appropriate for your machine
// for threaded rods, this is (steps motor per turn) / (pitch of the thread)
// for belts, this is (steps per motor turn) / (number of gear teeth) / (belt module)
// half-stepping doubles the number, quarter stepping requires * 4, etc.
#define	STEPS_PER_MM_X				320.000
#define	STEPS_PER_MM_Y				320.000
#define	STEPS_PER_MM_Z				200.000

// http://blog.arcol.hu/?p=157 may help with this next one
#define	STEPS_PER_MM_E				320.000


/*
	Values depending on the capabilities of your stepper motors and other mechanics.
		All numbers are integers, no decimals allowed.

		Units are mm/min
*/

// used for G0 rapid moves and as a cap for all other feedrates
#define	MAXIMUM_FEEDRATE_X		200
#define	MAXIMUM_FEEDRATE_Y		200
#define	MAXIMUM_FEEDRATE_Z		100
#define	MAXIMUM_FEEDRATE_E		200

// used when searching endstops and as default feedrate
#define	SEARCH_FEEDRATE_X			50
#define	SEARCH_FEEDRATE_Y			50
#define	SEARCH_FEEDRATE_Z			50
#define	SEARCH_FEEDRATE_E			50

// this is how many steps to suck back the filament by when we stop. set to zero to disable
#define	E_STARTSTOP_STEPS			20



/***************************************************************************\
*                                                                           *
* 2. ACCELERATION                                                           *
*                                                                           *
* IMPORTANT: choose only one! These algorithms choose when to step, trying  *
*            to use more than one will have undefined and probably          *
*            disastrous results!                                            *
*                                                                           *
\***************************************************************************/


/*
	acceleration, reprap style.
		Each movement starts at the speed of the previous command and accelerates or decelerates linearly to reach target speed at the end of the movement.
*/
// #define ACCELERATION_REPRAP


/*
	acceleration and deceleration ramping.
		Each movement starts at (almost) no speed, linearly accelerates to target speed and decelerates just in time to smoothly stop at the target. alternative to ACCELERATION_REPRAP
*/
#define ACCELERATION_RAMPING

// how fast to accelerate when using ACCELERATION_RAMPING
// smaller values give quicker acceleration
// valid range = 1 to 8,000,000; 500,000 is a good starting point
#define ACCELERATION_STEEPNESS	500000


/*
	temporal step algorithm
		This algorithm causes the timer to fire when any axis needs to step, instead of synchronising to the axis with the most steps ala bresenham.

		This algorithm is not a type of acceleration, and I haven't worked out how to integrate acceleration with it.
		However it does control step timing, so acceleration algorithms seemed appropriate

		The Bresenham algorithm is great for drawing lines, but not so good for steppers - In the case where X steps 3 times to Y's two, Y experiences massive jitter as it steps in sync with X every 2 out of 3 X steps. This is a worst-case, but the problem exists for most non-45/90 degree moves. At higher speeds, the jitter /will/ cause position loss and unnecessary vibration.
		This algorithm instead calculates when a step occurs on any axis, and sets the timer to that value.

		// TODO: figure out how to add acceleration to this algorithm
*/
// #define ACCELERATION_TEMPORAL



/***************************************************************************\
*                                                                           *
* 3. PINOUTS                                                                *
*                                                                           *
\***************************************************************************/

/*
	Machine Pin Definitions
	- make sure to avoid duplicate usage of a pin
	- comment out pins not in use, as this drops the corresponding code and makes operations faster
*/

#include	"arduino.h"

/*
	this is the ramps motherboard pinout
*/
#error RAMPS pinout needs to be entered! The values below are almost definitely wrong. Please post correct pinout in forum or via git
#define TX_ENABLE_PIN					DIO12
#define	RX_ENABLE_PIN					DIO13

#define	X_STEP_PIN						DIO15
#define	X_DIR_PIN							DIO18
#define	X_MIN_PIN							DIO20
#define	X_MAX_PIN							DIO21
#define	X_ENABLE_PIN					DIO19

#define	Y_STEP_PIN						DIO23
#define	Y_DIR_PIN							DIO22
#define	Y_MIN_PIN							AIO6
#define	Y_MAX_PIN							AIO5
#define	Y_ENABLE_PIN					DIO7

#define	Z_STEP_PIN						AIO4
#define	Z_DIR_PIN							AIO3
#define	Z_MIN_PIN							AIO1
#define	Z_MAX_PIN							AIO0
#define	Z_ENABLE_PIN					AIO2

#define	E_STEP_PIN						DIO16
#define	E_DIR_PIN							DIO17

#define	SD_CARD_DETECT				DIO2
#define	SD_WRITE_PROTECT			DIO3



/***************************************************************************\
*                                                                           *
* 4. TEMPERATURE SENSORS                                                    *
*                                                                           *
\***************************************************************************/

/*
	TEMP_HYSTERESIS: actual temperature must be target +/- hysteresis before target temperature can be achieved.
	  NOTE: format is 30.2 fixed point, so value of 20 actually means +/- 5 degrees

	TEMP_RESIDENCY_TIME: actual temperature must be close to target for this long before target is achieved

	temperature is "achieved" for purposes of M109 and friends when actual temperature is within [hysteresis] of target for [residency] seconds
*/
#define	TEMP_HYSTERESIS				20
#define	TEMP_RESIDENCY_TIME		60

// which temperature sensors are you using? (intercom is the gen3-style separate extruder board)
// #define	TEMP_MAX6675
#define	TEMP_THERMISTOR
// #define	TEMP_AD595
// #define	TEMP_PT100
// #define	TEMP_INTERCOM

// ANALOG_MASK is a bitmask of all analog channels used- bitwise-or them all together
#define	ANALOG_MASK				MASK(AIO0_PIN) | MASK(AIO1_PIN)

/***************************************************************************\
*                                                                           *
* Define your temperature sensors here                                      *
*                                                                           *
* for GEN3 set temp_type to TT_INTERCOM and temp_pin to 0                   *
*                                                                           *
* Types are same as TEMP_ list above- TT_MAX6675, TT_THERMISTOR, TT_AD595,  *
*   TT_PT100, TT_INTERCOM. See list in temp.c.                              *
*                                                                           *
\***************************************************************************/

#ifndef DEFINE_TEMP_SENSOR
	#define DEFINE_TEMP_SENSOR(...)
#endif

//                 name       type          pin
DEFINE_TEMP_SENSOR(extruder,	TT_THERMISTOR,	AIO0_PIN)
DEFINE_TEMP_SENSOR(bed,				TT_THERMISTOR,	AIO1_PIN)


/***************************************************************************\
*                                                                           *
* 5. HEATERS                                                                *
*                                                                           *
\***************************************************************************/

// check if heater responds to changes in target temperature, disable and spit errors if not
// largely untested, please comment in forum if this works, or doesn't work for you!
// #define	HEATER_SANITY_CHECK

/***************************************************************************\
*                                                                           *
* Define your heaters here                                                  *
*                                                                           *
* If your heater isn't on a PWM-able pin, set heater_pwm to zero and we'll  *
*   use bang-bang output. Note that PID will still be used                  *
*                                                                           *
* See Appendix 8 at the end of this file for PWMable pin mappings           *
*                                                                           *
* If a heater isn't attached to a temperature sensor above, it can still be *
*   controlled by host but otherwise is ignored by firmware                 *
*                                                                           *
* To attach a heater to a temp sensor above, simply use exactly the same    *
*   name - copy+paste is your friend                                        *
*                                                                           *
* Some common names are 'extruder', 'bed', 'fan', 'motor'                   *
*                                                                           *
\***************************************************************************/

#ifndef DEFINE_HEATER
	#define DEFINE_HEATER(...)
#endif

//               name      port   pin    pwm
DEFINE_HEATER(extruder,	PORTD, PINB3, OCR0A)
DEFINE_HEATER(bed,			PORTD, PINB4, OCR0B)
// DEFINE_HEATER(fan,			PORTD, PINB4, OCR0B)
// DEFINE_HEATER(chamber,	PORTD, PIND7, OCR2A)
// DEFINE_HEATER(motor,		PORTD, PIND6, OCR2B)

// and now because the c preprocessor isn't as smart as it could be,
// uncomment the ones you've listed above and comment the rest.
// NOTE: these are used to enable various capability-specific chunks of code, you do NOT need to create new entries unless you are adding new capabilities elsewhere in the code!
// so if you list a bed above, uncomment HEATER_BED, but if you list a chamber you do NOT need to create HEATED_CHAMBER
// I have searched high and low for a way to make the preprocessor do this for us, but so far I have not found a way.

#define	HEATER_EXTRUDER HEATER_extruder
#define HEATER_BED HEATER_bed
// #define HEATER_FAN HEATER_fan

/***************************************************************************\
*                                                                           *
* 6. COMMUNICATION OPTIONS                                                  *
*                                                                           *
\***************************************************************************/

/*
	RepRap Host changes it's communications protocol from time to time and intentionally avoids backwards compatibility. Set this to the date the source code of your Host was fetched from RepRap's repository, which is likely also the build date.
	See the discussion on the reprap-dev mailing list from 11 Oct. 2010.

	Undefine it for best human readability, set it to an old date for compatibility with hosts before August 2010
*/
// #define REPRAP_HOST_COMPATIBILITY 19750101
#define REPRAP_HOST_COMPATIBILITY 20100806
// #define REPRAP_HOST_COMPATIBILITY <date of next RepRap Host compatibility break>

/*
	Xon/Xoff flow control.
		Redundant when using RepRap Host for sending GCode, but mandatory when sending GCode files with a plain terminal emulator, like GtkTerm (Linux), CoolTerm (Mac) or HyperTerminal (Windows).
		Can also be set in Makefile
*/
// #define	XONXOFF



/***************************************************************************\
*                                                                           *
* 7. MISCELLANEOUS OPTIONS                                                  *
*                                                                           *
\***************************************************************************/

/*
	DEBUG
		enables /heaps/ of extra output, and some extra M-codes.
		WARNING: this WILL break most host-side talkers that expect particular responses from firmware such as reprap host and replicatorG
		use with serial terminal or other suitable talker only.
*/
// #define	DEBUG

/*
	BANG_BANG
		drops PID loop from heater control, reduces code size significantly (1300 bytes!)
		may allow DEBUG on '168
	BANG_BANG_ON
		PWM value for 'on'
	BANG_BANG_OFF
		PWM value for 'off'
*/
// #define	BANG_BANG
// #define	BANG_BANG_ON	200
// #define	BANG_BANG_OFF	45

/*
	move buffer size, in number of moves
		note that each move takes a fair chunk of ram (69 bytes as of this writing) so don't make the buffer too big - a bigger serial readbuffer may help more than increasing this unless your gcodes are more than 70 characters long on average.
		however, a larger movebuffer will probably help with lots of short consecutive moves, as each move takes a bunch of math (hence time) to set up so a longer buffer allows more of the math to be done during preceding longer moves
*/
#define	MOVEBUFFER_SIZE	8

/*
	DC extruder
		If you have a DC motor extruder, configure it as a "heater" above and define this value as the index or name
*/
// #define	DC_EXTRUDER HEATER_motor
// #define	DC_EXTRUDER_PWM	180

/*
	FiveD on Arduino implements a watchdog, which has to be reset every 250ms or it will reboot the controller. As rebooting (and letting the GCode sending application trying to continue the build with a then different Home point) is probably even worse than just hanging, and there is no better restore code in place, this is disabled for now.
*/
// #define USE_WATCHDOG

/*
	analog subsystem stuff
	REFERENCE - which analog reference to use. see analog.h for choices
*/
#define	REFERENCE			REFERENCE_AVCC

/*
	this option makes the step interrupt interruptible (nested).
	this should help immensely with dropped serial characters, but may also make debugging infuriating due to the complexities arising from nested interrupts
*/
#define		STEP_INTERRUPT_INTERRUPTIBLE	1

/*
	how often we overflow and update our clock; with F_CPU=16MHz, max is < 4.096ms (TICK_TIME = 65535)
*/
#define		TICK_TIME			2 MS
#define		TICK_TIME_MS	(TICK_TIME / (F_CPU / 1000))

/*
	temperature history count. This is how many temperature readings to keep in order to calculate derivative in PID loop
	higher values make PID derivative term more stable at the expense of reaction time
*/
#define	TH_COUNT					8

// this is the scaling of internally stored PID values. 1024L is a good value
#define	PID_SCALE						1024L



/***************************************************************************\
*                                                                           *
* 8. APPENDIX A - PWMABLE PINS AND MAPPINGS                                 *
*                                                                           *
*                                                                           *
* list of PWM-able pins and corresponding timers                            *
* timer1 is used for step timing so don't use OC1A/OC1B                     *
* they are omitted from this listing for that reason                        *
*                                                                           *
* For the atmega168/328, timer/pin mappings are as follows                  *
*                                                                           *
* OCR0A - PD6  - DIO6                                                       *
* OCR0B - PD5  - DIO5                                                       *
* OCR2A - PB3  - DIO11                                                      *
* OCR2B - PD3  - DIO3                                                       *
*                                                                           *
* For the atmega644, timer/pin mappings are as follows                      *
*                                                                           *
* OCR0A - PB3  - DIO3                                                       *
* OCR0B - PB4  - DIO4                                                       *
* OCR2A - PD7  - DIO15                                                      *
* OCR2B - PD6  - DIO14                                                      *
*                                                                           *
* For the atmega1280, timer/pin mappings are as follows                     *
*                                                                           *
* OCR0A  - PB7 - DIO13                                                      *
* OCR0B  - PG5 - DIO4                                                       *
* OCR2A  - PB4 - DIO10                                                      *
* OCR2B  - PH6 - DIO9                                                       *
* OCR3AL - PE3 - DIO5                                                       *
* OCR3BL - PE4 - DIO2                                                       *
* OCR3CL - PE5 - DIO3                                                       *
* OCR4AL - PH3 - DIO6                                                       *
* OCR4BL - PH4 - DIO7                                                       *
* OCR4CL - PH5 - DIO8                                                       *
* OCR5AL - PL3 - DIO46                                                      *
* OCR5BL - PL4 - DIO45                                                      *
* OCR5CL - PL5 - DIO44                                                      *
*                                                                           *
\***************************************************************************/
