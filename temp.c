#include	"temp.h"

/** \file
	\brief Manage temperature sensors

  \note ALL temperatures are stored as 14.2 fixed point in Teacup, so we have
  a range of 0 - 16383.75 and a precision of 0.25 degree Celsius.
*/

#include	<stdlib.h>
#warning remove when entirely integer
#include <math.h>
#ifndef SIMULATOR
#include	<avr/eeprom.h>
#include	<avr/pgmspace.h>
#endif
#include "simulator.h"

#include	"arduino.h"
#include	"debug.h"
#ifndef	EXTRUDER
	#include	"sersendf.h"
#endif
#include	"heater.h"
#ifdef	TEMP_INTERCOM
	#include	"intercom.h"
#endif

#ifdef	TEMP_MAX6675
#endif

#ifdef	TEMP_THERMISTOR
#include	"analog.h"
#endif

#ifdef	TEMP_AD595
#include	"analog.h"
#endif

typedef enum {
	PRESENT,
	TCOPEN
} temp_flags_enum;

/// holds metadata for each temperature sensor
typedef struct {
	temp_type_t temp_type; ///< type of sensor
	uint8_t     temp_pin;  ///< pin that sensor is on
	heater_t    heater;    ///< associated heater if any
  uint16_t    vadc;      ///< ADC reference voltage
  double      r0;        ///< Thermistor's reference resistance
  double      t0;        ///< Thermistor's reference temperature
  uint32_t    r2;        ///< Thermistor's compare resistor
  uint32_t    beta;      ///< Thermistor's calibration coefficient
} temp_sensor_definition_t;

#undef DEFINE_TEMP_SENSOR
/// help build list of sensors from entries in config.h
#ifndef SIMULATOR
#define DEFINE_TEMP_SENSOR(name, type, pin, vadc, r0, t0, r2, beta) { \
  (type), (pin ## _ADC), (HEATER_ ## name), (vadc) * 1024, (r0), \
  (t0) + 273.15, (r2), (beta) },
#else
#define DEFINE_TEMP_SENSOR(name, type, pin, vadc, r0, t0, r2, beta) { \
  (type), (TEMP_SENSOR_ ## name), (HEATER_ ## name), (vadc) },
#endif
static const temp_sensor_definition_t temp_sensors[NUM_TEMP_SENSORS] =
{
	#include	"config_wrapper.h"
};
#undef DEFINE_TEMP_SENSOR

/// this struct holds the runtime sensor data- read temperatures, targets, etc
struct {
	temp_flags_enum		temp_flags;     ///< flags

	uint16_t					last_read_temp; ///< last received reading
	uint16_t					target_temp;		///< manipulate attached heater to attempt to achieve this value

	uint16_t					temp_residency; ///< how long have we been close to target temperature in temp ticks?

	uint16_t					next_read_time; ///< how long until we can read this sensor again?
} temp_sensors_runtime[NUM_TEMP_SENSORS];

/// set up temp sensors. Currently only the 'intercom' sensor needs initialisation.
void temp_init() {
	temp_sensor_t i;
	for (i = 0; i < NUM_TEMP_SENSORS; i++) {
		switch(temp_sensors[i].temp_type) {
		#ifdef	TEMP_MAX6675
			// initialised when read
/*			case TT_MAX6675:
				break;*/
		#endif

		#ifdef	TEMP_THERMISTOR
			// handled by analog_init()
/*			case TT_THERMISTOR:
				break;*/
		#endif

		#ifdef	TEMP_AD595
			// handled by analog_init()
/*			case TT_AD595:
				break;*/
		#endif

		#ifdef	TEMP_INTERCOM
			case TT_INTERCOM:
				intercom_init();
				send_temperature(0, 0);
				break;
		#endif

			default: /* prevent compiler warning */
				break;
		}
	}
}

/**
  Natural logarithm (base e). Algorithm used in the HP-35 pocket calculator.

  See http://www.jacques-laporte.org/TheSecretOfTheAlgorithms.htm
  and http://www.jacques-laporte.org/Logarithm_1.htm

  Unused in Teacup, because floating point math. Kept for reference, because
  this algorithm is so brilliant and because the above references show no C
  implementation.
*/
double hp_35_log(double x) {
  double y, t;

  if (x == 0.)
    return 0.;

  /**
    The basic idea is quite simple.

    First step is to choose a convenient target value. As the HP-35 prefered
    BCD (binary coded decimals) over pure binary numbers, 10 is a natural
    choice. Then we have two things:

    1. Valid range is 0 < x < target, or 0 < x < 10.
    2. We start with the logarithm of this target:
  */
  y = 2.302585092994012; // ln(10)

  /**
    Normalize. Our argument could be much bigger than our target. To change
    that, we divide by the target and to compensate, we add the logarithm
    of the target. Just as the math textbooks say:

      ln(x * k) = ln(x) + ln(k)  <==>
      ln(x * 10) = ln(x) + ln(10)

    As we don't know the size of our number, yet, we have to try in reverse
    order.
  */
  while (x >= 10.) {
    x /= 10.;
    y += 2.302585092994012; // ln(10)
  }

  /**
    Here comes the brilliant part. Basically it's the opposite of what we did
    for normalisation. We try to multiply with small, well choosen numbers to
    fill the gap between our initial result ( ln(10) ) an the actually wanted
    value ( ln(x < 10) ). Then apply the same mathematical rule:

      ln(x / k) = ln(x) - ln(k) ... always as often as possible.

    That's it! A number of tries later, the gap between argument value and
    target value becomes very small and along the way, the gap between the
    target logarithm and actually searched logarithm becomes very small, too.

    To trade some precision for speed, shorten the following list of while()
    loops.
  */
  while (t = x * 2., t < 10.) {
    y -= 0.693147180559945; // ln(2)
    x = t;
  }
  while (t = x * 1.1, t < 10.) {
    y -= 0.095310179804325; // ln(1.1)
    x = t;
  }
  while (t = x * 1.01, t < 10.) {
    y -= 0.009950330853168; // ln(1.01)
    x = t;
  }
  while (t = x * 1.001, t < 10.) {
    y -= 0.000999500333084; // ln(1.001)
    x = t;
  }
  while (t = x * 1.0001, t < 10.) {
    y -= 0.000099995000333; // ln(1.0001)
    x = t;
  }
  while (t = x * 1.00001, t < 10.) {
    y -= 0.000009999950000; // ln(1.00001)
    x = t;
  }
  while (t = x * 1.000001, t < 10.) {
    y -= 0.000000999999500; // ln(1.000001)
    x = t;
  }

  return y;
}

/*!
  Natural logarithm (base e). Same as hp_35_log(), but using integers and
  optimized for binary numbers.

  \param Number in 32.0 fixed point.
  \return Natural logarithm of that number in 8.24 fixed point.

  Costs 741 ... 1795 clock cycles, 1654 on average when used in
  temp_sensor_tick() and is as such about 1000 cycles faster than avr-libc's
  log(). Measurements were done by running temp_sensor_tick() once for each of
  0 ... 1024.
*/
#define LN_PRECISION 10
// Storing this in PROGMEM saves 40 bytes RAM, but costs 120 bytes flash and
// 500 clock cycles per int_log() for the reading efforts.
static uint32_t ln[LN_PRECISION] = {
  11629079, // ln(2) * 2^24
   6802576, // ln(1 1/2) * 2^24
   3743728, // ln(1 1/4) * 2^24
   1976071, // ln(1 1/8) * 2^24
   1017112, // ln(1 1/16) * 2^24
    516262, // ln(1 1/32) * 2^24
    260117, // ln(1 1/64) * 2^24
    130562, // ln(1 1/128) * 2^24
     65408, // ln(1 1/256) * 2^24
     32736, // ln(1 1/512) * 2^24
// 10 refinements in the multiplication list are entirely sufficient for our
// current use, we currently throw away the least significant 14 bits anyways.
#if LN_PRECISION > 10
     16376, // ln(1 1/1024) * 2^24
      8190, // ln(1 1/2048) * 2^24
      4096, // ln(1 1/4096) * 2^24
      2048,
      1023,
       512,
       256,
       128,
        32,
        16,
         8,
         4,
         2,
         1
#endif
};

uint32_t teacup_log(uint32_t x) {
  uint32_t y, t; // 8.24 fixed point
  uint8_t dec;

  if (x == 0)
    return 0;

  // Target = 2.
  y = ln[0]; // ln(2) * 2^24

  // Normalize. Like find the most significant bit, then adjust result and bits.
  // Costs 35..375 clock cycles, 206 on average.
  t = 0x80000000;
  for (dec = 31; dec; dec--) {
    if (x & t)
      break;
    t >>= 1;
  }
  // Costs 12..110 clock cycles, 41 on average.
  x = x << (24 - dec);
  // Costs 56 clock cycles.
  y += ln[0] * dec;

  // Multiplication list.
  // Costs 465..543 clock cycles, 508 on average.
  for (dec = 1; dec <= LN_PRECISION - 2; dec++) {
    t = x + (x >> dec);
    if (t < (2UL << 24)) {
      y -= ln[dec];
      x = t;
    }
  }

  // Adjustment. Costs just 2 clock cycles :-)
  y -= ((2UL << 24) - ln[LN_PRECISION - 1]) / (2UL << 24);

  return y;
}

/// called every 10ms from clock.c - check all temp sensors that are ready for checking
void temp_sensor_tick(uint8_t sensor, uint16_t tempvalue) {
	temp_sensor_t i = sensor;
//	for (; i < NUM_TEMP_SENSORS; i++) {
//		if (temp_sensors_runtime[i].next_read_time) {
//			temp_sensors_runtime[i].next_read_time--;
//		}
//		else {
		{
			uint16_t	temp = tempvalue;
			//time to deal with this temp sensor
			switch(temp_sensors[i].temp_type) {
				#ifdef	TEMP_MAX6675
				case TT_MAX6675:
					#ifdef	PRR
						PRR &= ~MASK(PRSPI);
					#elif defined PRR0
						PRR0 &= ~MASK(PRSPI);
					#endif

					SPCR = MASK(MSTR) | MASK(SPE) | MASK(SPR0);

					// enable TT_MAX6675
					WRITE(SS, 0);

					// No delay required, see
					// https://github.com/triffid/Teacup_Firmware/issues/22

					// read MSB
					SPDR = 0;
					for (;(SPSR & MASK(SPIF)) == 0;);
					temp = SPDR;
					temp <<= 8;

					// read LSB
					SPDR = 0;
					for (;(SPSR & MASK(SPIF)) == 0;);
					temp |= SPDR;

					// disable TT_MAX6675
					WRITE(SS, 1);

					temp_sensors_runtime[i].temp_flags = 0;
					if ((temp & 0x8002) == 0) {
						// got "device id"
						temp_sensors_runtime[i].temp_flags |= PRESENT;
						if (temp & 4) {
							// thermocouple open
							temp_sensors_runtime[i].temp_flags |= TCOPEN;
						}
						else {
							temp = temp >> 3;
						}
					}

					// this number depends on how frequently temp_sensor_tick is called. the MAX6675 can give a reading every 0.22s, so set this to about 250ms
					temp_sensors_runtime[i].next_read_time = 25;

					break;
				#endif	/* TEMP_MAX6675	*/

				#ifdef	TEMP_THERMISTOR
          case TT_THERMISTOR: {
            /**
              Courtesy of Nophead in his Hydraraptor blog:
              http://hydraraptor.blogspot.de/2007/10/measuring-temperature-easy-way.html

              Let's do the same with r1 = 0, which is true for all RepRap
              electronics I'm aware of.
            */
            // Voltages in volts * 1024.
            uint16_t v;
            uint32_t r, k;

//            temp = analog_read(i);

            // k = r0 * exp(-beta / t0); // around 0.1
            // Instead of a divide, multiply with the inverse.
            // Multiply with 32 for higher accuracy.
            k = (uint32_t)((double)32. / (temp_sensors[i].r0 *
                                          exp(-(double)temp_sensors[i].beta /
                                          temp_sensors[i].t0)) + .5);
            // v = temp * vadc / 1024.;  // min. 0, max. 5000
            v = temp * (temp_sensors[i].vadc / 1024);
#warning ungeschickt, da sehr ungenau f+r vadc = 3.3.
            // r = r2 * v / (vadc - v);  // min. 0, max. 50'000'000
            r = (temp_sensors[i].r2 * v) / (temp_sensors[i].vadc - v);

            /**
              For better accuracy:
              - Subtract ln(32) in 8.24 fixed point = 58145400 to compensate
                multiplication by 32 above.
              - Do multiplication by 4 and 1024 in the numerator already.
            */
            // temp = (uint16_t)(((beta / log(r / k)) - 273.15) * 4.0);
            temp = (uint16_t)(((temp_sensors[i].beta << 2 << 10) /
                               ((teacup_log(r * k) - 58145400) >> 14)) - 1093);

            temp_sensors_runtime[i].next_read_time = 0;
          }
					break;
				#endif	/* TEMP_THERMISTOR */

				#ifdef	TEMP_AD595
				case TT_AD595:
					temp = analog_read(i);

					// convert
					// >>8 instead of >>10 because internal temp is stored as 14.2 fixed point
					temp = (temp * 500L) >> 8;

					temp_sensors_runtime[i].next_read_time = 0;

					break;
				#endif	/* TEMP_AD595 */

				#ifdef	TEMP_PT100
				case TT_PT100:
					#warning TODO: PT100 code
					break;
				#endif	/* TEMP_PT100 */

				#ifdef	TEMP_INTERCOM
				case TT_INTERCOM:
					temp = read_temperature(temp_sensors[i].temp_pin);

					temp_sensors_runtime[i].next_read_time = 25;

					break;
				#endif	/* TEMP_INTERCOM */

				#ifdef	TEMP_DUMMY
				case TT_DUMMY:
					temp = temp_sensors_runtime[i].last_read_temp;

					if (temp_sensors_runtime[i].target_temp > temp)
						temp++;
					else if (temp_sensors_runtime[i].target_temp < temp)
						temp--;

					temp_sensors_runtime[i].next_read_time = 0;

					break;
				#endif	/* TEMP_DUMMY */

				default: /* prevent compiler warning */
					break;
			}
			/* Exponentially Weighted Moving Average alpha constant for smoothing
			   noisy sensors. Instrument Engineer's Handbook, 4th ed, Vol 2 p126
			   says values of 0.05 to 0.1 for TEMP_EWMA are typical. */
			#ifndef TEMP_EWMA
				#define TEMP_EWMA 1.0
			#endif
			#define EWMA_SCALE  1024L
			#define EWMA_ALPHA  ((long) (TEMP_EWMA * EWMA_SCALE))
			temp_sensors_runtime[i].last_read_temp = (uint16_t) ((EWMA_ALPHA * temp +
			  (EWMA_SCALE-EWMA_ALPHA) * temp_sensors_runtime[i].last_read_temp
			                                         ) / EWMA_SCALE);
		}
		if (labs((int16_t)(temp_sensors_runtime[i].last_read_temp - temp_sensors_runtime[i].target_temp)) < (TEMP_HYSTERESIS*4)) {
			if (temp_sensors_runtime[i].temp_residency < (TEMP_RESIDENCY_TIME*120))
				temp_sensors_runtime[i].temp_residency++;
		}
		else {
			// Deal with flakey sensors which occasionally report a wrong value
			// by setting residency back, but not entirely to zero.
			if (temp_sensors_runtime[i].temp_residency > 10)
				temp_sensors_runtime[i].temp_residency -= 10;
			else
				temp_sensors_runtime[i].temp_residency = 0;
		}

		if (temp_sensors[i].heater < NUM_HEATERS) {
			heater_tick(temp_sensors[i].heater, temp_sensors[i].temp_type, temp_sensors_runtime[i].last_read_temp, temp_sensors_runtime[i].target_temp);
		}

    if (DEBUG_PID && (debug_flags & DEBUG_PID))
      sersendf_P(PSTR("DU temp: {%d %d %d.%d}"), i,
                 temp_sensors_runtime[i].last_read_temp,
                 temp_sensors_runtime[i].last_read_temp / 4,
                 (temp_sensors_runtime[i].last_read_temp & 0x03) * 25);
//	}
  if (DEBUG_PID && (debug_flags & DEBUG_PID))
    sersendf_P(PSTR("\n"));
}

/**
 * Report whether all temp sensors in use are reading their target
 * temperatures. Used for M116 and friends.
 */
uint8_t	temp_achieved() {
	temp_sensor_t i;
	uint8_t all_ok = 255;

	for (i = 0; i < NUM_TEMP_SENSORS; i++) {
    if (temp_sensors_runtime[i].target_temp > 0 &&
        temp_sensors_runtime[i].temp_residency < (TEMP_RESIDENCY_TIME*100))
			all_ok = 0;
	}
	return all_ok;
}

/// specify a target temperature
/// \param index sensor to set a target for
/// \param temperature target temperature to aim for
void temp_set(temp_sensor_t index, uint16_t temperature) {
	if (index >= NUM_TEMP_SENSORS)
		return;

	// only reset residency if temp really changed
	if (temp_sensors_runtime[index].target_temp != temperature) {
		temp_sensors_runtime[index].target_temp = temperature;
		temp_sensors_runtime[index].temp_residency = 0;
	#ifdef	TEMP_INTERCOM
		if (temp_sensors[index].temp_type == TT_INTERCOM)
			send_temperature(temp_sensors[index].temp_pin, temperature);
	#endif
	}
}

/// return most recent reading for a sensor
/// \param index sensor to read
uint16_t temp_get(temp_sensor_t index) {
	if (index >= NUM_TEMP_SENSORS)
		return 0;

	return temp_sensors_runtime[index].last_read_temp;
}

// extruder doesn't have sersendf_P
#ifndef	EXTRUDER
static void single_temp_print(temp_sensor_t index) {
	uint8_t c = (temp_sensors_runtime[index].last_read_temp & 3) * 25;
	sersendf_P(PSTR("%u.%u"), temp_sensors_runtime[index].last_read_temp >> 2, c);
}

/// send temperatures to host
/// \param index sensor value to send
void temp_print(temp_sensor_t index) {

	if (index == TEMP_SENSOR_none) { // standard behaviour
		#ifdef HEATER_EXTRUDER
			sersendf_P(PSTR("T:"));
			single_temp_print(HEATER_EXTRUDER);
		#endif
		#ifdef HEATER_BED
			sersendf_P(PSTR(" B:"));
			single_temp_print(HEATER_BED);
		#endif
	}
	else {
		if (index >= NUM_TEMP_SENSORS)
			return;
//		sersendf_P(PSTR("T[%su]:"), index);
		single_temp_print(index);
	}
}
#endif
