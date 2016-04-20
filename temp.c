#include	"temp.h"

/** \file
	\brief Manage temperature sensors

  \note All temperatures are stored as 14.2 fixed point in Teacup, so we have
  a range of 0 - 16383.75 deg Celsius at a precision of 0.25 deg. That includes
  the thermistor table, which is why you can't copy and paste one from other
  firmwares which don't do this.
*/

#include	<stdlib.h>
#include	"arduino.h"
#include "serial.h"
#include	"debug.h"
#ifndef	EXTRUDER
	#include	"sersendf.h"
#endif
#include	"heater.h"
#include "simulator.h"

#ifdef	TEMP_INTERCOM
  #ifdef __ARMEL__
    #error TEMP_INTERCOM not yet supported on ARM.
  #endif
	#include	"intercom.h"
  #include "pinio.h"
#endif

#ifdef	TEMP_MAX6675
  #ifdef __ARMEL__
    #error MAX6675 sensors (TEMP_MAX6675) not yet supported on ARM.
  #endif
  #include "spi.h"
#endif

#ifdef TEMP_MCP3008
  #ifdef __ARMEL__
    #error MCP3008 sensors (TEMP_MCP3008) not yet supported on ARM.
  #endif
  #include "spi.h"
  #include "thermistortable.h"
#endif

#ifdef	TEMP_THERMISTOR
#include	"analog.h"
#include	"thermistortable.h"
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
	uint8_t		additional; ///< additional, sensor type specifc config
} temp_sensor_definition_t;

#undef DEFINE_TEMP_SENSOR
/// help build list of sensors from entries in config.h
#ifndef SIMULATOR
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) { (type), (pin ## _ADC), (HEATER_ ## name), (additional) },
#else
#define DEFINE_TEMP_SENSOR(name, type, pin, additional) { (type), (TEMP_SENSOR_ ## name), (HEATER_ ## name), (additional) },
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

/// Set up temp sensors.
void temp_init() {
	temp_sensor_t i;
	for (i = 0; i < NUM_TEMP_SENSORS; i++) {
		switch(temp_sensors[i].temp_type) {
      #ifdef TEMP_MAX6675
        case TT_MAX6675:
          // Note that MAX6675's Chip Select pin is currently hardcoded to SS.
          // This isn't neccessary. See also spi.h.
          spi_deselect_max6675();
          break;
      #endif

      #ifdef TEMP_MCP3008
        case TT_MCP3008:
          SET_OUTPUT(MCP3008_SELECT_PIN);
          spi_deselect_mcp3008();
          break;
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

      #ifdef TEMP_INTERCOM
        case TT_INTERCOM:
          // Enable the RS485 transceiver
          SET_OUTPUT(RX_ENABLE_PIN);
          SET_OUTPUT(TX_ENABLE_PIN);
          WRITE(RX_ENABLE_PIN,0);
          disable_transmit();

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
  Read a measurement from the MCP3008 analog-digital-converter (ADC).

  \param channel The ADC channel to read.

  \return The raw ADC reading.

  Documentation for this ADC see

    https://www.adafruit.com/datasheets/MCP3008.pdf.
*/
#ifdef TEMP_MCP3008
static uint16_t mcp3008_read(uint8_t channel) {
  uint8_t temp_h, temp_l;

  spi_select_mcp3008();

  // Start bit.
  spi_rw(0x01);

  // Send read address and get MSB, then LSB byte.
  temp_h = spi_rw((0b1000 | channel) << 4) & 0b11;
  temp_l = spi_rw(0);

  spi_deselect_mcp3008();

  return temp_h << 8 | temp_l;
}
#endif /* TEMP_MCP3008 */

/**
  Look up a degree Celsius value from a raw ADC reading.

  \param temp   The raw ADC reading to look up.

  \param sensor The sensor to look up. Each sensor can have its own table.

  \return Degree Celsius reading in 14.2 fixed decimal format.

  The table(s) looked up here are in thermistortable.h and are created on the
  fly by Configtool when saving config.h. They contain value pairs mapping
  raw ADC readings to 14.2 values already, so all we have to do here is to
  inter-/extrapolate.
*/
#if defined TEMP_THERMISTOR || defined TEMP_MCP3008
static uint16_t temp_table_lookup(uint16_t temp, uint8_t sensor) {
  uint8_t lo, hi;
  uint8_t table_num = temp_sensors[sensor].additional;

  // Binary search for table value bigger than our target.
  //
  //   lo = index of highest entry less than target.
  //   hi = index of lowest entry greater than or equal to target.
  for (lo = 0, hi = NUMTEMPS - 1; hi - lo > 1; ) {
    uint8_t j = lo + (hi - lo) / 2 ;
    if (pgm_read_word(&(temptable[table_num][j][0])) >= temp)
      hi = j ;
    else
      lo = j ;
  }

  if (DEBUG_PID && (debug_flags & DEBUG_PID))
    sersendf_P(PSTR("pin:%d Raw ADC:%d table entry: %d"),
               temp_sensors[sensor].temp_pin, temp, hi);

  if (sizeof(temptable[0][0]) == 2 * sizeof(uint16_t)) {
    /**
      This code handles temptables with value pairs and is deprecated.
      It's kept for compatibility with legacy, handcrafted tables, only.

      The new code expects tables with triples, nevertheless it's smaller
      and also faster. Configtool was already changed to create tables
      with triples, only.
    */
    // Wikipedia's example linear interpolation formula.
    // y = ((x - x₀)y₁ + (x₁-x)y₀) / (x₁ - x₀)
    // y = temp
    // x = ADC reading
    // x₀= temptable[lo][0]
    // x₁= temptable[hi][0]
    // y₀= temptable[lo][1]
    // y₁= temptable[hi][1]
    temp = (
      // ((x - x₀)y₁
      ((uint32_t)temp - pgm_read_word(&(temptable[table_num][lo][0]))) *
                        pgm_read_word(&(temptable[table_num][hi][1]))
      //             +
      +
      //               (x₁-x)y₀)
      (pgm_read_word(&(temptable[table_num][hi][0])) - (uint32_t)temp) *
        pgm_read_word(&(temptable[table_num][lo][1])))
      //                        /
      /
      //                          (x₁ - x₀)
      (pgm_read_word(&(temptable[table_num][hi][0])) -
       pgm_read_word(&(temptable[table_num][lo][0])));
  } else
  if (sizeof(temptable[0][0]) == 3 * sizeof(uint16_t)) {
    // Linear interpolation using pre-computed slope.
    // y = y₁ - (x - x₁) * d₁
    #define X1 pgm_read_word(&(temptable[table_num][hi][0]))
    #define Y1 pgm_read_word(&(temptable[table_num][hi][1]))
    #define D1 pgm_read_word(&(temptable[table_num][hi][2]))

    temp = Y1 - ((((int32_t)temp - X1) * D1 + (1 << 7)) >> 8);
  }

  if (DEBUG_PID && (debug_flags & DEBUG_PID))
    sersendf_P(PSTR(" temp:%d.%d"), temp / 4, (temp % 4) * 25);

  if (DEBUG_PID && (debug_flags & DEBUG_PID))
    sersendf_P(PSTR(" Sensor:%d\n"), sensor);

  return temp;
}
#endif /* TEMP_THERMISTOR || TEMP_MCP3008 */

/// called every 10ms from clock.c - check all temp sensors that are ready for checking
void temp_sensor_tick() {
	temp_sensor_t i = 0;

	for (; i < NUM_TEMP_SENSORS; i++) {
		if (temp_sensors_runtime[i].next_read_time) {
			temp_sensors_runtime[i].next_read_time--;
		}
		else {
			uint16_t	temp = 0;
			//time to deal with this temp sensor
			switch(temp_sensors[i].temp_type) {
				#ifdef	TEMP_MAX6675
				case TT_MAX6675:
          // Note: value reading in this section was rewritten without
          //       testing when spi.c/.h was introduced. --Traumflug
          spi_select_max6675();
					// No delay required, see
					// https://github.com/triffid/Teacup_Firmware/issues/22

					// read MSB
          temp = spi_rw(0) << 8;
					// read LSB
          temp |= spi_rw(0);

          spi_deselect_max6675();

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
          case TT_THERMISTOR:
            // Read current temperature.
            temp = temp_table_lookup(analog_read(i), i);

            temp_sensors_runtime[i].next_read_time = 0;
            break;
				#endif	/* TEMP_THERMISTOR */

        #ifdef TEMP_MCP3008
          case TT_MCP3008:
            // Read current temperature.
            temp = temp_table_lookup(mcp3008_read(temp_sensors[i].temp_pin), i);

            // This is an SPI read so it is not as fast as on-chip ADC. A read
            // every 100ms should be sufficient.
            temp_sensors_runtime[i].next_read_time = 10;
            break;
        #endif /* TEMP_MCP3008 */

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
	}
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
  #ifdef REPORT_TARGET_TEMPS
    sersendf_P(PSTR("/"));
    c = (temp_sensors_runtime[index].target_temp & 3) * 25;
    sersendf_P(PSTR("%u.%u"), temp_sensors_runtime[index].target_temp >> 2, c);
  #endif
}

/// send temperatures to host
/// \param index sensor value to send
void temp_print(temp_sensor_t index) {

	if (index == TEMP_SENSOR_none) { // standard behaviour
		#ifdef HEATER_EXTRUDER
			sersendf_P(PSTR("T:"));
      single_temp_print(TEMP_SENSOR_extruder);
		#endif
		#ifdef HEATER_BED
			sersendf_P(PSTR(" B:"));
      single_temp_print(TEMP_SENSOR_bed);
		#endif
	}
	else {
		if (index >= NUM_TEMP_SENSORS)
			return;
		sersendf_P(PSTR("T[%su]:"), index);
		single_temp_print(index);
	}
  serial_writechar('\n');
}
#endif
