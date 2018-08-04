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
#include "clock.h"
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
static struct {
  uint_fast16_t         last_read_temp; ///< last received reading
  uint_fast16_t         target_temp;    ///< manipulate attached heater to attempt to achieve this value

  uint_fast16_t         temp_residency; ///< how long have we been close to target temperature in temp ticks?

  uint_fast8_t  active;          ///< State machine tracker for readers that need it.
} temp_sensors_runtime[NUM_TEMP_SENSORS];

/** \def TEMP_EWMA

  Default alpha constant for the Exponentially Weighted Moving Average (EWMA)
  for smoothing noisy sensors. Instrument Engineer's Handbook, 4th ed,
  Vol 2 p126 says values of 50 to 100 are typical.

  This is scaled by factor 1000. Setting it to 1000 turns EWMA off.
*/
#ifndef TEMP_EWMA
  #define TEMP_EWMA 1000
#endif

#define EWMA_SCALE  1024L
#define EWMA_ALPHA  ((TEMP_EWMA * EWMA_SCALE + 500) / 1000)

// If EWMA is used, continuously update analog reading for more data points.
#define TEMP_READ_CONTINUOUS (EWMA_ALPHA < EWMA_SCALE)
#define TEMP_NOT_READY       0xffff

/**
  Flag for wether we currently wait for achieving temperatures.
*/
static uint8_t wait_for_temp = 0;


// Automatic temperature report period (AUTOREPORT_TEMP)
static uint8_t          periodic_temp_seconds;
static temp_sensor_t    periodic_temp_index;
static uint8_t          periodic_temp_timer;

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

#ifdef TEMP_MAX6675
static uint16_t temp_max6675_read(temp_sensor_t i) {
  // Note: value reading in this section was rewritten without
  //       testing when spi.c/.h was introduced. --Traumflug
  // Note: MAX6675 can give a reading every 0.22s
  spi_select_max6675();
  // No delay required, see
  // https://github.com/Traumflug/Teacup_Firmware/issues/22

  // Read MSB.
  temp = spi_rw(0) << 8;
  // Read LSB.
  temp |= spi_rw(0);

  spi_deselect_max6675();

  if ((temp & 0x4) == 0) {
    temp = temp >> 3;
  }
  else { 
    // thermocouple open, send "not ready"
    temp = TEMP_NOT_READY;
    // and we will read it next time.
    temp_sensors_runtime[i].active = 0;
  }

  return temp;
}

static uint16_t temp_read_max6675(temp_sensor_t i) {
  switch (temp_sensors_runtime[i].active++) {
    case 1:
      return temp_max6675_read(i);

    case 22:  // read temperature at most every 220ms
      temp_sensors_runtime[i].active = 0;
      break;
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_MAX6675 */

#ifdef TEMP_THERMISTOR
static uint16_t temp_read_thermistor(temp_sensor_t i) {
  // Needs to be 'static', else GCC diagnostics emits a warning "'result' may
  // be used uninitialized". No surprise about this warning, it's part of the
  // logic that in some configurations temp_sensors_runtime[i].active never
  // reaches more than 1.
  static uint16_t result;

  switch (temp_sensors_runtime[i].active++) {
    case 1:  // Start ADC conversion.
      #ifdef NEEDS_START_ADC
        #if TEMP_READ_CONTINUOUS
          result = analog_read(i);
        #endif
        start_adc();
        #if ! TEMP_READ_CONTINUOUS
          return TEMP_NOT_READY;
        #endif
      #endif
      // If not in continuous mode or no need for start_adc() fall through.

    case 2:  // Convert temperature values.
      #if ! defined NEEDS_START_ADC || ! TEMP_READ_CONTINUOUS
        result = analog_read(i);
      #endif
      temp_sensors_runtime[i].active = 0;
      return temp_table_lookup(result, i);
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_THERMISTOR */

#ifdef TEMP_MCP3008
/**
  Read a measurement from the MCP3008 analog-digital-converter (ADC).

  \param channel The ADC channel to read.

  \return The raw ADC reading.

  Documentation for this ADC see

    https://www.adafruit.com/datasheets/MCP3008.pdf.
*/
static uint16_t temp_mcp3008_read(uint8_t channel) {
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

static uint16_t temp_read_mcp3008(temp_sensor_t i) {
  switch (temp_sensors_runtime[i].active++) {
    case 1:
      return temp_table_lookup(temp_mcp3008_read(temp_sensors[i].temp_pin), i);
    case 10:  // Idle for 100ms.
      temp_sensors_runtime[i].active = 0;
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_MCP3008 */

#ifdef TEMP_AD595
static uint16_t temp_read_ad595(temp_sensor_t i) {
  // Needs to be 'static', see comment in temp_read_thermistor().
  static uint16_t result;

  switch (temp_sensors_runtime[i].active++) {
    case 1:  // Start ADC conversion.
      #ifdef NEEDS_START_ADC
        #if TEMP_READ_CONTINUOUS
          result = analog_read(i);
        #endif
        start_adc();
        #if ! TEMP_READ_CONTINUOUS
          return TEMP_NOT_READY;
        #endif
      #endif
      // If not in continuous mode or no need for start_adc() fall through.

    case 2:  // Convert temperature values.
      #if ! TEMP_READ_CONTINUOUS
        result = analog_read(i);
      #endif
      temp_sensors_runtime[i].active = 0;
      // Convert >> 8 instead of >> 10 because internal temp is stored as
      // 14.2 fixed point.
      return (result * 500L) >> 8;
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_AD595 */

#ifdef TEMP_INTERCOM
static uint16_t temp_read_intercom(temp_sensor_t i) {
  switch (temp_sensors_runtime[i].active++) {
    case 1:
      return read_temperature(temp_sensors[i].temp_pin);
    case 25:  // Idle for 250ms.
      temp_sensors_runtime[i].active = 0;
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_INTERCOM */

#ifdef TEMP_DUMMY
static uint16_t dummy_temp[NUM_TEMP_SENSORS];

static uint16_t temp_read_dummy(temp_sensor_t i) {
  if (temp_sensors_runtime[i].target_temp > dummy_temp[i])
    dummy_temp[i]++;
  else if (temp_sensors_runtime[i].target_temp < dummy_temp[i])
    dummy_temp[i]--;

  switch (temp_sensors_runtime[i].active++) {
    case 1:
      return dummy_temp[i] ;
    case 5:  // Idle for 50ms.
      temp_sensors_runtime[i].active = 0;
  }
  return TEMP_NOT_READY;
}
#endif /* TEMP_DUMMY */

static uint16_t read_temp_sensor(temp_sensor_t i) {
  switch (temp_sensors[i].temp_type) {
    #ifdef TEMP_MAX6675
      case TT_MAX6675:
        return temp_read_max6675(i);
    #endif

    #ifdef TEMP_THERMISTOR
      case TT_THERMISTOR:
        return temp_read_thermistor(i);
    #endif

    #ifdef TEMP_MCP3008
      case TT_MCP3008:
        return temp_read_mcp3008(i);
    #endif

    #ifdef TEMP_AD595
      case TT_AD595:
        return temp_read_ad595(i);
    #endif

    #ifdef TEMP_PT100
      case TT_PT100:
        #warning TODO: PT100 code
        break;
    #endif

    #ifdef TEMP_INTERCOM
      case TT_INTERCOM:
        return temp_read_intercom(i);
    #endif

    #ifdef TEMP_DUMMY
      case TT_DUMMY:
        return temp_read_dummy(i);
    #endif

    default: /* Prevent compiler warning. */
      return 0;
  }
}

static void run_pid_loop(int i) {
  if (temp_sensors[i].heater < NUM_HEATERS) {
    heater_tick(temp_sensors[i].heater, temp_sensors[i].temp_type,
                temp_sensors_runtime[i].last_read_temp,
                temp_sensors_runtime[i].target_temp);
  }
}

/**
  Called every 10ms from clock.c. Check all temp sensors that are ready for
  checking. When complete, update the PID loop for sensors tied to heaters.
*/
void temp_sensor_tick() {
	temp_sensor_t i = 0;

	for (; i < NUM_TEMP_SENSORS; i++) {
    if (TEMP_READ_CONTINUOUS)
      if ( ! temp_sensors_runtime[i].active)
        temp_sensors_runtime[i].active = 1;

    if (temp_sensors_runtime[i].active) {
      uint16_t temp = read_temp_sensor(i);

      if (temp == TEMP_NOT_READY)
        continue;

      // Handle moving average.
      temp_sensors_runtime[i].last_read_temp = (uint16_t)(
        (EWMA_ALPHA * temp +
         (EWMA_SCALE - EWMA_ALPHA) * temp_sensors_runtime[i].last_read_temp) /
        EWMA_SCALE);

      if ( ! TEMP_READ_CONTINUOUS) {
        /**
          In one-shot mode we only update temps when triggered by the
          heater_tick for PID loops. So here we run the PID loop through a
          cycle. This must only be done four times per second to keep the PID
          values sane.
        */
        run_pid_loop(i);
      }
    }
  }
}

/**
  Called every 250ms from clock.c. Update heaters for all sensors.
*/
void temp_heater_tick() {
  temp_sensor_t i;

  for (i = 0; i < NUM_TEMP_SENSORS; i++)
    if (TEMP_READ_CONTINUOUS)
      run_pid_loop(i);
    else {
      // Signal all the temperature probes to begin reading. Each will run the
      // pid loop for us when it completes.
      temp_sensors_runtime[i].active = 1;
    }
}

/**
  Called every 1s from clock.c. Update temperature residency info.
*/
void temp_residency_tick() {
  temp_sensor_t i;

  for (i = 0; i < NUM_TEMP_SENSORS; i++) {
		if (labs((int16_t)(temp_sensors_runtime[i].last_read_temp - temp_sensors_runtime[i].target_temp)) < (TEMP_HYSTERESIS*4)) {
			if (temp_sensors_runtime[i].temp_residency < (TEMP_RESIDENCY_TIME*120))
        temp_sensors_runtime[i].temp_residency += 100;
		}
		else {
			// Deal with flakey sensors which occasionally report a wrong value
			// by setting residency back, but not entirely to zero.
      if (temp_sensors_runtime[i].temp_residency > 100)
        temp_sensors_runtime[i].temp_residency -= 100;
			else
				temp_sensors_runtime[i].temp_residency = 0;
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

/**
  Note that we're waiting for temperatures to achieve their target. Typically
  set by M116.
*/
void temp_set_wait() {
  wait_for_temp = 1;
}

/**
  Wether we're currently waiting for achieving temperatures.
*/
uint8_t temp_waiting(void) {
  if (temp_achieved()) {
    wait_for_temp = 0;
  }
  return wait_for_temp;
}

/**
  Wait until all temperatures are achieved.
*/
void temp_wait(void) {
  while (wait_for_temp && ! temp_achieved()) {
    clock();
  }
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

/// set parameters for periodic temperature reporting
/// \param secs reporting interval in seconds; 0 to disable reporting
/// \param index sensor to report
void temp_periodic_config(uint8_t secs, temp_sensor_t index) {
  periodic_temp_seconds = secs;
  periodic_temp_index = index;
  periodic_temp_timer = secs;
}

/// send temperatures to the host periodically.
/// Called once per second from clock.c
void temp_periodic_print() {
  if (periodic_temp_seconds == 0)
    return;
  if (--periodic_temp_timer != 0)
    return;

  temp_print(periodic_temp_index);
  periodic_temp_timer = periodic_temp_seconds;
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
