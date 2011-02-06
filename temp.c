#include	"temp.h"

#include	<stdlib.h>
#include	<avr/eeprom.h>
#include	<avr/pgmspace.h>

#include	"arduino.h"
#include	"delay.h"
#include	"debug.h"
#ifndef	EXTRUDER
	#include	"sersendf.h"
#endif
#include	"heater.h"
#ifdef	TEMP_INTERCOM
	#include	"intercom.h"
#endif

typedef enum {
	TT_THERMISTOR,
	TT_MAX6675,
	TT_AD595,
	TT_PT100,
	TT_INTERCOM,
	TT_DUMMY,
} temp_types;

typedef enum {
	PRESENT,
	TCOPEN
} temp_flags_enum;

typedef struct {
	uint8_t temp_type;
	uint8_t temp_pin;
	uint8_t heater_index;
} temp_sensor_definition_t;

#undef DEFINE_TEMP_SENSOR
#define DEFINE_TEMP_SENSOR(name, type, pin) { (type), (pin), (HEATER_ ## name) },
static const temp_sensor_definition_t temp_sensors[NUM_TEMP_SENSORS] =
{
	#include	"config.h"
};
#undef DEFINE_TEMP_SENSOR

// this struct holds the runtime sensor data- read temperatures, targets, etc
struct {
	temp_flags_enum		temp_flags;
	
	uint16_t					last_read_temp;
	uint16_t					target_temp;
	
	uint8_t						temp_residency;
	
	uint16_t					next_read_time;
} temp_sensors_runtime[NUM_TEMP_SENSORS];

#ifdef	TEMP_MAX6675
#endif

#ifdef	TEMP_THERMISTOR
#include	"analog.h"

#define NUMTEMPS 20
uint16_t temptable[NUMTEMPS][2] PROGMEM = {
	{1, 841},
	{54, 255},
	{107, 209},
	{160, 184},
	{213, 166},
	{266, 153},
	{319, 142},
	{372, 132},
	{425, 124},
	{478, 116},
	{531, 108},
	{584, 101},
	{637, 93},
	{690, 86},
	{743, 78},
	{796, 70},
	{849, 61},
	{902, 50},
	{955, 34},
	{1008, 3}
};
#endif

#ifdef	TEMP_AD595
#include	"analog.h"
#endif

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
				update_send_cmd(0);
				break;
		#endif
		}
	}
}

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
					#ifdef	PRR
						PRR &= ~MASK(PRSPI);
					#elif defined PRR0
						PRR0 &= ~MASK(PRSPI);
					#endif
					
					SPCR = MASK(MSTR) | MASK(SPE) | MASK(SPR0);
					
					// enable TT_MAX6675
					WRITE(SS, 0);
					
					// ensure 100ns delay - a bit extra is fine
					delay(1);
					
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
				case TT_THERMISTOR:
					do {
						uint8_t j;
						//Read current temperature
						temp = analog_read(temp_sensors[i].temp_pin);

						//Calculate real temperature based on lookup table
						for (j = 1; j < NUMTEMPS; j++) {
							if (pgm_read_word(&(temptable[j][0])) > temp) {
								// multiply by 4 because internal temp is stored as 14.2 fixed point
								temp = pgm_read_word(&(temptable[j][1])) * 4 + (temp - pgm_read_word(&(temptable[j-1][0]))) * 4 * (pgm_read_word(&(temptable[j][1])) - pgm_read_word(&(temptable[j-1][1]))) / (pgm_read_word(&(temptable[j][0])) - pgm_read_word(&(temptable[j-1][0])));
								break;
							}
						}

						//Clamp for overflows
						if (j == NUMTEMPS)
							temp = temptable[NUMTEMPS-1][1] * 4;

						temp_sensors_runtime[i].next_read_time = 0;
					} while (0);
					break;
				#endif	/* TEMP_THERMISTOR */
					
				#ifdef	TEMP_AD595
				case TT_AD595:
					temp = analog_read(temp_pin);
					
					// convert
					// >>8 instead of >>10 because internal temp is stored as 14.2 fixed point
					temp = (temp * 500L) >> 8;
					
					temp_sensors_runtime[i].next_read_time = 0;
					
					break;
				#endif	/* TEMP_AD595 */

				#ifdef	TEMP_PT100
				case TT_PT100:
					#warning TODO: PT100 code
					break
				#endif	/* TEMP_PT100 */

				#ifdef	TEMP_INTERCOM
				case TT_INTERCOM:
					temp = get_read_cmd() << 2;

					start_send();

					temp_sensors_runtime[i].next_read_time = 0;

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
			}
			temp_sensors_runtime[i].last_read_temp = temp;
			
			if (labs(temp - temp_sensors_runtime[i].target_temp) < TEMP_HYSTERESIS) {
				if (temp_sensors_runtime[i].temp_residency < TEMP_RESIDENCY_TIME)
					temp_sensors_runtime[i].temp_residency++;
			}
			else {
				temp_sensors_runtime[i].temp_residency = 0;
			}
			
			if (temp_sensors[i].heater_index < NUM_HEATERS) {
				heater_tick(temp_sensors[i].heater_index, i, temp_sensors_runtime[i].last_read_temp, temp_sensors_runtime[i].target_temp);
			}
		}
	}
}

uint8_t	temp_achieved() {
	temp_sensor_t i;
	uint8_t all_ok = 255;

	for (i = 0; i < NUM_TEMP_SENSORS; i++) {
		if (temp_sensors_runtime[i].temp_residency < TEMP_RESIDENCY_TIME)
			all_ok = 0;
	}
	return all_ok;
}

void temp_set(temp_sensor_t index, uint16_t temperature) {
	if (index >= NUM_TEMP_SENSORS)
		return;

	temp_sensors_runtime[index].target_temp = temperature;
	temp_sensors_runtime[index].temp_residency = 0;
#ifdef	TEMP_INTERCOM
	if (temp_sensors[index].temp_type == TT_INTERCOM)
		update_send_cmd(temperature >> 2);
#endif
}

uint16_t temp_get(temp_sensor_t index) {
	if (index >= NUM_TEMP_SENSORS)
		return 0;

	return temp_sensors_runtime[index].last_read_temp;
}

// extruder doesn't have sersendf_P
#ifndef	EXTRUDER
void temp_print(temp_sensor_t index) {
	uint8_t c = 0;

	if (index >= NUM_TEMP_SENSORS)
		return;

	c = (temp_sensors_runtime[index].last_read_temp & 3) * 25;

	sersendf_P(PSTR("T:%u.%u"), temp_sensors_runtime[index].last_read_temp >> 2, c);
	#ifdef HEATER_BED
		uint8_t b = 0;
		b = (temp_sensors_runtime[HEATER_BED].last_read_temp & 3) * 25;
	
		sersendf_P(PSTR(" B:%u.%u"), temp_sensors_runtime[HEATER_bed].last_read_temp >> 2 , b);
	#endif

}
#endif
