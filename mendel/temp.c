#include "temp.h"

#include	"machine.h"
#include	"pinout.h"

uint16_t	current_temp;
uint16_t	target_temp;

int16_t		heater_p;
int16_t		heater_i;
int16_t		heater_d;

void temp_setup() {
	SET_OUTPUT(SCK);
	SET_INPUT(MISO);
	SET_OUTPUT(SS);

	WRITE(SS, 0);
	WRITE(SCK, 0);
}

uint16_t temp_read() {
	uint16_t temp;
	SPCR = MASK(MSTR) | MASK(SPE);

	SPDR = 0;
	for (;(SPSR & MASK(SPIF)) == 0;);
	temp = SPDR << 8;

	SPDR = 0;
	for (;(SPSR & MASK(SPIF)) == 0;);
	temp |= SPDR;

	if ((temp & 0x8002) == 0) {
		// got "device id"
		if (temp & 4) {
			// thermocouple open
		}
		else {
			current_temp = temp >> 3;
			return current_temp;
		}
	}

	return 0;
}

void temp_set(uint16_t t) {
	target_temp = t;
}

void temp_tick() {
	uint16_t last_temp = current_temp;
	temp_read();

	int16_t	t_delta = target_temp - current_temp;

	// PID stuff
	heater_p = t_delta;
	heater_i += t_delta;
	// note: D follows temp rather than error so there's no large derivative when the target temperature changes
	heater_d = (current_temp - last_temp);

	int16_t pid_output = (heater_p * P_FACTOR) + (heater_i * I_FACTOR) + (heater_d * D_FACTOR);

#ifdef	HEATER_PIN_PWMABLE
	HEATER_PIN_PWMABLE = pid_output
#else
	if (pid_output > 0) {
		enable_heater();
	}
	else {
		disable_heater();
	}
#endif
}
