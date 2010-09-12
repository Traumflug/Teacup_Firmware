#include	"heater.h"

#include	<avr/eeprom.h>

#include	"sersendf.h"
#include	"machine.h"
#include	"pinout.h"
#include	"debug.h"
#include	"arduino.h"

int16_t		heater_p     = 0;
int16_t		heater_i     = 0;
int16_t		heater_d     = 0;

#define		DEFAULT_P				8192
#define		DEFAULT_I				512
#define		DEFAULT_D				-24576
#define		DEFAULT_I_LIMIT	384
int32_t		p_factor			= 0;
int32_t		i_factor			= 0;
int32_t		d_factor			= 0;
int16_t		i_limit				= 0;

int32_t		EEMEM EE_p_factor;
int32_t		EEMEM EE_i_factor;
int32_t		EEMEM	EE_d_factor;
int16_t		EEMEM EE_i_limit;

#define		TH_COUNT	8
uint16_t	temp_history[TH_COUNT] __attribute__ ((__section__ (".bss")));
uint8_t		th_p = 0;

#ifndef	ABSDELTA
#define	ABSDELTA(a, b)	(((a) >= (b))?((a) - (b)):((b) - (a)))
#endif

void heater_init() {
	p_factor = eeprom_read_dword((uint32_t *) &EE_p_factor);
	i_factor = eeprom_read_dword((uint32_t *) &EE_i_factor);
	d_factor = eeprom_read_dword((uint32_t *) &EE_d_factor);
	i_limit = eeprom_read_word((uint16_t *) &EE_i_limit);
	
	if ((p_factor == 0) && (i_factor == 0) && (d_factor == 0) && (i_limit == 0)) {
		p_factor = DEFAULT_P;
		i_factor = DEFAULT_I;
		d_factor = DEFAULT_D;
		i_limit = DEFAULT_I_LIMIT;
	}
}

void heater_save_settings() {
	eeprom_write_dword((uint32_t *) &EE_p_factor, p_factor);
	eeprom_write_dword((uint32_t *) &EE_i_factor, i_factor);
	eeprom_write_dword((uint32_t *) &EE_d_factor, d_factor);
	eeprom_write_word((uint16_t *) &EE_i_limit, i_limit);
}

void heater_tick(int16_t current_temp, int16_t target_temp) {
	int16_t	t_error = target_temp - current_temp;
	
	temp_history[th_p++] = current_temp;
	th_p &= (TH_COUNT - 1);
	
	// PID stuff
	// proportional
	heater_p = t_error;
	
	// integral
	heater_i += t_error;
	// prevent integrator wind-up
	if (heater_i > i_limit)
		heater_i = i_limit;
	else if (heater_i < -i_limit)
		heater_i = -i_limit;
	
	// derivative
	// note: D follows temp rather than error so there's no large derivative when the target changes
	heater_d = current_temp - temp_history[th_p];
	
	// combine factors
	int32_t pid_output_intermed = (
		(
			(((int32_t) heater_p) * p_factor) +
			(((int32_t) heater_i) * i_factor) +
			(((int32_t) heater_d) * d_factor)
		) / PID_SCALE
	);
	
	// rebase and limit factors
	uint8_t pid_output;
	if (pid_output_intermed > 255)
		pid_output = 255;
	else if (pid_output_intermed < 0)
		pid_output = 0;
	else
		pid_output = pid_output_intermed & 0xFF;
	
	if (debug_flags & DEBUG_PID)
		sersendf_P(PSTR("T{E:%d, P:%d * %ld = %ld / I:%d * %ld = %ld / D:%d * %ld = %ld # O: %ld = %u}\n"), t_error, heater_p, p_factor, (int32_t) heater_p * p_factor / PID_SCALE, heater_i, i_factor, (int32_t) heater_i * i_factor / PID_SCALE, heater_d, d_factor, (int32_t) heater_d * d_factor / PID_SCALE, pid_output_intermed, pid_output);
	
	#ifdef	HEATER_PWM
	HEATER_PWM = pid_output;
	#else
	if (pid_output >= 8)
		enable_heater();
	else
		disable_heater();
	#endif
}
