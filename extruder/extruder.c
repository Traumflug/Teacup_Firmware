#include	<stdint.h>
#include	<string.h>

#include	<avr/interrupt.h>

#include	"intercom.h"
#include	"analog.h"
#include	"config.h"
#include	"watchdog.h"

static uint8_t motor_pwm;

#define NUMTEMPS 20
short temptable[NUMTEMPS][2] = {
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

void io_init(void) {
	// setup I/O pins
	WRITE(DEBUG_LED, 0); SET_OUTPUT(DEBUG_LED);
	WRITE(H1D,0); SET_OUTPUT(H1D);
	WRITE(H1E,0); SET_OUTPUT(H1E);
	WRITE(H2D,0); SET_OUTPUT(H2D);
	WRITE(H2E,0); SET_OUTPUT(H2E);

	SET_INPUT(TRIM_POT);
	SET_INPUT(TEMP_PIN);
	SET_INPUT(E_STEP_PIN);
	SET_INPUT(E_DIR_PIN);

	//Enable the RS485 transceiver
	SET_OUTPUT(RX_ENABLE_PIN);
	SET_OUTPUT(TX_ENABLE_PIN);
	disable_transmit();

	#ifdef	HEATER_PIN
		WRITE(HEATER_PIN, 0); SET_OUTPUT(HEATER_PIN);
	#endif

	#ifdef BED_PIN
		WRITE(BED_PIN, 0); SET_OUTPUT(BED_PIN);
	#endif

	#if defined(HEATER_PWM) || defined(FAN_PWM) || defined(BED_PWM)
		// setup PWM timer: fast PWM, no prescaler
		TCCR2A = MASK(WGM21) | MASK(WGM20);
		TCCR2B = MASK(CS22);
		TIMSK2 = 0;
		OCR2A = 0;
		OCR2B = 0;
	#endif

	#if defined(H1E_PWM) && defined(H2E_PWM)
		TCCR0A = MASK(WGM01) | MASK(WGM00);
		TCCR0B = MASK(CS20);
		TIMSK0 = 0;
		OCR0A = 0;
		OCR0B = 0;
	#endif
}

void motor_init(void) {
	//Enable an interrupt to be triggered when the step pin changes
	//This will be PCIE0
	PCICR = MASK(PCIE0);
	PCMSK0 = MASK(PCINT2);
}

ISR(PCINT0_vect) {
	static uint8_t coil_pos, pwm, flag;

	if (flag == 1) flag = 0;
	else flag = 1;
		
	//if the step pin is high, we advance the motor
	if (flag) {

		//Turn on motors only on first tick to save power I guess
		enable_motors();

		//Advance the coil position
		if (READ(E_DIR_PIN)) 
			coil_pos++;
		else
			coil_pos--;

		coil_pos &= 31;

		//Grab the latest motor power to use
		pwm = motor_pwm;

		switch(coil_pos >> 2) {
			case 0:
			  WRITE(H1D, 0);
			  WRITE(H2D, 0);
			  H1E_PWM = 0;
			  H2E_PWM = pwm;
			  break;
			case 1:
			  WRITE(H1D, 1);
			  WRITE(H2D, 0);
			  H1E_PWM = pwm;
			  H2E_PWM = pwm;
			  break;
			case 2:
			  WRITE(H1D, 1);
			  WRITE(H2D, 0);
			  H1E_PWM = pwm;
			  H2E_PWM = 0;
			  break;
			case 3:
			  WRITE(H1D, 1);
			  WRITE(H2D, 1);
			  H1E_PWM = pwm;
			  H2E_PWM = pwm;
			  break;
			case 4:
			  WRITE(H1D, 1);
			  WRITE(H2D, 1);
			  H1E_PWM = 0;
			  H2E_PWM = pwm;  
			  break;
			case 5:
			  WRITE(H1D, 0);
			  WRITE(H2D, 1);
			  H1E_PWM = pwm;
			  H2E_PWM = pwm;  
			  break;
			case 6:
			  WRITE(H1D, 0);
			  WRITE(H2D, 1);
			  H1E_PWM = pwm;
			  H2E_PWM = 0;  
			  break;
			case 7:
			  WRITE(H1D, 0);
			  WRITE(H2D, 0);
			  H1E_PWM = pwm;
			  H2E_PWM = pwm;  
			  break;
		}
	}
}

void init(void) {
	// set up watchdog
	wd_init();

	// setup analog reading
	analog_init();

	// set up serial
	intercom_init();

	// set up inputs and outputs
	io_init();

	// set up extruder motor driver
	motor_init();

	// enable interrupts
	sei();

	// reset watchdog
	wd_reset();
}


int main (void)
{
	static uint8_t i;
	static uint16_t raw_temp;

	init();

	enable_heater();

	start_send();

	// main loop
	for (;;)
	{
		wd_reset();

		//Read motor PWM
		motor_pwm = analog_read(TRIM_POT_CHANNEL) >> 2;

		//Read current temperature
		raw_temp = analog_read(TEMP_PIN_CHANNEL);

		//Calculate real temperature based on lookup table
		for (i = 1; i < NUMTEMPS; i++) {
			if (temptable[i][0] > raw_temp) {
				raw_temp = temptable[i][1] + 
					(temptable[i][0] - raw_temp) * (temptable[i-1][1] - temptable[i][1]) / (temptable[i][0] - temptable[i-1][0]);
		
				break;
			}
		}

		//Clamp for overflows
		if (i == NUMTEMPS) raw_temp = temptable[NUMTEMPS-1][1];
		if (raw_temp > 255) raw_temp = 255;

		//Update the intercom values
		update_send_cmd(raw_temp);

		HEATER_PWM = get_read_cmd();
	}
}
