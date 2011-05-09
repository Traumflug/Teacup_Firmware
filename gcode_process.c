#include	"gcode_process.h"

/** \file
	\brief Work out what to do with received G-Code commands
*/

#include	<string.h>

#include	"gcode_parse.h"

#include	"dda_queue.h"
#include	"watchdog.h"
#include	"delay.h"
#include	"serial.h"
#include	"sermsg.h"
#include	"temp.h"
#include	"heater.h"
#include	"timer.h"
#include	"sersendf.h"
#include	"pinio.h"
#include	"debug.h"
#include	"clock.h"
#include	"config.h"
#include	"home.h"

/// the current tool
uint8_t tool;

/// the tool to be changed when we get an M6
uint8_t next_tool;


/*
	private functions

	this is where we construct a move without a gcode command, useful for gcodes which require multiple moves eg; homing
*/

/// move to X = 0
static void zero_x(void) {
	TARGET t = startpoint;
	t.X = 0;
	t.F = SEARCH_FEEDRATE_X;
	enqueue(&t);
}

/// move to Y = 0
static void zero_y(void) {
	TARGET t = startpoint;
	t.Y = 0;
	t.F = SEARCH_FEEDRATE_Y;
	enqueue(&t);
}

/// move to Z = 0
static void zero_z(void) {
	TARGET t = startpoint;
	t.Z = 0;
	t.F = SEARCH_FEEDRATE_Z;
	enqueue(&t);
}

#if E_STARTSTOP_STEPS > 0
/// move E by a certain amount at a certain speed
static void SpecialMoveE(int32_t e, uint32_t f) {
	TARGET t = startpoint;
	t.E += e;
	t.F = f;
	enqueue(&t);
}
#endif /* E_STARTSTOP_STEPS > 0 */

/************************************************************************//**

  \brief Processes command stored in global \ref next_target.
  This is where we work out what to actually do with each command we
    receive. All data has already been scaled to integers in gcode_process.
    If you want to add support for a new G or M code, this is the place.


*//*************************************************************************/

void process_gcode_command() {
	uint32_t	backup_f;

	// convert relative to absolute
	if (next_target.option_relative) {
		next_target.target.X += startpoint.X;
		next_target.target.Y += startpoint.Y;
		next_target.target.Z += startpoint.Z;
		#ifdef	E_ABSOLUTE
			next_target.target.E += startpoint.E;
		#endif
	}
	// E ALWAYS relative, otherwise we overflow our registers after only a few layers
	// 	next_target.target.E += startpoint.E;
	// easier way to do this
	// 	startpoint.E = 0;
	// moved to dda.c, end of dda_create() and dda_queue.c, next_move()

	// implement axis limits
	#ifdef	X_MIN
		if (next_target.target.X < (X_MIN * STEPS_PER_MM_X))
			next_target.target.X = X_MIN * STEPS_PER_MM_X;
	#endif
	#ifdef	X_MAX
		if (next_target.target.X > (X_MAX * STEPS_PER_MM_X))
			next_target.target.X = X_MAX * STEPS_PER_MM_X;
	#endif
	#ifdef	Y_MIN
		if (next_target.target.Y < (Y_MIN * STEPS_PER_MM_Y))
			next_target.target.Y = Y_MIN * STEPS_PER_MM_Y;
	#endif
	#ifdef	Y_MAX
		if (next_target.target.Y > (Y_MAX * STEPS_PER_MM_Y))
			next_target.target.Y = Y_MAX * STEPS_PER_MM_Y;
	#endif
	#ifdef	Z_MIN
		if (next_target.target.Z < (Z_MIN * STEPS_PER_MM_Z))
			next_target.target.Z = Z_MIN * STEPS_PER_MM_Z;
	#endif
	#ifdef	Z_MAX
		if (next_target.target.Z > (Z_MAX * STEPS_PER_MM_Z))
			next_target.target.Z = Z_MAX * STEPS_PER_MM_Z;
	#endif


	if (next_target.seen_T) {
		next_tool = next_target.T;
	}

	if (next_target.seen_G) {
		uint8_t axisSelected = 0;
		switch (next_target.G) {
			// 	G0 - rapid, unsynchronised motion
			// since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
			case 0:
				backup_f = next_target.target.F;
				next_target.target.F = MAXIMUM_FEEDRATE_X * 2L;
				enqueue(&next_target.target);
				next_target.target.F = backup_f;
				break;

				//	G1 - synchronised motion
			case 1:
				enqueue(&next_target.target);
				break;

				//	G2 - Arc Clockwise
				// unimplemented

				//	G3 - Arc Counter-clockwise
				// unimplemented

				//	G4 - Dwell
			case 4:
				// wait for all moves to complete
				queue_wait();
				// delay
				for (;next_target.P > 0;next_target.P--) {
					ifclock(clock_flag_10ms) {
						clock_10ms();
					}
					delay_ms(1);
				}
				break;

				//	G20 - inches as units
			case 20:
				next_target.option_inches = 1;
				break;

				//	G21 - mm as units
			case 21:
				next_target.option_inches = 0;
				break;

				//	G30 - go home via point
			case 30:
				enqueue(&next_target.target);
				// no break here, G30 is move and then go home

				//	G28 - go home
			case 28:
				queue_wait();

				if (next_target.seen_X) {
					zero_x();
					axisSelected = 1;
				}
				if (next_target.seen_Y) {
					zero_y();
					axisSelected = 1;
				}
				if (next_target.seen_Z) {
					zero_z();
					axisSelected = 1;
				}
				// there's no point in moving E, as E has no endstops

				if (!axisSelected) {
					zero_x();
					zero_y();
					zero_z();
				}

				break;

				//	G90 - absolute positioning
				case 90:
					next_target.option_relative = 0;
					break;

					//	G91 - relative positioning
				case 91:
					next_target.option_relative = 1;
					break;

					//	G92 - set home
				case 92:
					// wait for queue to empty
					queue_wait();

					if (next_target.seen_X) {
						startpoint.X = current_position.X = next_target.target.X;
						axisSelected = 1;
					}
					if (next_target.seen_Y) {
						startpoint.Y = current_position.Y = next_target.target.Y;
						axisSelected = 1;
					}
					if (next_target.seen_Z) {
						startpoint.Z = current_position.Z = next_target.target.Z;
						axisSelected = 1;
					}
					if (next_target.seen_E) {
						#ifdef	E_ABSOLUTE
							startpoint.E = current_position.E = next_target.target.E;
						#endif
						axisSelected = 1;
					}

					if (axisSelected == 0) {
						startpoint.X = current_position.X = next_target.target.X =
						startpoint.Y = current_position.Y = next_target.target.Y =
						startpoint.Z = current_position.Z = next_target.target.Z = 0;
					}
					break;

				// G161 - Home negative
				case 161:
					if (next_target.seen_X)
						home_x_negative();
					if (next_target.seen_Y)
						home_y_negative();
					if (next_target.seen_Z)
						home_z_negative();
					break;
				// G162 - Home positive
				case 162:
					if (next_target.seen_X)
						home_x_positive();
					if (next_target.seen_Y)
						home_y_positive();
					if (next_target.seen_Z)
						home_z_positive();
					break;

					// unknown gcode: spit an error
				default:
					sersendf_P(PSTR("E: Bad G-code %d"), next_target.G);
					// newline is sent from gcode_parse after we return
					return;
		}
		#ifdef	DEBUG
			if (debug_flags & DEBUG_POSITION)
				print_queue();
		#endif
	}
	else if (next_target.seen_M) {
		switch (next_target.M) {
			// M2- program end
			case 2:
				timer_stop();
				queue_flush();
				x_disable();
				y_disable();
				z_disable();
				e_disable();
				power_off();
				for (;;)
					wd_reset();
				break;

			// M6- tool change
			case 6:
				tool = next_tool;
				break;
			// M3/M101- extruder on
			case 3:
			case 101:
				if (temp_achieved() == 0) {
					enqueue(NULL);
				}
				#ifdef DC_EXTRUDER
					heater_set(DC_EXTRUDER, DC_EXTRUDER_PWM);
				#elif E_STARTSTOP_STEPS > 0
					do {
						// backup feedrate, move E very quickly then restore feedrate
						backup_f = startpoint.F;
						startpoint.F = MAXIMUM_FEEDRATE_E;
						SpecialMoveE(E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
						startpoint.F = backup_f;
					} while (0);
				#endif
				break;

			// M102- extruder reverse

			// M5/M103- extruder off
			case 5:
			case 103:
				#ifdef DC_EXTRUDER
					heater_set(DC_EXTRUDER, 0);
				#elif E_STARTSTOP_STEPS > 0
					do {
						// backup feedrate, move E very quickly then restore feedrate
						backup_f = startpoint.F;
						startpoint.F = MAXIMUM_FEEDRATE_E;
						SpecialMoveE(-E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
						startpoint.F = backup_f;
					} while (0);
				#endif
				break;

			// M104- set temperature
			case 104:
				temp_set(next_target.P, next_target.S);
				if (next_target.S)
					power_on();
				break;

			// M105- get temperature
			case 105:
				temp_print(next_target.P);
				break;

			// M7/M106- fan on
			case 7:
			case 106:
				#ifdef HEATER_FAN
					heater_set(HEATER_FAN, 255);
				#endif
				break;
			// M107- fan off
			case 9:
			case 107:
				#ifdef HEATER_FAN
					heater_set(HEATER_FAN, 0);
				#endif
				break;

			// M109- set temp and wait
			case 109:
				if (next_target.seen_S)
					temp_set(next_target.P, next_target.S);
				if (next_target.S) {
					power_on();
					enable_heater();
				}
				else {
					disable_heater();
				}
				enqueue(NULL);
				break;

			// M110- set line number
			case 110:
				// this is a no-op in Teacup
				break;
			// M111- set debug level
			#ifdef	DEBUG
			case 111:
				debug_flags = next_target.S;
				break;
			#endif
			// M112- immediate stop
			case 112:
				timer_stop();
				queue_flush();
				power_off();
				break;
				// M113- extruder PWM
			// M114- report XYZEF to host
			case 114:
				sersendf_P(PSTR("X:%lq,Y:%lq,Z:%lq,E:%lq,F:%ld"), current_position.X * ((int32_t) UM_PER_STEP_X), current_position.Y * ((int32_t) UM_PER_STEP_Y), current_position.Z * ((int32_t) UM_PER_STEP_Z), current_position.E * ((int32_t) UM_PER_STEP_E), current_position.F);
				// newline is sent from gcode_parse after we return
				break;
			// M115- capabilities string
			case 115:
				sersendf_P(PSTR("FIRMWARE_NAME:Teacup FIRMWARE_URL:http%%3A//github.com/triffid/Teacup_Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:%d TEMP_SENSOR_COUNT:%d HEATER_COUNT:%d"), 1, NUM_TEMP_SENSORS, NUM_HEATERS);
				// newline is sent from gcode_parse after we return
				break;
			// M116 - Wait for all temperatures and other slowly-changing variables to arrive at their set values.
			case 116:
				enqueue(NULL);
				break;
			// M130- heater P factor
			case 130:
				if (next_target.seen_S)
					pid_set_p(next_target.P, next_target.S);
				break;
			// M131- heater I factor
			case 131:
				if (next_target.seen_S)
					pid_set_i(next_target.P, next_target.S);
				break;
			// M132- heater D factor
			case 132:
				if (next_target.seen_S)
					pid_set_d(next_target.P, next_target.S);
				break;
			// M133- heater I limit
			case 133:
				if (next_target.seen_S)
					pid_set_i_limit(next_target.P, next_target.S);
				break;
			// M134- save PID settings to eeprom
			case 134:
				heater_save_settings();
				break;
			// M135- set heater output
			case 135:
				if (next_target.seen_S) {
					heater_set(next_target.P, next_target.S);
					power_on();
				}
				break;
			#ifdef	DEBUG
			// M136- PRINT PID settings to host
			case 136:
				heater_print(next_target.P);
				break;
			#endif

			case 140: //Set heated bed temperature
				#ifdef	HEATER_BED
					temp_set(HEATER_BED, next_target.S);
					if (next_target.S)
						power_on();
				#endif
				break;

			// M190- power on
			case 190:
				power_on();
				x_enable();
				y_enable();
				z_enable();
				e_enable();
				steptimeout = 0;
				break;
			// M191- power off
			case 191:
				x_disable();
				y_disable();
				z_disable();
				e_disable();
				power_off();
				break;

			#ifdef	DEBUG
			// M240- echo off
			case 240:
				debug_flags &= ~DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo off"));
				// newline is sent from gcode_parse after we return
				break;
				// M241- echo on
			case 241:
				debug_flags |= DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo on"));
				// newline is sent from gcode_parse after we return
				break;

			// DEBUG: return current position, end position, queue
			case 250:
				sersendf_P(PSTR("{X:%ld,Y:%ld,Z:%ld,E:%ld,F:%lu,c:%lu}\t{X:%ld,Y:%ld,Z:%ld,E:%ld,F:%lu,c:%lu}\t"), current_position.X, current_position.Y, current_position.Z, current_position.E, current_position.F, movebuffer[mb_tail].c, movebuffer[mb_tail].endpoint.X, movebuffer[mb_tail].endpoint.Y, movebuffer[mb_tail].endpoint.Z, movebuffer[mb_tail].endpoint.E, movebuffer[mb_tail].endpoint.F,
					#ifdef ACCELERATION_REPRAP
						movebuffer[mb_tail].end_c
					#else
						movebuffer[mb_tail].c
					#endif
					);

				print_queue();
				break;

			// DEBUG: read arbitrary memory location
			case 253:
				if (next_target.seen_P == 0)
					next_target.P = 1;
				for (; next_target.P; next_target.P--) {
					serwrite_hex8(*(volatile uint8_t *)(next_target.S));
					next_target.S++;
				}
				// newline is sent from gcode_parse after we return
				break;

			// DEBUG: write arbitrary memory locatiom
			case 254:
				sersendf_P(PSTR("%x:%x->%x"), next_target.S, *(volatile uint8_t *)(next_target.S), next_target.P);
				(*(volatile uint8_t *)(next_target.S)) = next_target.P;
				// newline is sent from gcode_parse after we return
				break;
			#endif /* DEBUG */
				// unknown mcode: spit an error
			default:
				sersendf_P(PSTR("E: Bad M-code %d"), next_target.M);
				// newline is sent from gcode_parse after we return
		} // switch (next_target.M)
	} // else if (next_target.seen_M)
} // process_gcode_command()
