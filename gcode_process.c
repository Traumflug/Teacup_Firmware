#include	"gcode_process.h"

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

/****************************************************************************
*                                                                           *
* Command Received - process it                                             *
*                                                                           *
****************************************************************************/

void process_gcode_command() {
	uint32_t	backup_f;
	
	// convert relative to absolute
	if (next_target.option_relative) {
		next_target.target.X += startpoint.X;
		next_target.target.Y += startpoint.Y;
		next_target.target.Z += startpoint.Z;
	}
	// E ALWAYS relative, otherwise we overflow our registers after only a few layers
	// 	next_target.target.E += startpoint.E;
	// easier way to do this
	// 	startpoint.E = 0;
	// moved to dda.c, end of dda_create() and dda_queue.c, next_move()
	
	if (next_target.seen_G) {
		switch (next_target.G) {
			// 	G0 - rapid, unsynchronised motion
			// since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
			case 0:
				backup_f = next_target.target.F;
				next_target.target.F = MAXIMUM_FEEDRATE_X * 2;
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
				for (;queue_empty() == 0;)
					wd_reset();
				// delay
				delay_ms(next_target.P);
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
				/*
				Home XY first
				*/
				// hit endstops, no acceleration- we don't care about skipped steps
				startpoint.F = MAXIMUM_FEEDRATE_X;
				SpecialMoveXY(-250L * STEPS_PER_MM_X, -250L * STEPS_PER_MM_Y, MAXIMUM_FEEDRATE_X);
				startpoint.X = startpoint.Y = 0;
				
				// move forward a bit
				SpecialMoveXY(5 * STEPS_PER_MM_X, 5 * STEPS_PER_MM_Y, SEARCH_FEEDRATE_X);
				
				// move back in to endstops slowly
				SpecialMoveXY(-20 * STEPS_PER_MM_X, -20 * STEPS_PER_MM_Y, SEARCH_FEEDRATE_X);
				
				// wait for queue to complete
				for (;queue_empty() == 0;)
					wd_reset();
				
				// this is our home point
				startpoint.X = startpoint.Y = current_position.X = current_position.Y = 0;
				
				/*
				Home Z
				*/
				// hit endstop, no acceleration- we don't care about skipped steps
				startpoint.F = MAXIMUM_FEEDRATE_Z;
				SpecialMoveZ(-250L * STEPS_PER_MM_Z, MAXIMUM_FEEDRATE_Z);
				startpoint.Z = 0;
				
				// move forward a bit
				SpecialMoveZ(5 * STEPS_PER_MM_Z, SEARCH_FEEDRATE_Z);
				
				// move back into endstop slowly
				SpecialMoveZ(-20L * STEPS_PER_MM_Z, SEARCH_FEEDRATE_Z);
				
				// wait for queue to complete
				for (;queue_empty() == 0;)
					wd_reset();
				
				// this is our home point
				startpoint.Z = current_position.Z = 0;
				
				/*
				Home E
				*/
				// extruder only runs one way and we have no "endstop", just set this point as home
				startpoint.E = current_position.E = 0;
				
				/*
				Home F
				*/
				
				// F has been left at SEARCH_FEEDRATE_Z by the last move, this is a usable "home"
				// uncomment the following or substitute if you prefer a different default feedrate
				// startpoint.F = SEARCH_FEEDRATE_Z
				
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
					startpoint.X = startpoint.Y = startpoint.Z = startpoint.E =
					current_position.X = current_position.Y = current_position.Z = current_position.E = 0;
					startpoint.F =
					current_position.F = SEARCH_FEEDRATE_Z;
					break;
					
					// unknown gcode: spit an error
				default:
					sersendf_P(PSTR("E: Bad G-code %d"), next_target.G);
					// newline is sent from gcode_parse after we return
		}
	}
	else if (next_target.seen_M) {
		switch (next_target.M) {
			// M101- extruder on
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
				
				// M103- extruder off
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
				if (next_target.S) {
					power_on();
				}
				else {
					disable_heater();
				}
				break;
				
				// M105- get temperature
			case 105:
				temp_print(next_target.P);
				break;
				
				// M106- fan on
			#if NUM_HEATERS > 1
			case 106:
				heater_set(1, 255);
				break;
				// M107- fan off
			case 107:
				heater_set(1, 0);
				break;
			#endif
				
				// M109- set temp and wait
			case 109:
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
				next_target.N_expected = next_target.S - 1;
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
				sersendf_P(PSTR("X:%ld,Y:%ld,Z:%ld,E:%ld,F:%ld"), current_position.X, current_position.Y, current_position.Z, current_position.E, current_position.F);
				// newline is sent from gcode_parse after we return
				break;
				// M115- capabilities string
			case 115:
				sersendf_P(PSTR("FIRMWARE_NAME:FiveD_on_Arduino FIRMWARE_URL:http%3A//github.com/triffid/FiveD_on_Arduino/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:%d TEMP_SENSOR_COUNT:%d HEATER_COUNT:%d"), 1, NUM_TEMP_SENSORS, NUM_HEATERS);
				// newline is sent from gcode_parse after we return
				break;

			#if	NUM_HEATERS > 0
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
				if (next_target.seen_S)
					heater_set(next_target.P, next_target.S);
				break;
			#endif	/* NUM_HEATERS > 0 */
				
				// M190- power on
			case 190:
				power_on();
				x_enable();
				y_enable();
				z_enable();
				steptimeout = 0;
				break;
				// M191- power off
			case 191:
				x_disable();
				y_disable();
				z_disable();
				power_off();
				break;
				
			#ifdef	DEBUG
				// M140- echo off
			case 140:
				debug_flags &= ~DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo off"));
				// newline is sent from gcode_parse after we return
				break;
				// M141- echo on
			case 141:
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
		}
	}
}
