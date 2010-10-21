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
					serial_writestr_P(PSTR("E: Bad G-code "));
					serwrite_uint8(next_target.G);
					serial_writechar('\n');
		}
	}
	else if (next_target.seen_M) {
		switch (next_target.M) {
			// M101- extruder on
			case 101:
				if (temp_achieved() == 0) {
					enqueue(NULL);
				}
				do {
					// backup feedrate, move E very quickly then restore feedrate
					backup_f = startpoint.F;
					startpoint.F = MAXIMUM_FEEDRATE_E;
					SpecialMoveE(E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
					startpoint.F = backup_f;
				} while (0);
				break;
				
				// M102- extruder reverse
				
				// M103- extruder off
			case 103:
				do {
					// backup feedrate, move E very quickly then restore feedrate
					backup_f = startpoint.F;
					startpoint.F = MAXIMUM_FEEDRATE_E;
					SpecialMoveE(-E_STARTSTOP_STEPS, MAXIMUM_FEEDRATE_E);
					startpoint.F = backup_f;
				} while (0);
				break;
				
				// M104- set temperature
			case 104:
				temp_set(next_target.S);
				if (next_target.S) {
					enable_heater();
					power_on();
				}
				else {
					disable_heater();
				}
				break;
				
				// M105- get temperature
			case 105:
				temp_print();
				break;
				
				// M106- fan on
				#ifdef	FAN_PIN
			case 106:
				enable_fan();
				break;
				// M107- fan off
			case 107:
				disable_fan();
				break;
				#endif
				
				// M109- set temp and wait
			case 109:
				temp_set(next_target.S);
				if (next_target.S) {
					enable_heater();
					power_on();
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
				disableTimerInterrupt();
				queue_flush();
				power_off();
				break;
				// M113- extruder PWM
				// M114- report XYZEF to host
			case 114:
				sersendf_P("X:%ld,Y:%ld,Z:%ld,E:%ld,F:%ld\n", current_position.X, current_position.Y, current_position.Z, current_position.E, current_position.F);
				break;
				// M115- capabilities string
			case 115:
				serial_writestr_P("FIRMWARE_NAME:FiveD_on_Arduino FIRMWARE_URL:http%3A//github.com/triffid/FiveD_on_Arduino/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:1 HEATER_COUNT:1\n");
				break;

				#ifdef	HEATER_PIN
				// M130- heater P factor
			case 130:
				if (next_target.seen_S)
					p_factor = next_target.S;
				break;
				// M131- heater I factor
			case 131:
				if (next_target.seen_S)
					i_factor = next_target.S;
				break;
				// M132- heater D factor
			case 132:
				if (next_target.seen_S)
					d_factor = next_target.S;
				break;
				// M133- heater I limit
			case 133:
				if (next_target.seen_S)
					i_limit = next_target.S;
				break;
				// M134- save PID settings to eeprom
			case 134:
				heater_save_settings();
				break;
				#endif	/* HEATER_PIN */
				
				// M190- power on
			case 190:
				power_on();
				#ifdef	X_ENABLE_PIN
				WRITE(X_ENABLE_PIN, 0);
				#endif
				#ifdef	Y_ENABLE_PIN
				WRITE(Y_ENABLE_PIN, 0);
				#endif
				#ifdef	Z_ENABLE_PIN
				WRITE(Z_ENABLE_PIN, 0);
				#endif
				steptimeout = 0;
				break;
				// M191- power off
			case 191:
				#ifdef	X_ENABLE_PIN
				WRITE(X_ENABLE_PIN, 1);
				#endif
				#ifdef	Y_ENABLE_PIN
				WRITE(Y_ENABLE_PIN, 1);
				#endif
				#ifdef	Z_ENABLE_PIN
				WRITE(Z_ENABLE_PIN, 1);
				#endif
				power_off();
				break;
				
				#ifdef	DEBUG
				// M140- echo off
			case 140:
				debug_flags &= ~DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo off\n"));
				break;
				// M141- echo on
			case 141:
				debug_flags |= DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo on\n"));
				break;
				
				// DEBUG: return current position
			case 250:
				serial_writestr_P(PSTR("{X:"));
				serwrite_int32(current_position.X);
				serial_writestr_P(PSTR(",Y:"));
				serwrite_int32(current_position.Y);
				serial_writestr_P(PSTR(",Z:"));
				serwrite_int32(current_position.Z);
				serial_writestr_P(PSTR(",E:"));
				serwrite_int32(current_position.E);
				serial_writestr_P(PSTR(",F:"));
				serwrite_int32(current_position.F);
				serial_writestr_P(PSTR(",c:"));
				serwrite_uint32(movebuffer[mb_tail].c);
				serial_writestr_P(PSTR("}\n"));
				
				serial_writestr_P(PSTR("{X:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.X);
				serial_writestr_P(PSTR(",Y:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.Y);
				serial_writestr_P(PSTR(",Z:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.Z);
				serial_writestr_P(PSTR(",E:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.E);
				serial_writestr_P(PSTR(",F:"));
				serwrite_int32(movebuffer[mb_tail].endpoint.F);
				serial_writestr_P(PSTR(",c:"));
				#ifdef ACCELERATION_REPRAP
				serwrite_uint32(movebuffer[mb_tail].end_c);
				#else
				serwrite_uint32(movebuffer[mb_tail].c);
				#endif
				serial_writestr_P(PSTR("}\n"));
				
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
				serial_writechar('\n');
				break;
				
				// DEBUG: write arbitrary memory locatiom
			case 254:
				serwrite_hex8(next_target.S);
				serial_writechar(':');
				serwrite_hex8(*(volatile uint8_t *)(next_target.S));
				serial_writestr_P(PSTR("->"));
				serwrite_hex8(next_target.P);
				serial_writechar('\n');
				(*(volatile uint8_t *)(next_target.S)) = next_target.P;
				break;
				#endif /* DEBUG */
				// unknown mcode: spit an error
			default:
				serial_writestr_P(PSTR("E: Bad M-code "));
				serwrite_uint8(next_target.M);
				serial_writechar('\n');
		}
	}
}
