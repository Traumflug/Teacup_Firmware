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


	// The GCode documentation was taken from http://reprap.org/wiki/Gcode .

	if (next_target.seen_T) {
	    //? ==== T: Select Tool ====
	    //?
	    //? Example: T1
	    //?
	    //? Select extruder number 1 to build with.  Extruder numbering starts at 0.

	    next_tool = next_target.T;
	}

	if (next_target.seen_G) {
		uint8_t axisSelected = 0;
		switch (next_target.G) {
			// 	G0 - rapid, unsynchronised motion
			// since it would be a major hassle to force the dda to not synchronise, just provide a fast feedrate and hope it's close enough to what host expects
			case 0:
				//? ==== G0: Rapid move ====
				//?
				//? Example: G0 X12
				//?
				//? In this case move rapidly to X = 12 mm.  In fact, the RepRap firmware uses exactly the same code for rapid as it uses for controlled moves (see G1 below), as - for the RepRap machine - this is just as efficient as not doing so.  (The distinction comes from some old machine tools that used to move faster if the axes were not driven in a straight line.  For them G0 allowed any movement in space to get to the destination as fast as possible.)

				backup_f = next_target.target.F;
				next_target.target.F = MAXIMUM_FEEDRATE_X * 2L;
				enqueue(&next_target.target);
				next_target.target.F = backup_f;
				break;

				//	G1 - synchronised motion
			case 1:
				//? ==== G1: Controlled move ====
				//?
				//? Example: G1 X90.6 Y13.8 E22.4
				//?
				//? Go in a straight line from the current (X, Y) point to the point (90.6, 13.8), extruding material as the move happens from the current extruded length to a length of 22.4 mm.
				//?
				//? RepRap does subtle things with feedrates.  Thus:
				//?
				//? <pre>
				//? G1 F1500
				//? G1 X90.6 Y13.8 E22.4
				//? </pre>
				//?
				//? Will set a feedrate of 1500 mm/minute, then do the move described above at that feedrate.  But
				//?
				//? <pre>
				//? G1 F1500
				//? G1 X90.6 Y13.8 E22.4 F3000
				//? </pre>
				//?
				//? Will set a feedrate of 1500 mm/minute, then do the move described above accelerating to a feedrate of 3000 mm/minute as it does so.  The extrusion will accelerate along with the X, Y movement so everything stays synchronized.
				//?
				//? RepRap thus treats feedrate as simply another variable (like X, Y, Z, and E) to be linearly interpolated.  This gives complete control over accelerations and decelerations in a way that ensures that everything moves together and the right volume of material is extruded at all points.
				//?
				//? The first example shows how to get a constant-speed movement.  The second how to accelerate or decelerate.  Thus
				//?
				//? <pre>
				//? G1 F1500
				//? G1 X90.6 Y13.8 E22.4 F3000
				//? G1 X80 Y20 E36 F1500
				//? </pre>
				//?
				//? Will do the first movement accelerating as before, and the second decelerating from 3000 mm/minute back to 1500 mm/minute.
				//?
				//? To reverse the extruder by a given amount (for example to reduce its internal pressure while it does an in-air movement so that it doesn't dribble) simply use G1 to send an E value that is less than the currently extruded length.

				enqueue(&next_target.target);
				break;

				//	G2 - Arc Clockwise
				// unimplemented

				//	G3 - Arc Counter-clockwise
				// unimplemented

				//	G4 - Dwell
			case 4:
				//? ==== G4: Dwell ====
				//?
				//? Example: G4 P200
				//?
				//? In this case sit still doing nothing for 200 milliseconds.  During delays the state of the machine (for example the temperatures of its extruders) will still be preserved and controlled.
				//?

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
				//? ==== G20: Set Units to Inches ====
				//?
				//? Example: G20
				//?
				//? Units from now on are in inches.
				//?
				next_target.option_inches = 1;
				break;

				//	G21 - mm as units
			case 21:
				//? ==== G21: Set Units to Millimeters ====
				//?
				//? Example: G21
				//?
				//? Units from now on are in millimeters.  (This is the RepRap default.)
				//?
				next_target.option_inches = 0;
				break;

				//	G30 - go home via point
			case 30:
				//? ==== G30: Go home via point ====
				//?
				//? Undocumented.
				enqueue(&next_target.target);
				// no break here, G30 is move and then go home

				//	G28 - go home
			case 28:
				//? ==== G28: Move to Origin ====
				//?
				//? Example: G28
				//?
				//? This causes the RepRap machine to move back to its X, Y and Z zero endstops.  It does so
				//? accelerating, so as to get there fast.  But when it arrives it backs off by 1 mm in each
				//? direction slowly, then moves back slowly to the stop.  This ensures more accurate positioning.
				//?
				//? If you add coordinates, then just the axes with coordinates specified will be zeroed.  Thus
				//?
				//? G28 X0 Y72.3
				//?
				//? will zero the X and Y axes, but not Z.  The actual coordinate values are ignored.
				//?

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
				//? ==== G90: Set to Absolute Positioning ====
				//?
				//? Example: G90
				//?
				//? All coordinates from now on are absolute relative to the origin of the machine.  (This is the RepRap default.)
				next_target.option_relative = 0;
				break;

				//	G91 - relative positioning
			case 91:
				//? ==== G91: Set to Relative Positioning ====
				//?
				//? Example: G91
				//?
				//? All coordinates from now on are relative to the last position.
				next_target.option_relative = 1;
				break;

				//	G92 - set home
			case 92:
				//? ==== G92: Set Position ====
				//?
				//? Example: G92 X10 E90
				//?
				//? Allows programming of absolute zero point, by reseting the current position to the values specified.  This would set the machine's X coordinate to 10, and the extrude coordinate to 90. No physical motion will occur.

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
				//? ==== G161: Home negative ====
				//?
				//? Find the minimum limit of the specified axes by searching for the limit switch.
				if (next_target.seen_X)
					home_x_negative();
				if (next_target.seen_Y)
					home_y_negative();
				if (next_target.seen_Z)
					home_z_negative();
				break;
			// G162 - Home positive
			case 162:
				//? ==== G161: Home positive ====
				//?
				//? Find the maximum limit of the specified axes by searching for the limit switch.
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
			if (DEBUG_POSITION && (debug_flags & DEBUG_POSITION))
				print_queue();
		#endif
	}
	else if (next_target.seen_M) {
		switch (next_target.M) {
			// M2- program end
			case 2:
				//? ==== M2: program end ====
				//?
				//? Undocumented.
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
				//? ==== M6: tool change ====
				//?
				//? Undocumented.
				tool = next_tool;
				break;
			// M3/M101- extruder on
			case 3:
			case 101:
				//? ==== M101: extruder on ====
				//?
				//? Undocumented.
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
				//? ==== M103: extruder off ====
				//?
				//? Undocumented.
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
				//? ==== M104: Set Extruder Temperature (Fast) ====
				//?
				//? Example: M104 S190
				//?
				//? Set the temperature of the current extruder to 190<sup>o</sup>C and return control to the host immediately (''i.e.'' before that temperature has been reached by the extruder).  See also M109.
				temp_set(next_target.P, next_target.S);
				if (next_target.S)
					power_on();
				break;

			// M105- get temperature
			case 105:
				//? ==== M105: Get Extruder Temperature ====
				//?
				//? Example: M105
				//?
				//? Request the temperature of the current extruder and the build base in degrees Celsius.  The temperatures are returned to the host computer.  For example, the line sent to the host in response to this command looks like
				//?
				//? <tt>ok T:201 B:117</tt>
				//?
				temp_print(next_target.P);
				break;

			// M7/M106- fan on
			case 7:
			case 106:
				//? ==== M106: Fan On ====
				//?
				//? Example: M106
				//?
				//? Turn on the cooling fan (if any).

				#ifdef HEATER_FAN
					heater_set(HEATER_FAN, 255);
				#endif
				break;
			// M107- fan off
			case 9:
			case 107:
				//? ==== M107: Fan Off ====
				//?
				//? Example: M107
				//?
				//? Turn off the cooling fan (if any).

				#ifdef HEATER_FAN
					heater_set(HEATER_FAN, 0);
				#endif
				break;

			// M109- set temp and wait
			case 109:
				//? ==== M109: Set Extruder Temperature ====
				//?
				//? Example: M109 S190
				//?
				//? Set the temperature of the current extruder to 190<sup>o</sup>C and wait for it to reach that value before sending an acknowledgment to the host.  In fact the RepRap firmware waits a while after the temperature has been reached for the extruder to stabilise - typically about 40 seconds.  This can be changed by a parameter in the firmware configuration file when the firmware is compiled.  See also M104 and M116.
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
				//? ==== M110: Set Current Line Number ====
				//?
				//? Example: N123 M110
				//?
				//? Set the current line number to 123.  Thus the expected next line after this command will be 124.
				//? This is a no-op in Teacup.
				break;
			// M111- set debug level
			#ifdef	DEBUG
			case 111:
				//? ==== M111: Set Debug Level ====
				//?
				//? Example: M111 S6
				//?
				//? Set the level of debugging information transmitted back to the host to level 6.  The level is the OR of three bits:
				//?
				//? <Pre>
				//? #define DEBUG_ECHO (1<<0)
				//? #define DEBUG_INFO (1<<1)
				//? #define DEBUG_ERRORS (1<<2)
				//? </pre>
				//?
				//? Thus 6 means send information and errors, but don't echo commands.  (This is the RepRap default.)
				//? This command is only available in DEBUG builds of Teacup.

				debug_flags = next_target.S;
				break;
			#endif
			// M112- immediate stop
			case 112:
				//? ==== M112: Emergency Stop ====
				//?
				//? Example: M112
				//?
				//? Any moves in progress are immediately terminated, then RepRap shuts down.  All motors and heaters are turned off.
				//? It can be started again by pressing the reset button on the master microcontroller.  See also M0.

				timer_stop();
				queue_flush();
				power_off();
				break;
				// M113- extruder PWM
			// M114- report XYZEF to host
			case 114:
				//? ==== M114: Get Current Position ====
				//?
				//? Example: M114
				//?
				//? This causes the RepRap machine to report its current X, Y, Z and E coordinates to the host.
				//?
				//? For example, the machine returns a string such as:
				//?
				//? <tt>ok C: X:0.00 Y:0.00 Z:0.00 E:0.00</tt>
				sersendf_P(PSTR("X:%lq,Y:%lq,Z:%lq,E:%lq,F:%ld"), current_position.X * ((int32_t) UM_PER_STEP_X), current_position.Y * ((int32_t) UM_PER_STEP_Y), current_position.Z * ((int32_t) UM_PER_STEP_Z), current_position.E * ((int32_t) UM_PER_STEP_E), current_position.F);
				// newline is sent from gcode_parse after we return
				break;
			// M115- capabilities string
			case 115:
				//? ==== M115: Get Firmware Version and Capabilities ====
				//?
				//? Example: M115
				//?
				//? Request the Firmware Version and Capabilities of the current microcontroller
				//? The details are returned to the host computer as key:value pairs separated by spaces and terminated with a linefeed.
				//?
				//? sample data from firmware:
				//?  ok PROTOCOL_VERSION:0.1 FIRMWARE_NAME:FiveD FIRMWARE_URL:http%3A//? reprap.org MACHINE_TYPE:Mendel EXTRUDER_COUNT:1

				sersendf_P(PSTR("FIRMWARE_NAME:Teacup FIRMWARE_URL:http%%3A//github.com/triffid/Teacup_Firmware/ PROTOCOL_VERSION:1.0 MACHINE_TYPE:Mendel EXTRUDER_COUNT:%d TEMP_SENSOR_COUNT:%d HEATER_COUNT:%d"), 1, NUM_TEMP_SENSORS, NUM_HEATERS);
				// newline is sent from gcode_parse after we return
				break;
			// M116 - Wait for all temperatures and other slowly-changing variables to arrive at their set values.
			case 116:
				//? ==== M116: Wait ====
				//?
				//? Example: M116
				//?
				//? Wait for ''all'' temperatures and other slowly-changing variables to arrive at their set values.  See also M109.

				enqueue(NULL);
				break;
			// M130- heater P factor
			case 130:
				//? ==== M130: heater P factor ====
				//? Undocumented.
				if (next_target.seen_S)
					pid_set_p(next_target.P, next_target.S);
				break;
			// M131- heater I factor
			case 131:
				//? ==== M131: heater I factor ====
				//? Undocumented.
				if (next_target.seen_S)
					pid_set_i(next_target.P, next_target.S);
				break;
			// M132- heater D factor
			case 132:
				//? ==== M132: heater D factor ====
				//? Undocumented.
				if (next_target.seen_S)
					pid_set_d(next_target.P, next_target.S);
				break;
			// M133- heater I limit
			case 133:
				//? ==== M133: heater I limit ====
				//? Undocumented.
				if (next_target.seen_S)
					pid_set_i_limit(next_target.P, next_target.S);
				break;
			// M134- save PID settings to eeprom
			case 134:
				//? ==== M134: save PID settings to eeprom ====
				//? Undocumented.
				heater_save_settings();
				break;
			// M135- set heater output
			case 135:
				//? ==== M135: set heater output ====
				//? Undocumented.
				if (next_target.seen_S) {
					heater_set(next_target.P, next_target.S);
					power_on();
				}
				break;
			#ifdef	DEBUG
			// M136- PRINT PID settings to host
			case 136:
				//? ==== M136: PRINT PID settings to host ====
				//? Undocumented.
				//? This comand is only available in DEBUG builds.
				heater_print(next_target.P);
				break;
			#endif

			case 140: //Set heated bed temperature
				//? ==== M140: Set heated bed temperature ====
				//? Undocumented.
				#ifdef	HEATER_BED
					temp_set(HEATER_BED, next_target.S);
					if (next_target.S)
						power_on();
				#endif
				break;

			// M190- power on
			case 190:
				//? ==== M190: Power On ====
				//? Undocumented.
				power_on();
				x_enable();
				y_enable();
				z_enable();
				e_enable();
				steptimeout = 0;
				break;
			// M191- power off
			case 191:
				//? ==== M191: Power Off ====
				//? Undocumented.
				x_disable();
				y_disable();
				z_disable();
				e_disable();
				power_off();
				break;

                        // M200 - report endstop status
                        case 200:
				//? ==== M200: report endstop status ====
				//? Report the current status of the endstops configured in the firmware to the host.
                                #if defined(X_MIN_PIN)
				    sersendf_P(PSTR("x_min:%d "), x_min());
                                #endif
                                #if defined(X_MAX_PIN)
				    sersendf_P(PSTR("x_max:%d "), x_max());
                                #endif
                                #if defined(Y_MIN_PIN)
				    sersendf_P(PSTR("y_min:%d "), y_min());
                                #endif
                                #if defined(Y_MAX_PIN)
				    sersendf_P(PSTR("y_max:%d "), y_max());
                                #endif
                                #if defined(Z_MIN_PIN)
				    sersendf_P(PSTR("z_min:%d "), z_min());
                                #endif
                                #if defined(Z_MAX_PIN)
				    sersendf_P(PSTR("z_max:%d "), z_max());
                                #endif
                                #if !(defined(X_MIN_PIN) || defined(X_MAX_PIN) || defined(Y_MIN_PIN) || defined(Y_MAX_PIN) || defined(Z_MIN_PIN) || defined(Z_MAX_PIN))
				    sersendf_P(PSTR("no endstops defined"));
                                #endif
				break;

			#ifdef	DEBUG
			// M240- echo off
			case 240:
				//? ==== M240: echo off ====
				//? Disable echo.
				//? This command is only available in DEBUG builds.
				debug_flags &= ~DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo off"));
				// newline is sent from gcode_parse after we return
				break;
				// M241- echo on
			case 241:
				//? ==== M241: echo on ====
				//? Enable echo.
				//? This command is only available in DEBUG builds.
				debug_flags |= DEBUG_ECHO;
				serial_writestr_P(PSTR("Echo on"));
				// newline is sent from gcode_parse after we return
				break;

			// DEBUG: return current position, end position, queue
			case 250:
				//? ==== M250: return current position, end position, queue ====
				//? Undocumented
				//? This command is only available in DEBUG builds.
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
				//? ==== M253: read arbitrary memory location ====
				//? Undocumented
				//? This command is only available in DEBUG builds.
				if (next_target.seen_P == 0)
					next_target.P = 1;
				for (; next_target.P; next_target.P--) {
					serwrite_hex8(*(volatile uint8_t *)(next_target.S));
					next_target.S++;
				}
				// newline is sent from gcode_parse after we return
				break;

			// DEBUG: write arbitrary memory location
			case 254:
				//? ==== M254: write arbitrary memory location ====
				//? Undocumented
				//? This command is only available in DEBUG builds.
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
