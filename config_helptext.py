helpText = {
'STEPS_PER_M': "steps per meter ( = steps per mm * 1000 ) \
calculate these values appropriate for your machine.\n\
for threaded rods, this is:\n\n\
\t(steps motor per turn) / (pitch of the thread) * 1000\n\n\
for belts, this is\n\n\
\t(steps per motor turn) / (number of gear teeth) / (belt module) * 1000\n\n\
half-stepping doubles the number, quarter stepping requires * 4, etc.\n\
valid range = 20 to 4,0960,000 (0.02 to 40960 steps/mm). \
all numbers are integers, so no decimal point",

'MAXIMUM_FEEDRATE': "maximum feed rate - in mm/min - for G0 rapid moves and \
as a cap for all other feedrates",

'SEARCH_FEEDRATE': "search feed rate - in mm/min - used when doing precision \
endstop search and as a default feed rate",

'ENDSTOP_CLEARANCE': "When hitting an endstop, Teacup properly decelerates \
instead of doing an abrupt stop\nto save your mechanics. Inevitably, this \
means it overshoots the endstop trigger point by some distance.\n\n\
To deal with this, Teacup adapts homing movement speeds to what your endstops \
can deal with.\nThe higher the allowed acceleration and the more clearance \
the endstop comes with, the faster Teacup\nwill do homing movements.\n\n\
Set here how many micrometers (mm * 1000) your endstop allows the carriage to \
overshoot the\ntrigger point. Typically 1000 or 2000 for mechanical endstops, \
more for optical ones.\nYou can set it to zero, in which case \
SEARCH_FEEDRATE_{XYZ} is used, but expect very slow\nhoming movements.\n\n\
  Units: micrometers\n\
  Sane values: 0 to 20000   (0 to 20 mm)\n\
  Valid range: 0 to 1000000",

'MINMAX': "soft axis limits, in mm.\n\ndefine them to your machine's size \
relative to what your host considers to be the origin.",

'E_ABSOLUTE': "some G-Code creators produce relative length commands for the \
extruder,\nothers absolute ones. G-Code using absolute lengths can be \
recognized when there\nare G92 E0 commands from time to time. if you have \
G92 E0 in your G-code, check this box.",

'ACCELERATION_REPRAP': "acceleration, reprap style.\n\n\
Each movement starts at the speed of the previous command and accelerates or \
decelerates\nlinearly to reach target speed at the end of the movement.",
'ACCELERATION_RAMPING': "acceleration and deceleration ramping.\n\n\
Each movement starts at (almost) no speed, linearly accelerates to target \
speed and decelerates\njust in time to smoothly stop at the target.",
'ACCELERATION_TEMPORAL': "This algorithm causes the timer to fire when any \
axis needs to step, instead of\n\synchronising to the axis with the most \
steps ala bresenham",
'ACCELERATION' : "how fast to accelerate when using acceleration ramping.\n\n\
given in mm/s^2, decimal allowed, useful range 1. to 10,000.\n\
Start with 10. for milling (high precision) or 1000. for printing",
'LOOKAHEAD': "Define this to enable look-ahead during *ramping* acceleration to \
smoothly transition\nbetween moves instead of performing a dead stop every \
move. Enabling look-ahead requires about\n3600 bytes of flash memory.",
'MAX_JERK': "When performing look-ahead, we need to decide what an acceptable \
jerk to the\nmechanics is. Look-ahead attempts to instantly change direction \
at movement\ncrossings, which means instant changes in the speed of the axes \
participating\nin the movement. Define here how big the speed bumps on each \
of the axes is\nallowed to be.\n\n\
If you want a full stop before and after moving a specific axis, define\n\
maximum jerk of this axis to 0. This is often wanted for the Z axis. If you \
want\nto ignore jerk on an axis, define it to twice the maximum feedrate of \
this axis.\n\nHaving these values too low results in more than necessary \
slowdown at\nmovement crossings, but is otherwise harmless. Too high values \
can result\nin stepper motors suddenly stalling. If angles between movements \
in your\nG-code are small and your printer runs through entire curves full \
speed,\nthere's no point in raising the values.\n\n\
  Units: mm/min\n\
  Sane values: 0 to 400\n\
  Valid range: 0 to 65535",

'INCLUDE_ARDUINO': "Include arduino.h header file in the C++ source code. \
This allows you\nto define pins using the atmel/avr conventions (e.g. DI012 \
or AI01)",

'USE_INTERNAL_PULLUPS': "Use Internal Pullups. the ATmega has internal pullup \
resistors on it's input pins\nwhich are counterproductive with the commonly \
used electronic endstops, so this should be unchecked.\n\For other endstops, \
like mechanical ones, you may want to check this.",

'TX_ENABLE_PIN': "Tx Enable Pin. Not yet used",
'RX_ENABLE_PIN': "Rx Enable Pin. Not yet used",

'X_STEP_PIN': "the pin for the stepper motor step signal",
'X_DIR_PIN': "the pin for the stepper motor direction signal",
'X_MIN_PIN': "the pin for the endstop at the axis minimum position",
'X_MAX_PIN': "the pin for the endstop at the axis maximum position",
'X_ENABLE_PIN': "the pin for the stepper motor enable signal",
'X_INVERT_DIR': "true or false - invert the stepper motor direction",
'X_INVERT_MIN': "true or false - invert the signal received from the minimum endstop",
'X_INVERT_MAX': "true or false - invert the signal received from the maximum endstop",
'X_INVERT_ENABLE': "true or false - invert the stepper motor enable signal",

'Y_STEP_PIN': "the pin for the stepper motor step signal",
'Y_DIR_PIN': "the pin for the stepper motor direction signal",
'Y_MIN_PIN': "the pin for the endstop at the axis minimum position",
'Y_MAX_PIN': "the pin for the endstop at the axis maximum position",
'Y_ENABLE_PIN': "the pin for the stepper motor enable signal",
'Y_INVERT_DIR': "true or false - invert the stepper motor direction",
'Y_INVERT_MIN': "true or false - invert the signal received from the minimum endstop",
'Y_INVERT_MAX': "true or false - invert the signal received from the maximum endstop",
'Y_INVERT_ENABLE': "true or false - invert the stepper motor enable signal",

'Z_STEP_PIN': "the pin for the stepper motor step signal",
'Z_DIR_PIN': "the pin for the stepper motor direction signal",
'Z_MIN_PIN': "the pin for the endstop at the axis minimum position",
'Z_MAX_PIN': "the pin for the endstop at the axis maximum position",
'Z_ENABLE_PIN': "the pin for the stepper motor enable signal",
'Z_INVERT_DIR': "true or false - invert the stepper motor direction",
'Z_INVERT_MIN': "true or false - invert the signal received from the minimum endstop",
'Z_INVERT_MAX': "true or false - invert the signal received from the maximum endstop",
'Z_INVERT_ENABLE': "true or false - invert the stepper motor enable signal",

'E_STEP_PIN': "the pin for the stepper motor step signal",
'E_DIR_PIN': "the pin for the stepper motor direction signal",
'E_ENABLE_PIN': "the pin for the stepper motor enable signal",
'E_INVERT_DIR': "true or false - invert the stepper motor direction",
'E_INVERT_ENABLE': "true or false - invert the stepper motor enable signal",

'PS_ON_PIN': "Which pin is used to turn the power supply off",
'PS_MOSFET_PIN': "???",

'STEPPER_ENABLE_PIN': "???",
'STEPPER_INVERT_ENABLE': "???",

'SD_CARD_DETECT': "the pin used to detect if an SD card has been inserted.",
'SD_WRITE_PROTECT': "the pin used to determine if an SD card is write-protected",
'DEBUG_LED_PIN': "Enable flashing of a LED during motor stepping.\n\n\
Disabled by default. Turning this on this makes the binary a few bytes larger\n\
 and adds a few cycles to the step timing interrupt. This is also used for\n\
 for precision profiling",

'TEMP_HYSTERESIS': "Actual temperature must be target +/- this hysteresis \
before target\ntemperature is considered to be achieved. Also, BANG_BANG \
tries to stay\nwithin half of this hysteresis. Unit is degrees Celcius",
'TEMP_RESIDENCY_TIME': "actual temperature must be close to target (within\n\
set temperature +- TEMP_HYSTERESIS) for this long before target is achieved\n\
(and a M116 succeeds). Unit is seconds.",
'TEMP_EWMA': "Smooth noisy temperature sensors. Good hardware shouldn't be\n\
noisy. Set to 1.0 for unfiltered data (and a 140 bytes smaller binary)\n\n\
Valid range: 0.001 - 1.0",
'TEMP_TYPES': "which temperature sensors are you using? Check every type of \
sensor you use\nto enable the appropriate code. Intercom is the gen3-style \
separate extruder board. Note\nthat you will not be able to uncheck a sensor \
type that is in use",

'ADDSENSOR': 'Define your temperature sensors. One entry for each sensor, \
only limited\nby the number of available ATmega pins.\n\n\
Types are only those chosen on the main page.\n\n\
The "additional" field is used for TT_THERMISTOR only. It defines the\n\
name of the table(s) in ThermistorTable.h to use. Typically, this is\n\
THERMISTOR_EXTRUDER for the first or only table, or THERMISTOR_BED for\n\
the second table. See also early in ThermistorTable.{single|double}.h,\n\n\
For a GEN3 set type to TT_INTERCOM and pin to AIO0. The pin won\'t be used in \
this case.',
'DELSENSOR': 'Remove the selected temperature sensor from the configuration',

"HEATER_SANITY_CHECK": "check if heater responds to changes in target \
temperature, disable\nand spit errors if not. largely untested, please \
comment in forum if this works, or doesn't work for you!",

'ADDHEATER': "Add a heater to the configuration",
'DELHEATER': "Remove a heater from the configuration",

'BAUD': "Baud rate for the serial RS232 protocol connection to the host. \
Usually 115200,\nother common values are 19200, 38400 or 57600. Ignored when \
USB_SERIAL is checked.",
'USB_SERIAL': "Define this for using USB instead of the serial RS232 protocol. \
Works on\nUSB-equipped ATmegas, like the ATmega32U4, only",
'XONXOFF': "Xon/Xoff flow control. Redundant when using RepRap Host for \
sending GCode,\nbut mandatory when sending GCode files with a plain terminal \
emulator, like GtkTerm (Linux),\nCoolTerm (Mac) or HyperTerminal (Windows). \
Can also be set in Makefile",

'MOTHERBOARD': "This is the motherboard, as opposed to the extruder.",
'F_CPU': "The CPU clock rate",
'EECONFIG': "Enable EEPROM configuration storage.\n\n\
Enabled by default. Commenting this out makes the binary several hundred\n\
bytes smaller, so you might want to disable EEPROM storage on small MCUs,\n\
like the ATmega168.",
'DEBUG': "enables extra output, and some extra M-codes.\n\n\
WARNING: this WILL break most host-side talkers that expect particular \
responses\nfrom firmware such as reprap host and replicatorG. Use with serial \
terminal\nor other suitable talker only.",
'BANG_BANG': "Drops PID loop from heater control, reduces code size\n\
significantly (1300 bytes!)",
'BANG_BANG_ON': "PWM value for 'on'",
'BANG_BANG_OFF': "PWM value for 'off'",
'MOVEBUFFER_SIZE': "Move buffer size, in number of moves\n\nnote that each \
move takes a fair chunk of ram (~69 bytes) so don't make the buffer too big.\n\
A bigger serial readbuffer may help more than increasing this unless your \
gcodes are more than\n70 characters long on average. However, a larger \
movebuffer will probably help with lots of \nshort consecutive moves, as each \
move takes a bunch of math (hence time) to set up, so a longer\nbuffer allows \
more of the math to be done during preceding longer moves",
'DC_EXTRUDER': 'If you have a DC motor extruder, configure it as a "heater" \
on the heater page,\nand choose the name you used there in this field. You \
probably also want to comment out E_STEP_PIN\nand E_DIR_PIN on the Pinouts \
page.',
'DC_EXTRUDER_PWM': "The PWM value at which to operate the DC Extruder motor",
'USE_WATCHDOG': "Teacup implements a watchdog, which has to be reset every \
250ms or it will\nreboot the controller. As rebooting (and letting the GCode\
sending application trying to\ncontinue the build with a then different Home \
point) is probably even worse than just hanging,\nand there is no better \
restore code in place, this is disabled for now.",
'REFERENCE': "which analog reference to use. see analog.h for choices",
'STEP_INTERRUPT_INTERRUPTIBLE': "this option makes the step interrupt \
interruptible (nested).\nthis should help immensely with dropped serial \
characters, but may also make\ndebugging infuriating due to the complexities \
arising from nested interrupts.\n\nnote disable this option if you're using a \
'168 or for some reason your ram usage\nis above 90%. This option hugely \
increases likelihood of stack smashing.",
'TH_COUNT': "Temperature history count. This is how many temperature readings \
to keep in\norder to calculate derivative in PID loop. Higher values make PID \
derivative term more\nstable at the expense of reaction time",
'FAST_PWM': "Teacup offers two PWM frequencies, 76(61) Hz and 78000(62500) \
Hz on a\n20(16) MHz electronics. The slower one is the default, as it's the \
safer choice.\nThe drawback is in a quiet environment you might notice the \
heaters and power supply humming.\n\nUncomment this option if you want to get \
rid of this humming or want faster PWM for other reasons.",
'ENDSTOP_STEPS': "Number of steps to run into the endstops intentionally\n\n\
As Endstops trigger false alarm sometimes, Teacup debounces them by counting \
a number of\nconsecutive positives. Valid range is 1...255. Use 4 or less for \
reliable endstops, 8 or\neven more for flaky ones.",
'CANNED_CYCLE': "G-code commands in this string will be executed over and \
over again, without\nuser interaction or even a serial connection. It's \
purpose is e.g. for\nexhibitions or when using Teacup for other purposes than \
printing. You can\nadd any G-code supported by Teacup.",
}
