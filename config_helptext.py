helpText = {
'SPM': "steps per meter ( = steps per mm * 1000 ) \
calculate these values appropriate for your machine.\n\
for threaded rods, this is:\n\n\
\t(steps motor per turn) / (pitch of the thread) * 1000\n\n\
for belts, this is\n\n\
\t(steps per motor turn) / (number of gear teeth) / (belt module) * 1000\n\n\
half-stepping doubles the number, quarter stepping requires * 4, etc.\n\
valid range = 20 to 4,0960,000 (0.02 to 40960 steps/mm).  \
all numbers are integers, so no decimal point",
'SMX': "steps per meter for the X axis",
'SMY': "steps per meter for the Y axis",
'SMZ': "steps per meter for the Z axis",
'SME': "steps per meter for the E axis",

'MFR': "maximum feed rate - in mm/min - for G0 rapid moves and as a cap for \
all other feedrates",
'MFRX': "maximum feed rate for the X axis (mm/min)",
'MFRY': "maximum feed rate for the Y axis (mm/min)",
'MFRZ': "maximum feed rate for the Z axis (mm/min)",
'MFRE': "maximum feed rate for the E axis (mm/min)",

'MSR': "search feed rate - in mm/min - used when doing precision endstop \
search and as a default feed rate",
'MSRX': "search feed rate for the X axis (mm/min)",
'MSRY': "search feed rate for the Y axis (mm/min)",
'MSRZ': "search feed rate for the Z axis (mm/min)",

'ECL': "When hitting an endstop, Teacup properly decelerates instead of \
doing an abrupt stop\nto save your mechanics. Ineviteably, this means it \
overshoots the endstop trigger point by some distance.\n\n\
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
'ECX': "endstop clearance for the X axis (mm * 1000)",
'ECY': "endstop clearance for the Y axis (mm * 1000)",
'ECZ': "endstop clearance for the Z axis (mm * 1000)",

'MINMAX': "soft axis limits, in mm.\n\ndefine them to your machine's size \
relative to what your host considers to be the origin.",
'MINX': "Minimum limit for the X axis:",
'MAXX': "maximum limit for the X axis:",
'MINY': "minimum limit for the Y axis:",
'MAXY': "maximum limit for the Y axis:",
'MINZ': "minimum limit for the Z axis",
'MAXZ': "maximum limit for the Z axis",

'ABSE': "some G-code creators produce relative length commands for the \
extruder,\nothers absolute ones. G-code using absolute lengths can be \
recognized when there\nare G92 E0 commands from time to time. if you have \
G92 E0 in your G-code, check this box.",

'ACTYPE': "Acceleration algorithm",
'ACRR': "acceleration, reprap style.\n\n\
Each movement starts at the speed of the previous command and accelerates or \
decelerates\nlinearly to reach target speed at the end of the movement.",
'ACRP': "acceleration and deceleration ramping.\n\n\
Each movement starts at (almost) no speed, linearly accelerates to target \
speed and decelerates\njust in time to smoothly stop at the target.",
'ACTP': "This algorithm causes the timer to fire when any axis needs to step, \
instead of\nsynchronising to the axis with the most steps ala bresenham",
'ACCEL' : "how fast to accelerate when using acceleration ramping.\n\n\
given in mm/s^2, decimal allowed, useful range 1. to 10,000.\n\
Start with 10. for milling (high precision) or 1000. for printing",
'LKAH': "Define this to enable look-ahead during *ramping* acceleration to \
smoothly transition\nbetween moves instead of performing a dead stop every \
move.  Enabling look-ahead requires about\n3600 bytes of flash memory.",
'JERK': "When performing look-ahead, we need to decide what an acceptable \
jerk to the\nmechanics is. Look-ahead attempts to instantly change direction \
at movement\ncrossings, which means instant changes in the speed of the axes \
participating\nin the movement. Define here how big the speed bumps on each \
of the axes is\nallowed to be.\n\n\
If you want a full stop before and after moving a specific axis, define\n\
maximum jerk of this axis to 0. This is often wanted for the Z axis. If you want\n\
to ignore jerk on an axis, define it to twice the maximum feedrate of this axis.\n\n\
Having these values too low results in more than neccessary slowdown at\n\
movement crossings, but is otherwise harmless. Too high values can result\n\
in stepper motors suddenly stalling. If angles between movements in your\n\
G-code are small and your printer runs through entire curves full speed,\n\
there's no point in raising the values.\n\n\
  Units: mm/min\n\
  Sane values: 0 to 400\n\
  Valid range: 0 to 65535",
'JERKX': "maximum jerk for the X axis",
'JERKY': "maximum jerk for the Y axis",
'JERKZ': "maximum jerk for the Z axis",
'JERKE': "maximum jerk for the E axis",
}
