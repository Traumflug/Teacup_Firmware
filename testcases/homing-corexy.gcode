G1 X10 Y10 F3000
G28 X
; Watch Y after we home X. We do not change Y in the gcode, so Y should not
; drift.
G1 X10 F3000
; The printer should report we are at X10 Y10. The step counters should match.
M114
G28 Z
