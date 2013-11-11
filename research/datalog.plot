#!/usr/bin/gnuplot

# Plot interesting data from datalog.out simulator trace log
#
# Key bindings:
#   Left-arrow: Scroll left
#   Right-arrow: Scroll right
#   Ctrl-Left: Scroll one screen left
#   Ctrl-Right: Scroll one screen right
#   Plus: Zoom in
#   Minus: Zoom out
#   Z: reset to show whole graph
#

#_____________________________________________________________________________
#                                                                       MACROS
set macro
scroll = "x=(x_right - x_left) * factor ; x_left = x_left +x ; x_right = x_right + x; @normalize ; @update "
zoom = " x=(x_right - x_left) * factor ; x_left = x_left + x ; x_right = x_right - x ; @normalize ; @update"

# Bound left and right of screen to our imported data
lnormalize = "x=(x_left  < GPVAL_DATA_X_MIN ? GPVAL_DATA_X_MIN-x_left  : 0) ; x_left = x_left + x ; x_right = x_right + x "
rnormalize = "x=(x_right > GPVAL_DATA_X_MAX ? GPVAL_DATA_X_MAX-x_right : 0) ; x_left = x_left + x ; x_right = x_right + x "
hnormalize = "x_left=(x_left < 0 ? 0 : x_left) "
normalize = "@lnormalize ; @rnormalize ; @hnormalize "

# Update screen
update = " set xrange[x_left:x_right] ; refresh "

#_____________________________________________________________________________
#                                                                 KEY BINDINGS

# scroll
bind "Right"      "factor=0.10; @scroll"
bind "Left"       "factor=-0.10; @scroll"

# scroll by a whole screen
bind "Ctrl-Right" "factor=1; @scroll"
bind "Ctrl-Left"  "factor=-1; @scroll"

# zoom
bind "+"      "factor=0.25; @zoom"
bind "="      "factor=0.25; @zoom"
bind "-"      "factor=-0.5; @zoom"


#_____________________________________________________________________________
#                                                                    LOAD DATA

# Note: If your datalog columns change, adjust the following to match.

set yrange [-1:14]
set style fill transparent solid 0.5 noborder
plot 'datalog.out' u ($1/10000000):( $6*0.8 + 9)  with steps t 'X_STEP' , \
     'datalog.out' u ($1/10000000):( $7*0.8 + 8)  with steps t 'X_DIR' , \
     'datalog.out' u ($1/10000000):( $9*0.8 + 7)  with steps t 'X_ENABLE' , \
     'datalog.out' u ($1/10000000):($10*0.8 + 6)  with steps t 'Y_STEP' , \
     'datalog.out' u ($1/10000000):($11*0.8 + 5)  with steps t 'Y_DIR' , \
     'datalog.out' u ($1/10000000):($13*0.8 + 4)  with steps t 'Y_ENABLE' , \
     'datalog.out' u ($1/10000000):($21*0.8 + 3)  with steps t 'STEPPER_ENABLE' , \
     'datalog.out' u ($1/10000000):($2/1000)  with filledcurve x1 t 'X_POS' , \
     'datalog.out' u ($1/10000000):($3/1000)  with filledcurve x1 t 'Y_POS' , \
     'datalog.out' u ($1/10000000):($4/10  )  with filledcurve x1 t 'Z_POS'

# Set the range (need to do this after our initial plot so data is loaded)
x_left = GPVAL_DATA_X_MIN
x_right= GPVAL_DATA_X_MAX

set yrange [-1:14] ; @update

pause mouse close
