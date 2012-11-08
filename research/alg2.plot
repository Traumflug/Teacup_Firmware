# set terminal x11 persist raise
set terminal png size 800,600
set output "alg2.png"

set title "G1 X5 Y5 / G1 X5 Y10 / G1 X10 Y10"

set xlabel "X [mm]"
set ylabel "Y [mm]"
set zlabel "delay [us]"

set xrange [0:10.]
set yrange [0:10.]

set arrow from 0,0 to 5,5 nohead
set arrow from 5,5 to 5,10 nohead
set arrow from 10,10 to 10,10 nohead

# 2D
plot "alg2.data" using ($1 / 1000):($2 / 1000) title "position"

# 3D
#set view 45, 20, 1.2, 1.2
#splot "alg2.data" using ($1 / 1000):($2 / 1000):3

#plot "alg.data" using ($1 / 16000000):($2) with lines title "X vs time", \
#		"alg.data" using ($1 / 16000000):($3) with lines title "Y vs time", \
#		"alg.data" using ($2 / 1000):($3) with lines axes x2y1 title "X vs Y", \
#		(x * 34000 / 40) with points axes x2y1 title "Ideal"
