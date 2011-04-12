# set terminal x11 persist raise
set terminal png size 1024,768
set output "alg.png"

set title "Move from [40,34] to [0,0] with acceleration [9,5] and deceleration [3,8]\n\
showing geometric correctness as a result of acceleration and deceleration trimming"

set xlabel "seconds"
set x2label "millimeters"
set ylabel "millimeters"

plot "alg.data" using ($1 / 16000000):($2 / 1000) with lines title "X vs time", \
		"alg.data" using ($1 / 16000000):($3 / 1000) with lines title "Y vs time", \
		"alg.data" using ($2 / 1000):($3 / 1000) with lines axes x2y1 title "X vs Y", \
		(x * 34 / 40) with points axes x2y1 title "Ideal"
