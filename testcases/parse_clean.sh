drwnumber=${1:-000}

if [ -z "$2" ]
  then
    test_it=true
  else
    test_it=false
fi

if $test_it
  then
    cd ..
    make -f Makefile-SIM clean
    make -f Makefile-SIM
    cd testcases
    ../sim -t0 -g swan-test.gcode -o
fi

python3 parse_datalog.py 'datalog.out' 'swan.log' "swan-${drwnumber}.asc"

gnuplot <<__EOF
set term png size 1024,768
set output "swan-diff-${drwnumber}.png"
plot 'swan.log' u 1:4 with lines
set yrange [80:120]
set output "swan-reference-${drwnumber}.png"
plot 'swan.log' u 1:2 with lines
set output "swan-current-${drwnumber}.png"
plot 'swan.log' u 1:3 with lines
__EOF
