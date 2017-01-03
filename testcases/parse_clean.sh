name=${1:-000}

cd ..
make -f Makefile-SIM clean
make -f Makefile-SIM USER_CONFIG=./testcases/config.regtest-ramps.h
cd testcases
../sim -t0 -g swan-test.gcode -o

python3 parse_datalog.py 'datalog.out' 'swan.log' "swan-${name}.asc"

gnuplot <<__EOF
set term png size 1024,768
set output "swan-diff-${name}.png"
plot 'swan.log' u 1:4 with lines
set yrange [80:120]
set output "swan-reference-${name}.png"
plot 'swan.log' u 1:2 with lines
set output "swan-current-${name}.png"
plot 'swan.log' u 1:3 with lines
__EOF
