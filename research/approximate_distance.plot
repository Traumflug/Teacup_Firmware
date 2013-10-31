# Explore the error ratio of the approximate_distance function

set terminal wxt persist raise

set isosamples 100
set cntrparam levels 10
set contour base
f(x,y) = (x*1007 + 441*y - (x<y*16 ? x*40 : 0))/1024
g(x,y) = (x>y?f(x,y):f(y,x)) / sqrt(x*x+y*y)
splot [0:200000] [0:200000] g(x,y) with pm3d t "Error ratio of approximate_distance"
