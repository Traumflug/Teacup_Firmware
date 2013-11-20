#!/usr/bin/gnuplot

set terminal wxt persist raise

set isosamples 1000
set cntrparam levels 10
set contour base

n = 8

a(x,y) = sqrt(x*x+y*y) # Actual distance
b(x,y) = (x>y?x:y)  # max
c(x,y) = (x>y?y:x)  # min

# Find the part of the octant/n which any x/y belong to
scaler(x,y,n) = int(c(x,y)/b(x,y) * n)

# Original selected scale factors
Xl="1023 1008 979 941 898 853 807 757 757"
Yl="65 187 303 406 494 568 632 691 691"

# Refined factors
Xl="1023 1007 979 938 893 845 794 742 742"
Yl="58 189 302 412 502 579 647 706 706"

xscaler(x,y) = (0+word(Xl,1+scaler(x,y,n)))
yscaler(x,y) = (0+word(Yl,1+scaler(x,y,n)))

f(x,y) = (x*xscaler(x,y) + y*yscaler(x,y) + 512 )/1024
g(x,y) = (x>y?f(x,y):f(y,x)) / sqrt(x*x+y*y)

splot [0:200000] [0:200000] [0.98:1.02]  g(x,y) with pm3d t "Error ratio of approximate_distance"
