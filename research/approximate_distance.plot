#!/usr/bin/gnuplot

reset

# Phase angle adjustment
#
# I don't quite understand this part, but I've empirically
# determined an optimized "tuning" to these sin/cos points
# which are not at the expected offsets.  The tuning is an
# adjustment to the landing angle each of the points 0-7.
# It closely follows a familiar (sine) curve.
#
# Point:        0     1     2     3     4     5     6     7
# Adjustment:  0.65  0.9  1.1   1.20  1.25  1.1   0.95   0.7
#
# Our phase-shift _should_ be identity plus some constant:
# phase(x) = x + 0.65
#
# But it has an extra sine component for some reason.  The following
# phase correction works well for points=8, but it was hand-tuned.
#
phase(x) = x + 0.65 + 0.5 * sin(x/5.9*pi-x*.05 )

# Discrete angle values (p values per circle)
dv(x,p) = phase(floor(x*p/2.0/pi + 0.5))*2*pi/p

octantPoints = 8
totalPoints = octantPoints * 8

d(x) = dv(x,totalPoints)
ac(x,s) = cos(dv(x,s*8))
as(x,s) = sin(dv(x,s*8))

# Angle between each pair of points
theta(n) = 2*pi/totalPoints

# Adjust to compensate for "error" of chords approximating a circle with n points
  # Choose any two points
  x1(n,t) = ac(t,n)
  y1(n,t) = as(t,n)
  x2(n,t) = ac(t+theta(n),n)
  y2(n,t) = as(t+theta(n),n)

  # Find the center of the resulting chord
  cx(n,t) = (x1(n,t) + x2(n,t)) / 2
  cy(n,t) = (y1(n,t) + y2(n,t)) / 2

  # Find the chord's center "radius"
  cr(n,t) = sqrt( cx(n,t)**2 + cy(n,t)**2 )

  # target is the (fudged/weighted) average of the ideal radius (1.0) and the chord radius
  tr(n) = ( 1 + cr(n,0)) / 2

  # Reach the target by scaling the chosen point radius out by tr/cr
  sf(n) = tr(n)/cr(n,0)

  # Scaled samples for cos and sin
  sc(x,p) = sf(p)*ac(x,p)
  ss(x,p) = sf(p)*as(x,p)

# Approximation function with 10-bit fixed-point decimal
a(x,y) = sqrt(x*x+y*y) # Actual distance
b(x,y) = (x>y?x:y)  # max
c(x,y) = (x>y?y:x)  # min

# Find the part of the octant/n which any x/y belong to
scaler(x,y,n) = floor(int(c(x,y)/b(x,y) * n))

# Find the multipliers to use for a given octant-segment
xscaler(p,n) = floor(sc(p*pi/4/n,n) * 1024+0.5)
yscaler(p,n) = floor(ss(p*pi/4/n,n) * 1024+0.5)

n = octantPoints
# Graph the resulting ratio of approx-distance to actual-distance
mx(x,y,p) = ((xscaler(p,n)*b(x,y)+512)/1024.0 + (yscaler(p,n)*c(x,y)+512)/1024.0 )
lx(x,y) = mx(x,y,scaler(x,y,n))
mr(x,y) = lx(x,y)/a(x,y)
md(x,y) = lx(x,y)-a(x,y)

fx(x,y) = (p = scaler(x,y,n) , xscaler( p, n))
fy(x,y) = (p = scaler(x,y,n) , yscaler( p, n))


#----------------------------------------------------------
# Calculated values

print sprintf("uint16_t xscale[%u] = { /* X-scaler values when ... */ ", octantPoints)
  do for [i=0:octantPoints-1] {
   print sprintf("%u,", xscaler(i,octantPoints))
  }
print "};"

print sprintf("uint16_t yscale[%u] = { /* Y-scaler values when ... */ ", octantPoints)
  do for [i=0:octantPoints-1] {
   print sprintf("%u,", yscaler(i,octantPoints))
  }
print "};"

#----------------------------------------------------------
# Graph ratio of approximated value to real value

set grid
set term wxt 1
set samples 1000
set ylabel "Percent error (actual)"
set xlabel "Smaller leg (when larger leg=10000)"
plot [0:10000] [0:0.4] 100*abs(1.0-mr(10000,x)) with lines t "Distance approximation error (percent)"

set ylabel ""
set xlabel ""
set term wxt 2
set isosamples 50
splot [0:2000] [0:2000] [0.9:1.1] mr(x,y) with pm3d t "Approximate/Actual (ratio)"

set ylabel ""
set xlabel ""
set term wxt 3
set isosamples 100
splot [0:2000] [0:2000] [-50:50] md(x,y) with pm3d t "Approximate-Actual (difference)"

#----------------------------------------------------------
# Graph unit-circle derivation coordinates

set term wxt 0
set parametric
set size square
set samples 1000

# Draw one octet of a circle
#plot [0:pi*2] [-1.5:1.5] [-1.5:1.5] cos(t),sin(t)
set arrow 1 from 0,0 to cos(0),sin(0)  nofilled linetype 1
set arrow 2 from 0,0 to cos(pi/4),sin(pi/4)  nofilled linetype 1
plot [0:pi/4] [0.5:1.3] [0:0.8] cos(t),sin(t) t "Ideal circle"

x=cx(octantPoints, 0)
y=cy(octantPoints, 0)
s=sf(octantPoints)
set arrow 3 from x,y to x*s,y*s nofilled linetype 1

do for [i=0:octantPoints-1] {
  set label sprintf("%u", i) at xscaler(i,octantPoints)/1024.0, yscaler(i,octantPoints)/1024.0
}

#replot  ac(t,2),as(t,2) with linespoints t "2 Samples"
replot  ac(t,8),as(t,8) with linespoints t "8 Samples"

replot  sc(t,octantPoints),ss(t,octantPoints) with lines t sprintf("%u Samples - scaled", octantPoints)

pause mouse button1
