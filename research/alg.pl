#!/usr/bin/perl

use strict;

# from http://www.eetimes.com/design/embedded/4006438/Generate-stepper-motor-speed-profiles-in-real-time
# f = F_CPU
# a = 1 / steps_per_mm (ie mm per step)
# w = speed (mm/sec)
# w' = accel (mm/sec/sec)
# c = timer ticks (integer)
# n = acceleration value (integer)
# C0 = f * sqrt(2 a / w' )
#    = F_CPU * sqrt(2 / accel / steps_per_mm)
# Cn = C0 * (sqrt(n + 1) - sqrt(n))
# approximation:
# Cn = Cn-1 - ((2 * Cn-1) / (4n + 1))
# detach n from step index:
# Ci = Ci-1 - ((2 * Ci-1) / (4ni + 1))
# ramp down to stop in m steps:
# ni = i - m
# inaccuracies: C1 is inaccurate
# use c1 = 0.4056 * C0

# number of steps to reach speed w with acceleration w':
# n = (w^2 / (2 * a * w'))
#   = w * w * steps_per_mm / 2 / w'

# changes of acceleration
# (n1 + 0.5).w'1 = (n2 + 0.5).w'2
# n2 = ((n1 + 0.5) * w'1 / w'2) - 0.5

# when to decelerate (short move of m steps)
# n = m.w'2 / (w'1 + w'2)


my $f_cpu = 16000000;
my ($x_mm, $y_mm, $f_mm_min) = (40, 34, 1500);
my ($x_steps_per_mm, $y_steps_per_mm) = (320, 320);

my ($x_accel_mm_s_s, $y_accel_mm_s_s) = (9, 5);
my ($x_decel_mm_s_s, $y_decel_mm_s_s) = (3, 8);

# **************************************

my ($x_um_per_step, $y_um_per_step) = (1000 / $x_steps_per_mm, 1000 / $y_steps_per_mm);

my ($x_delta, $y_delta) = ($x_mm * $x_steps_per_mm, $y_mm * $y_steps_per_mm);

my $distance = sqrt(($x_delta * $x_delta * $x_um_per_step * $x_um_per_step) + ($y_delta * $y_delta * $y_um_per_step * $y_um_per_step));

my $duration = $distance * $f_cpu * 60 / 1000 / $f_mm_min;

printf "MOVE %dmmx%dmm@%dmm/min: %d um (%d mm), %d ticks (%dms), %gmm/min (%gmm/s)\n", $x_mm, $y_mm, $f_mm_min, $distance, $distance / 1000, $duration, $duration / $f_cpu * 1000, $distance / 1000 / $duration * $f_cpu * 60, $distance / 1000 / $duration * $f_cpu;

my ($x_speed, $y_speed) = ($x_delta * $x_um_per_step / $duration * $f_cpu / 1000, $y_delta * $y_um_per_step / $duration * $f_cpu / 1000);

printf "X: %gmm/s, Y: %gmm/s\n", $x_speed, $y_speed;

# **************************************
# n steps to accelerate to w at w' = w * w * steps_per_mm / 2 / w'
my $x_steps_to_accel = $x_speed * $x_speed * $x_steps_per_mm / 2 / $x_accel_mm_s_s;
my $y_steps_to_accel = $y_speed * $y_speed * $y_steps_per_mm / 2 / $y_accel_mm_s_s;

printf "Xns: %d (%dum), Yns: %d (%dum)\n", $x_steps_to_accel, $x_steps_to_accel * $x_um_per_step, $y_steps_to_accel, $y_steps_to_accel * $y_um_per_step;

# now we work out which axis reaches plateau last
if ($x_steps_to_accel / $x_steps_per_mm > $y_steps_to_accel / $y_steps_per_mm) {
	# x reaches last- slow down Y
	# when X reaches plateau, where is Y?
	# x_steps / x_distance = y_steps / y_distance
	# y_steps = x_steps / x_distance * y_distance
	my $y_plateau_steps = $x_steps_to_accel / $x_delta * $y_delta;
	$y_accel_mm_s_s = $y_speed * $y_speed * $y_steps_per_mm / 2 / $y_plateau_steps;
}
else {
	# y reaches last- slow down X
	# when Y reaches plateau, where is X?
	# y_steps / y_distance = x_steps / x_distance
	# x_steps = y_steps / y_distance * x_distance
	my $x_plateau_steps = $y_steps_to_accel / $y_delta * $x_delta;
	$x_accel_mm_s_s = $x_speed * $x_speed * $x_steps_per_mm / 2 / $x_plateau_steps;
}

$x_steps_to_accel = $x_speed * $x_speed * $x_steps_per_mm / 2 / $x_accel_mm_s_s;
$y_steps_to_accel = $y_speed * $y_speed * $y_steps_per_mm / 2 / $y_accel_mm_s_s;

printf "new Xns: %d, Yns: %d\n", $x_steps_to_accel, $y_steps_to_accel;
printf "Xaccel: %g, Yaccel: %g\n", $x_accel_mm_s_s, $y_accel_mm_s_s;

# now we work out which axis has to decelerate first
# n steps to decelerate from w at w' = w * w * steps_per_mm / 2 / w'
my $x_steps_to_decel = $x_speed * $x_speed * $x_steps_per_mm / 2 / $x_decel_mm_s_s;
my $y_steps_to_decel = $y_speed * $y_speed * $y_steps_per_mm / 2 / $y_decel_mm_s_s;

printf "Xds: %d, Yds: %d\n", $x_steps_to_decel, $y_steps_to_decel;

# now we work out which axis reaches plateau last
if ($x_steps_to_decel / $x_steps_per_mm > $y_steps_to_decel / $y_steps_per_mm) {
	# x reaches last- slow down Y
	# when X reaches plateau, where is Y?
	# x_steps / x_distance = y_steps / y_distance
	# y_steps = x_steps / x_distance * y_distance
	my $y_plateau_steps = $x_steps_to_decel / $x_delta * $y_delta;
	$y_decel_mm_s_s = $y_speed * $y_speed * $y_steps_per_mm / 2 / $y_plateau_steps;
}
else {
	# y reaches last- slow down X
	# when Y reaches plateau, where is X?
	# y_steps / y_distance = x_steps / x_distance
	# x_steps = y_steps / y_distance * x_distance
	my $x_plateau_steps = $y_steps_to_decel / $y_delta * $x_delta;
	$x_decel_mm_s_s = $x_speed * $x_speed * $x_steps_per_mm / 2 / $x_plateau_steps;
}

my $x_steps_to_decel = $x_speed * $x_speed * $x_steps_per_mm / 2 / $x_decel_mm_s_s;
my $y_steps_to_decel = $y_speed * $y_speed * $y_steps_per_mm / 2 / $y_decel_mm_s_s;

printf "new Xds: %d, Yds: %d\n", $x_steps_to_decel, $y_steps_to_decel;

if (($x_steps_to_accel + $x_steps_to_decel) > $x_delta) {
	# we will never reach full speed, however this doesn't affect our accel trimming so we can do this last
	# n = (m.w'2) / (w'1 + w'2)
	$x_steps_to_decel = $x_delta * $x_decel_mm_s_s / ($x_accel_mm_s_s + $x_decel_mm_s_s);
}
if (($y_steps_to_accel + $y_steps_to_decel) > $y_delta) {
	# we will never reach full speed, however this doesn't affect our accel trimming so we can do this last
	# n = (m.w'2) / (w'1 + w'2)
	$y_steps_to_decel = $y_delta * $y_decel_mm_s_s / ($y_accel_mm_s_s + $y_decel_mm_s_s);
}

printf "new Xds: %d, Yds: %d\n", $x_steps_to_decel, $y_steps_to_decel;

# now we work out initial delays (C0)

# = F_CPU * sqrt(2 / accel / steps_per_mm)
my $x_c = $f_cpu * sqrt(2 / $x_accel_mm_s_s / $x_steps_per_mm);
my $y_c = $f_cpu * sqrt(2 / $y_accel_mm_s_s / $y_steps_per_mm);

# now we work out speed limits so we know when to stop accelerating

# mm/sec -> ticks per step
# mm/sec * steps/mm = steps/sec
# 1 / (mm/sec * steps/sec) = secs/step
# f_cpu / (mm/sec * steps/sec) = ticks/step
my $x_min_c = $f_cpu / ($x_speed * $x_steps_per_mm);
my $y_min_c = $f_cpu / ($y_speed * $y_steps_per_mm);

printf "XminC: %dt/s, YminC: %dt/s\n", $x_min_c, $y_min_c;

# now we set up counters

my $x_n = 1;
my $y_n = 1;

printf "Xc0: %d (%gus), Yc0: %d (%gus)\n", $x_c, $x_c / $f_cpu * 1000000, $y_c, $y_c / $f_cpu * 1000000;

my $elapsed_ticks = ($x_c < $y_c)?$x_c:$y_c;
my ($x_cd, $y_cd) = ($x_c, $y_c);

my $total_ticks = 0;

printf stderr "%d %.3f %.3f\n", $total_ticks, $x_delta / $x_steps_per_mm, $y_delta / $y_steps_per_mm;

while ($x_delta > 0 || $y_delta > 0) {
	$x_cd -= $elapsed_ticks;
	$y_cd -= $elapsed_ticks;
	if ($x_cd <= 0 && $x_delta > 0) {
		$x_delta--;
		if ($x_delta == int($x_steps_to_decel)) {
			# start decelerating
			$x_n = -$x_delta;
			printf "[X DECEL]";
		}
		printf "[X: %ds:%gmm, %dc, %dn] ", $x_delta, $x_delta / $x_steps_per_mm, $x_c, $x_n;
		if ($x_n == 1) {
			$x_c = 0.4056 * $x_c;
		}
		else {
			$x_c = $x_c - ((2 * $x_c) / ((4 * $x_n) + 1));
		}
		$x_cd = $x_c;
		$x_n++;
		$x_c = $x_min_c if $x_c < $x_min_c;
	}
	if ($y_cd <= 0 && $y_delta > 0) {
		$y_delta--;
		if ($y_delta == int($y_steps_to_decel)) {
			$y_n = -$y_delta;
			printf "[Y DECEL]";
		}
		printf "[Y: %ds:%gmm, %dc, %dn] ", $y_delta, $y_delta / $y_steps_per_mm, $y_c, $y_n;
		if ($y_n == 1) {
			$y_c = 0.4056 * $y_c;
		}
		else {
			$y_c = $y_c - ((2 * $y_c) / ((4 * $y_n) + 1));
		}
		$y_cd = $y_c;
		$y_n++;
		$y_c = $y_min_c if $y_c < $y_min_c;
	}

	printf stderr "%d %.3f %.3f\n", $total_ticks, $x_delta / $x_steps_per_mm, $y_delta / $y_steps_per_mm;

	$elapsed_ticks = 2**31;
	$elapsed_ticks = $x_cd
		if $x_delta > 0 && $elapsed_ticks > $x_cd;
	$elapsed_ticks = $y_cd
		if $y_delta > 0 && $elapsed_ticks > $y_cd;

	if ($elapsed_ticks < 2**31) {
		$total_ticks += $elapsed_ticks;
		printf "wait %d ticks\n", $elapsed_ticks;
	}
	else {
		print "finished\n";
	}
}
