#!/usr/bin/perl

my ($ss, $es, $f, $dn, $dt, $a, $n0, $nn, $np, $c0, $cn, $cp, $n, $c, $t, $tp, $v, $vp);
my ($c0_exact, $cn_exact, $v_exact, $t_exact, $vp_exact, $tp_exact);
my ($n_pre);

$ss = 300;
$es = 400;
$f = 16000000;
$dn = 50000;

$ssq = $ss * $ss;
$esq = $es * $es;
$dsq = ($esq - $ssq);

# $a = $dsq / ($dn << 1);

#$n0 = int(($ss * $ss) / (2 * $a));
#$nn = int(($es * $es) / (2 * $a));
# $n0 = int($ssq * $dn / $dsq);
# $nn = int($esq * $dn / $dsq);

$c0 = int($f / $ss);
# $c0_exact = $f * sqrt(2 / abs($a));


# $dt = ($es - $ss) / $a;

# printf "A:\t%d-%d/%g: %d\n", $es, $ss, $dt, $a;
# printf "N:\t%d-%d %d:%d\n", $n0, $nn, $nn - $n0, $dn, $a;
# printf "C:\t%d\t%g\n", $c0, $c0_exact * (sqrt(abs($n0) + 1) - sqrt(abs($n0)));
# $n = $np = $n0;
$c = $cp = int($f / $ss);
$end_c = int($f / $es);
$t = $tp = $t_exact = 0;
# $v = $vp = $ss;

$n_pre = int(4 * $ssq * $dn / $dsq) | 1;

# $cn_exact = $c0_exact * (sqrt(abs($n0) + 1) - sqrt(abs($n0)));
# $v_exact = $vp_exact = $f / $cn_exact;

printf "\tt:i\t\t\tdt\tn\tV\t\ta\n";
for (0..$dn) {
	# approximation
	# $c = int($c * 1000) / 1000;
	printf "Approx:\t%8.6f:%i\t%10d\t%d\t%12.3f\t%12.3f\n", $t, $_, $c, ($n_pre / 4) - 1, $f / $c, ($t > 0)?($v - $ss) / ($t):0;

# 	$tp = $t;
# 	$cp = $c;
# 	$np = $n;
# 	$vp = $v;

	$t += $c / $f;
	if (
		(($n_pre > 0) && ($c > $end_c)) ||
		(($n_pre < 0) && ($c < $end_c))
		) {
		$c = int($c - ((2 * $c) / $n_pre));
		$n_pre += 4;
	}
# 	$v = $f / $c;

	# exact
# 	printf "Exact:\t%8.6f:%i\t%10.3f\t%i\t%12.3f\t%12.3f\n\n", $t_exact, $_, $cn_exact, $n, $v_exact, ($t_exact > 0)?($v_exact - $ss) / ($t_exact):0
# 		if ($_ % 10 == 0);
#
# 	$vp_exact = $v_exact;
# 	$tp_exact = $t_exact;
#
# 	$t_exact += $cn_exact / $f;
# 	$cn_exact = $c0_exact * (sqrt(abs($n) + 1) - sqrt(abs($n)));
# 	$v_exact = $f / $cn_exact;

	# loop increment
# 	if ($nn > $n0) {
# 		$n++;
		$n_pre += 4;
# 	}
# 	else {
# 		$n--;
# 		$n_pre -= 4;
# 	}
}

printf "dt:%8.3f\tv:%8.3f\n", int(($f / $es) + 0.5), $f / int(($f / $es) + 0.5);
