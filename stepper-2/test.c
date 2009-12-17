//

#include	<stdint.h>
#include	<stdio.h>


// generate with:
// perl -e 'my $n = 16; my @st; for (0..($n - 1)) { push @st, sprintf "%i", sin($_ * 90 * 3.141592653 * 2 / 360 / ($n - 1)) * 255 }; print "#define sinsteps $n\nstatic uint8_t sintable[sinsteps] = { "; print join ", ", @st; print " };\n";';
// #define sinsteps 16
// static uint8_t sintable[sinsteps] = { 0, 26, 53, 78, 103, 127, 149, 170, 189, 206, 220, 232, 242, 249, 253, 255 };

#define sinsteps 5
static uint8_t sintable[sinsteps] = { 0, 97, 180, 235, 255 };

#define	sinstepi  (sinsteps - 1)
#define sinstepi2 (sinstepi * 2)
#define sinstepi3 (sinstepi * 3)
#define sinstepi4 (sinstepi * 4)

int sinstep(uint8_t sequence) {
	while (sequence >= sinstepi4)
		sequence -= sinstepi4;
	if (sequence < (sinstepi + 1))
		return sintable[sequence];
	if ((sequence >= (sinstepi + 1)) && (sequence < (sinstepi2 + 1)))
		return sintable[sinstepi2 - sequence];
	if ((sequence >= (sinstepi2 + 1)) && (sequence < (sinstepi3 + 1)))
		return -sintable[sequence - sinstepi2];
	if ((sequence >= (sinstepi3 + 1)) && (sequence < (sinstepi4 + 1)))
		return -sintable[sinstepi4 - sequence];
}



int main(int argc, char **argv)
{
	int i;
	for (i = 0; i < ((sinstepi * 4) * 3); i++)
	{
		printf("%2i: %+4i %+4i\n", i, sinstep(i), sinstep(i + sinstepi));
	}
	printf("\n");
}