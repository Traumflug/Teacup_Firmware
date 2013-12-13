// Thermistor lookup table for one thermistor or
// identical thermistors in all places.

/*
   This table doesn't depend on the type of electronics, but on the type of
   thermistor(s) you use. You want one table for each thermistor type you use.
*/

// How many thermistor tables we have.
#define NUMTABLES 1

// Names for our tables, so you can use them in config.h.
// Table numbering starts at 0.
#define THERMISTOR_EXTRUDER   0

/*
   You may be able to improve the accuracy of this table in various ways.

   1. Measure the actual resistance of the resistor. It's "nominally" 4.7K,
      but that's ± 5%.
   2. Measure the actual beta of your thermistor:
      http://reprap.org/wiki/MeasuringThermistorBeta
   3. Generate more table entries than you need, then trim down the ones
      in uninteresting ranges.

   In either case you'll have to regenerate this table with
   createTemperatureLookup.py, which requires python, which is difficult to
   install on windows. Since you'll have to do some testing to determine the
   correct temperature for your application anyway, you may decide that the
   effort isn't worth it. Who cares if it's reporting the "right" temperature
   as long as it's keeping the temperature steady enough to print, right?
*/

// The number of value pairs in our table.
// Must be the same for all tables.
#define NUMTEMPS 20

const uint16_t PROGMEM temptable[NUMTABLES][NUMTEMPS][2] = {

// Table for the Extruder.
// Thermistor: unknown

// ./createTemperatureLookup.py --r0=100000 --t0=25 --r1=0 --r2=4700 --beta=4066 --max-adc=1023
{
   {1, 3364}, // 841.027617469 C
   {21, 1329}, // 332.486789769 C
   {41, 1104}, // 276.102666373 C
   {61, 987}, // 246.756060004 C
   {81, 909}, // 227.268080588 C
   {101, 851}, // 212.78847342 C
   {121, 805}, // 201.30176775 C
   {141, 767}, // 191.787692666 C
   {161, 734}, // 183.662212795 C
   {181, 706}, // 176.561442671 C
   {201, 680}, // 170.244089549 C
   {221, 658}, // 164.542298163 C
   {241, 637}, // 159.33475843 C
   {321, 567}, // 141.921298995 C
   {381, 524}, // 131.166509425 C
   {581, 406}, // 101.561865389 C
   {781, 291}, // 72.9710018071 C
   {881, 219}, // 54.8051659223 C
   {981, 93}, // 23.4825243529 C
   {1010, 1} // 0.498606463441 C
}
};
