#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Creates a C code lookup table for doing ADC to temperature conversion
# on a microcontroller
# based on: http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html
# Modified Thu 10 Feb 2011 02:02:28 PM MST jgilmore for 5D_on_arduino firmware
#	temps are now in 14.2 fixed point notation (i.e. measured in quarter-degrees)
#	temps are not permitted to be negative (BUG:may result in numtemps fewer than requested)
#	bugfix: --num-temps command line option works.
# 2012-11-08, DaveX: Modified to add  --vcc=, --min_adc=, --mult and to print per-ADC comments  

"""Thermistor Value Lookup Table Generator

Generates lookup to temperature values for use in a microcontroller in C format based on: 
  http://hydraraptor.blogspot.com/2007/10/measuring-temperature-easy-way.html

The main use is for Arduino programs that read data from the circuit board described here:
  http://make.rrrf.org/ts-1.0 or http://reprap.org/wiki/Temperature_Sensor_2_0

Usage: python createTemperatureLookup.py [options]

Options:
  -h, --help			show this help
  --r0=... 			thermistor rating where # is the ohm rating of the thermistor at t0 (eg: 10K = 10000)
  --t0=... 			thermistor temp rating where # is the temperature in Celsuis to get r0 (from your datasheet)
  --beta=...			thermistor beta rating. see http://reprap.org/bin/view/Main/MeasuringThermistorBeta
  --r1=...			R1 rating where # is the ohm rating of R1 (eg: 10K = 10000)
  --r2=... 			R2 rating where # is the ohm rating of R2 (eg: 10K = 10000)
  --num-temps=... 	the number of temperature points to calculate (default: 20)
  --max-adc=... 	the max ADC reading to use.  if you use R1, it limits the top value for the thermistor circuit, and thus the possible range of ADC values
  --min-adc=... 	the minimum ADC reading to use. 
  --vadc=...            ADC reference voltage (high leg of R2) same as Vcc
  --vcc=...             Voltage divider supply (high leg of R2)  Unused
  --table               Format data as one of an array of tables

It is suggested to generate more values than you need, and delete some of the ones in the ranges
that aren't interesting. This will improve accuracy in the temperature ranges that are important to you.
"""

from math import *
import sys
import getopt

class Thermistor:
	"Class to do the thermistor maths"
	def __init__(self, r0, t0, beta, r1, r2,vcc,vadc):
		self.r0 = r0                        # stated resistance, e.g. 10K
		self.t0 = t0 + 273.15               # temperature at stated resistance, e.g. 25C
		self.beta = beta                    # stated beta, e.g. 3500
		self.vadc = vadc                    # ADC reference
		self.vcc = vcc                      # supply voltage to potential divider
		self.k = r0 * exp(-beta / self.t0)   # constant part of calculation

		if r1 > 0:
			self.vs = r1 * self.vadc / (r1 + r2) # effective bias voltage
			self.rs = r1 * r2 / (r1 + r2)       # effective bias impedance
		else:
			self.vs = self.vadc					 # effective bias voltage
			self.rs = r2                         # effective bias impedance

	def temp(self,adc):
		"Convert ADC reading into a temperature in Celcius"
		v = adc * self.vadc / 1024          # convert the 10 bit ADC value to a voltage
		r = self.rs * v / (self.vs - v)     # resistance of thermistor
		try:
			return (self.beta / log(r / self.k)) - 273.15        # temperature
		except: 
			print "// error for ADC={adc}, {v},{r}".format(adc=adc, v=v,r=r)
			return 0

        def resistance(self, t):
		"Convert a temperature into a thermistor resistance"
		return self.r0 * exp(self.beta * (1 / (t + 273.15) - 1 / self.t0)) # resistance of the thermistor

	def setting(self, t):
		"Convert a temperature into a ADC value"
		r = self.r0 * exp(self.beta * (1 / (t + 273.15) - 1 / self.t0))  # resistance of the thermistor
		v = self.vs * r / (self.rs + r)     # the voltage at the potential divider
		return round(v / self.vadc * 1024)  # the ADC reading

def main(argv):

	r0 = 10000;
	t0 = 25;
	beta = 3947;
	r1 = 680;
	r2 = 1600;
	num_temps = int(20);
	max_adc = int(1023);
	min_adc = int(1);
	vadc=5.0
	vcc=5.0
	mult=4
	table=False

	try:
		opts, args = getopt.getopt(argv, "h", ["help", "r0=", "t0=", "beta=", "r1=",
		                                       "r2=", "max-adc=", "min-adc=",
		                                       "num-temps=", "vcc=", "vadc=",
		                                       "multiplier=", "table"])
	except getopt.GetoptError:
		usage()
		sys.exit(2)

	for opt, arg in opts:
		if opt in ("-h", "--help"):
			usage()
			sys.exit()
		elif opt == "--r0":
			r0 = int(arg)
		elif opt == "--t0":
			t0 = int(arg)
		elif opt == "--beta":
		 	beta = int(arg)
		elif opt == "--r1":
			r1 = int(arg)
		elif opt == "--r2":
			r2 = int(arg)
		elif opt == "--max-adc":
			max_adc = int(arg)
		elif opt == "--min-adc":
			min_adc = int(arg)
		elif opt == "--num-temps":
			num_temps = int(arg)
		elif opt == "--vadc":
			vadc = float(arg)
		elif opt == "--vcc":
			vcc = float(arg)
		elif opt == "--multiplier":
			mult = float(arg)
		elif opt == "--table":
			table = True
	if r1:
		max_adc = int(1023. * r1 / (r1 + r2))
	else:
		max_adc = 1023

	increment = int((max_adc-min_adc)/(num_temps-1));
	t = Thermistor(r0, t0, beta, r1, r2, vcc, vadc)

	adcs = range(min_adc, max_adc, increment);
	adcs.append(max_adc)
#	adcs = [1, 20, 25, 30, 35, 40, 45, 50, 60, 70, 80, 90, 100, 110, 130, 150, 190, 220,  250, 300]

	#Chop of negative temperatures (as we're using a unsigned 16-bit value for temp)
	for i in range(0,len(adcs)):
		if int(t.temp(adcs[i])*mult) < 0:
			adcs=adcs[0:i+1]
			#Replace this with the ADC reading for 0C
			adcs[i]=int(t.setting(0))
			#If the closes ADC reading to 0C is negative, convert to next highest ADC reading
			if int(t.temp(adcs[i])*mult)<0:
				adcs[i] -=1
			break
	print "// Thermistor lookup table for RepRap Temperature Sensor Boards (http://reprap.org/wiki/Temperature_Sensor_2_0)"
        print "// Made with createTemperatureLookup.py (https://github.com/traumflug/Teacup_Firmware/blob/master/createTemperatureLookup.py)"
        print "//           (patched per https://github.com/drf5n/Teacup_Firmware/blob/Gen7/createTemperatureLookup.py)"
	print "// default thermistor lookup table"
	print "// You may be able to improve the accuracy of this table in various ways."
	print "//   1. Measure the actual resistance of the resistor. It's \"nominally\" 4.7K, but that's ± 5%."
	print "//   2. Measure the actual beta of your thermistor:http://reprap.org/wiki/MeasuringThermistorBeta"
	print "//   3. Generate more table entries than you need, then trim down the ones in uninteresting ranges."
	print "// In either case you'll have to regenerate this table, which requires python, which is difficult to install on windows."
	print "// Since you'll have to do some testing to determine the correct temperature for your application anyway, you"
	print "// may decide that the effort isn't worth it. Who cares if it's reporting the \"right\" temperature as long as it's"
	print "// keeping the temperature steady enough to print, right?"
	print "// Temp*%s table from https://github.com/drf5n/Teacup_Firmware/blob/Gen7/createTemperatureLookup.py" %mult
	print "// ./createTemperatureLookup.py --r0=%s --t0=%s --r1=%s --r2=%s --beta=%s --max-adc=%s --min-adc=%s --multiplier=%s --vadc=%s" % (
		r0, t0, r1, r2, beta, max_adc, min_adc, mult, vadc)
	print "// r0: %s" % (r0)
	print "// t0: %s" % (t0)
	print "// r1: %s (parallel with rTherm)" % (r1)
	print "// r2: %s (series with rTherm)" % (r2)
	print "// beta: %s" % (beta)
	print "// min adc: %s at %s V" % (min_adc, min_adc*t.vadc/1024)
	print "// max adc: %s at %s V" % (max_adc, max_adc*t.vadc/1024)
	print "// ADC counts from {min} to {max} by {x}".format(min=min_adc, max=max_adc, x=increment)
	if table == True:
		print "// #define NUMTABLES 1    // These three lines open the temptable[NUMTABLES]... array"
		print "// #define NUMTEMPS %s   //  ...   " %  (len(adcs))
		print "// uint16_t temptable[NUMTABLES][NUMTEMPS][2] PROGMEM = { // ..."
		print "{ //" + " Table 0 chunk for B={b}, R0={r0}, R1={r1}, R2={r2}, Vref={v}".format(par="{",b=beta,r0=r0,r1=r1,r2=r2,v=vadc) 
	else:
		print "#define NUMTEMPS  %s " %  (len(adcs))
		print "const uint16_t temptable[NUMTEMPS][2] PROGMEM = {"
	print "// {ADC, temp*%s }, // temp         Rtherm     Vtherm      resolution   power" % (mult)

	counter = 0
	for adc in adcs:
		counter = counter +1
		degC=t.temp(adc)
		resistance=t.resistance(t.temp(adc))
		vTherm= adc*t.vadc/1024
		ptherm= vTherm*vTherm/resistance
		resolution = ( t.temp(adc-1)-t.temp(adc) if adc>1 else t.temp(adc) -t.temp(adc+1))
		sep = (',' if counter != len(adcs) else ' ')
		print "   {%4s, %6s}%s // %7.2f C,  %7.0f Ohm, %0.3f V, %0.2f C/count, %0.2fmW" % (adc, int(t.temp(adc)*mult), sep,degC, resistance,vTherm,resolution,ptherm*1000)
	if table == False:
		print "};"
	else:
		print '}, // remove comma for last table chunk'
		print "// };   // Closure for the temptable[NUMTABLES] array"

def usage():
    print __doc__

if __name__ == "__main__":
	main(sys.argv[1:])
