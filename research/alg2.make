#!/bin/bash

gcc -O2 alg2.c -o alg2 && ./alg2 >alg2.data && gnuplot alg2.plot && gnome-open alg2.png

