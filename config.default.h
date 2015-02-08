/** \file config.default.h
  In case you prefer to build with the makefile or with Arduino IDE instead
  of the config tool, copy this file to config.h and adjust it to your needs.

  \note to developers: when adding a file here, also add it to
  configtool/protectedfiles.py.
*/

// Uncomment your controller board, comment out all others.
//#include "config/board.3drag.h"
//#include "config/board.gen3.h"
//#include "config/board.gen6.h"
//#include "config/board.gen7-v1.1-v1.3.h"
#include "config/board.gen7-v1.4.h"
//#include "config/board.ramps-v1.2.h"
//#include "config/board.ramps-v1.3.h"
//#include "config/board.rumba.h"
//#include "config/board.sanguinololu-v1.1.h"
//#include "config/board.sanguinololu-v1.2.h"
//#include "config/board.sanguish.h"
//#include "config/board.teensy-v2.0.h"
//#include "config/board.teensy++-v2.0.h"

// Uncomment your printer type, comment out all others.
#include "config/printer.mendel.h"
//#include "config/printer.wolfstrap.h"
