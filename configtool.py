#!/usr/bin/env python

# Note: syntax in this file has to be kept compatible with Pyton 3, else
#       Python errors in the compilation stage already, without showing
#       user help about the incompatibility. To test, simply run
#
#         python3 ./configtool.py
#
#       This should show the explaining text rather than a SyntaxError.
#
# If you feel like porting Configtool to Python 3 compatibility altogether:
# patches are welcome! See https://docs.python.org/3/howto/pyporting.html

import sys
import time
if sys.version_info.major >= 3:
  print("You are currently running Python3. Python3 is not supported, because\n"
        "there is no wxPython for Python3. Please try running with Python2.\n"
        "It often works to type \"python2 configtool.py\" in the command line.")
  time.sleep(10)
  sys.exit(-1)

import getopt
import os.path
import inspect

from configtool.gui import StartGui


def cmdLoad(arg):
  print("Want to load %s, but don't know how.\n" % arg)

def cmdHelp():
  print("""Usage: %s [options]

Running without any options starts the gui (normal operation).
Following options are available for command line automation:

  -h, --help                  Show this help text.

  -l <file>, --load=<file>    Load a specific printer config, board config
                              or .ini file.
""" % sys.argv[0])

def CommandLine(argv):
  """ Parse and act on command line arguments.  All script automation commands
      result in sys.exit() (i.e. they do not return from this function).  Other
      options like --debug will return to allow the gui to launch.
  """
  try:
    opts, args = getopt.getopt(argv, "hl:", ["help", "load="])
  except getopt.GetoptError as err:
    print(err)
    print("Use '%s --help' to get help with command line options." %
          sys.argv[0])
    sys.exit(2)

  # Check for HELP first.
  for opt, arg in opts:
    if opt in ("-h", "--help"):
      cmdHelp()
      sys.exit()

  # Now parse other options.
  for opt, arg in opts:
    if opt in ("-l", "--load"):
      cmdLoad(arg)

if __name__ == '__main__':
  CommandLine(sys.argv[1:])

  cmdFolder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile(
                               inspect.currentframe()))[0]))
  StartGui(cmdFolder)
