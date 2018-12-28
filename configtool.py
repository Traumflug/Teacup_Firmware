#!/usr/bin/env python

# Note: syntax in this file has to be kept compatible with Python 3, else
#       Python errors in the compilation stage already, without showing
#       user help about the incompatibility. To test, simply run
#
#         python3 ./configtool.py
#
#       This should show the explaining text rather than a SyntaxError.
#
# If you feel like porting Configtool to Python 3 compatibility altogether:
# patches are welcome! See https://docs.python.org/3/howto/pyporting.html

from __future__ import print_function
import sys

import getopt
import os.path
import inspect

from configtool.settings import Settings
from configtool.board import Board
from configtool.printer import Printer

cmdFolder = os.path.realpath(
    os.path.abspath(os.path.split(inspect.getfile(inspect.currentframe()))[0])
)

verbose = 0
settings = None
board = None
printer = None


def getSettings(arg=None):
    global settings
    if arg or not settings:
        settings = Settings(None, cmdFolder, arg)
    settings.verbose = verbose
    return settings


def cmdLoad(arg):
    xx, ext = os.path.splitext(arg)
    fn = os.path.basename(arg)

    if ext == ".ini":
        settings = getSettings(arg)
        if not settings.loaded:
            print("Failed to load settings file: %s." % arg)
            sys.exit(2)
        return

    if ext == ".h":
        if fn.startswith("board."):
            global board
            board = Board(getSettings())
            ok, fn = board.loadConfigFile(arg)
            if not ok:
                print("Failed trying to load board file: %s." % fn)
                sys.exit(2)
            return
        elif fn.startswith("printer."):
            global printer
            printer = Printer(getSettings())
            ok, fn = printer.loadConfigFile(arg)
            if not ok:
                print("Failed trying to load printer file: %s" % fn)
                sys.exit(2)
            return

    print("Unrecognized file: %s." % arg)
    print("Expected one of *.ini, board.*.h or printer.*.h.")
    sys.exit(2)


def cmdSave(arg):
    xx, ext = os.path.splitext(arg)
    fn = os.path.basename(arg)

    if ext == ".ini":
        if not getSettings(arg).save(arg):
            print("Failed to save settings file: %s." % arg)
            sys.exit(2)
        return

    if ext == ".h":
        if fn.startswith("board."):
            global board
            if not board.saveConfigFile(arg, None):
                print("Failed trying to save board file: %s." % arg)
                sys.exit(2)
            return
        elif fn.startswith("printer."):
            global printer
            if not printer.saveConfigFile(arg, None):
                print("Failed trying to save printer file: %s." % arg)
                sys.exit(2)
            return

    print("Unrecognized file: %s." % arg)
    print("Expected one of *.ini, board.*.h or printer.*.h.")
    sys.exit(2)


def cmdShowAll():
    names = {"configtool": getSettings(), "board": board, "printer": printer}
    for namespace in names:
        if names[namespace]:
            values = names[namespace].getValues()
            for k in sorted(values):
                print("%s.%s: %s" % (namespace, k, str(values[k])))


def cmdHelp():
    print(
        """Usage: %s [options]

Running without any options starts the gui (normal operation).
Following options are available for command line automation:

  -h, --help                  Show this help text.

  -v, --verbose               Be more verbose. Can be applied multiple times
                              to get even more output.

  -l <file>, --load=<file>    Load a specific printer config, board config
                              or .ini file. Can be applied multiple times to
                              load multiple files.

                              Content of this file is valid before the GUI
                              loads, only. GUI will overwrite them with the
                              files found in config.h.

  -s <file>, --save=<file>    Save the config, printer or board file.

  -a, --show-all              Show all loaded variables and values.

  -q, --quit                  Quit processing without launching the GUI.
"""
        % sys.argv[0]
    )


def CommandLine(argv):
    """ Parse and act on command line arguments.  All script automation commands
      result in sys.exit() (i.e. they do not return from this function).  Other
      options like --debug will return to allow the gui to launch.
  """
    global settings, verbose

    try:
        opts, args = getopt.getopt(
            argv, "hvl:as:q", ["help", "verbose", "load=", "show-all", "save=", "quit"]
        )
    except getopt.GetoptError as err:
        print(err)
        print("Use '%s --help' to get help with command line options." % sys.argv[0])
        sys.exit(2)

    # Check for HELP first.
    for opt, arg in opts:
        if opt in ("-h", "--help"):
            cmdHelp()
            sys.exit()

    # Now parse other options.
    for opt, arg in opts:
        if opt in ("-v", "--verbose"):
            verbose += 1
            getSettings()

        elif opt in ("-l", "--load"):
            cmdLoad(arg)

        elif opt in ("-s", "--save"):
            cmdSave(arg)

        elif opt in ("-a", "--show-all"):
            cmdShowAll()

        elif opt in ("-q", "--quit"):
            sys.exit()


if __name__ == "__main__":
    CommandLine(sys.argv[1:])

    from configtool.gui import StartGui

    StartGui(getSettings())
