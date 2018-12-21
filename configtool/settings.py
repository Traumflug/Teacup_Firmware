from __future__ import print_function

import ConfigParser
import os
from configtool.data import BSIZESMALL, offsetTcLabel

INIFILE = "configtool.ini"
DEFAULT_INIFILE = "configtool.default.ini"


class Settings:
  def __init__(self, app, folder, ini=None):
    self.app = app
    self.folder = folder
    self.inifile = os.path.join(folder, INIFILE)
    self.section = "configtool"

    self.arduinodir = ""
    self.cflags = ""
    self.ldflags = ""
    self.objcopyflags = ""
    self.programmer = "wiring"
    self.programflags = ""
    self.port = "/dev/ttyACM0"
    self.uploadspeed = 38400

    self.t0 = 25;
    self.r1 = 0;
    self.numTemps = 25
    self.maxAdc = 1023
    self.minAdc = 1

    # Runtime settings
    self.verbose = 0

    self.cfg = ConfigParser.ConfigParser()
    self.cfg.optionxform = str

    self.loaded = self.readConfig(ini)

  def readConfig(self, ini):
    if ini:
      if not self.cfg.read(ini):
        return False
    else:
      if not self.cfg.read(self.inifile):
        if not self.cfg.read(os.path.join(self.folder, DEFAULT_INIFILE)):
          print ("Neither of settings files %s or %s exist. Using default values."
                 % (INIFILE, DEFAULT_INIFILE))
          return False

    if self.cfg.has_section(self.section):
      for opt, value in self.cfg.items(self.section):
        value = value.replace('\n', ' ')
        if opt == "arduinodir":
          self.arduinodir = value
        elif opt == "cflags":
          self.cflags = value
        elif opt == "ldflags":
          self.ldflags = value
        elif opt == "programmer":
          self.programmer = value
        elif opt == "port":
          self.port = value
        elif opt == "objcopyflags":
          self.objcopyflags = value
        elif opt == "programflags":
          self.programflags = value
        elif opt == "t0":
          self.t0 = value
        elif opt == "r1":
          self.r1 = value
        elif opt == "numtemps":
          self.numTemps = value
        elif opt == "maxadc":
          self.maxAdc = value
        elif opt == "minadc":
          self.minAdc = value
        elif opt == "uploadspeed":
          self.uploadspeed = value
        else:
          print("Unknown %s option: %s - ignoring." % (self.section, opt))
    else:
      print("Missing %s section - assuming defaults." % self.section)
      return False

    return True

  def getValues(self):
    return {
      "arduinodir": str(self.arduinodir),
      "cflags": str(self.cflags),
      "ldflags": str(self.ldflags),
      "objcopyflags": str(self.objcopyflags),
      "programmer": str(self.programmer),
      "port": str(self.port),
      "t0": str(self.t0),
      "r1": str(self.r1),
      "numtemps": str(self.numTemps),
      "maxadc": str(self.maxAdc),
      "minadc": str(self.minAdc),
      "uploadspeed": str(self.uploadspeed)
    }

  def saveSettings(self, inifile = None):
    if not inifile:
      inifile = self.inifile

    self.section = "configtool"
    try:
      self.cfg.add_section(self.section)
    except ConfigParser.DuplicateSectionError:
      pass

    values = self.getValues()
    for k in values.keys():
      self.cfg.set(self.section, k, values[k])

    try:
      cfp = open(inifile, 'wb')
    except:
      print("Unable to open settings file %s for writing." % inifile)
      return False

    self.cfg.write(cfp)
    cfp.close()

    return True

