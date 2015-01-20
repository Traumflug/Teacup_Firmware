
import ConfigParser
import os

INIFILE = "configtool.default.ini"


class Settings:
  def __init__(self, app, folder):
    self.app = app
    self.cmdfolder = folder
    self.inifile = os.path.join(folder, INIFILE)
    self.section = "configtool"

    self.arduinodir = ""
    self.cflags = ""
    self.ldflags = ""
    self.objcopyflags = ""
    self.programmer = "wiring"
    self.port = "/dev/ttyACM0"

    self.cfg = ConfigParser.ConfigParser()
    self.cfg.optionxform = str
    if not self.cfg.read(self.inifile):
      print "Settings file %s does not exist. Using default values." % INIFILE

      return

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
        else:
          print "Unknown %s option: %s - ignoring." % (self.section, opt)
    else:
      print "Missing %s section - assuming defaults." % self.section
