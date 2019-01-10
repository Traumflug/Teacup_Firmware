from __future__ import print_function
from six import iteritems

import time
import sys

try:
    import configparser
except ImportError:
    print(
        "ImportError: No module named configparser\n\n"
        "configparser is not installed. This program requires configparser to run.\n"
        "Your package manager may help you."
    )
    time.sleep(10)
    sys.exit(-1)

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

        self.t0 = 25
        self.r1 = 0
        self.numTemps = 25
        self.maxAdc = 1023
        self.minAdc = 1

        # Runtime settings
        self.verbose = 0

        self.cfg = configparser.ConfigParser()
        self.cfg.optionxform = str

        self.loaded = self.readConfig(ini)

    def readConfig(self, ini):
        if ini:
            if not self.cfg.read(ini):
                return False
        else:
            if not self.cfg.read(self.inifile):
                if not self.cfg.read(os.path.join(self.folder, DEFAULT_INIFILE)):
                    print(
                        "Neither of settings files %s or %s exist. Using default values."
                        % (INIFILE, DEFAULT_INIFILE)
                    )
                    return False

        if self.cfg.has_section(self.section):
            cfg = self.cfg[self.section]
            values = self.getValues()

            def cfg_get(key):
                val = cfg.get(key, values[key])
                val = val.replace("\n", " ")
                return val

            self.arduinodir = cfg_get("arduinodir")
            self.cflags = cfg_get("cflags")
            self.ldflags = cfg_get("ldflags")
            self.objcopyflags = cfg_get("objcopyflags")
            self.programflags = cfg_get("programflags")
            self.programmer = cfg_get("programmer")
            self.port = cfg_get("port")
            self.t0 = cfg_get("t0")
            self.r1 = cfg_get("r1")
            self.numTemps = cfg_get("numtemps")
            self.maxAdc = cfg_get("maxadc")
            self.minAdc = cfg_get("minadc")
            self.uploadspeed = cfg_get("uploadspeed")

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
            "programflags": str(self.programflags),
            "programmer": str(self.programmer),
            "port": str(self.port),
            "t0": str(self.t0),
            "r1": str(self.r1),
            "numtemps": str(self.numTemps),
            "maxadc": str(self.maxAdc),
            "minadc": str(self.minAdc),
            "uploadspeed": str(self.uploadspeed),
        }

    def saveSettings(self, inifile=None):
        if not inifile:
            inifile = self.inifile

        self.section = "configtool"
        try:
            self.cfg.add_section(self.section)
        except configparser.DuplicateSectionError:
            pass

        values = self.getValues()
        for k, v in iteritems(values):
            self.cfg[self.section][k] = v.replace("%", "%%")

        try:
            cfp = open(inifile, "w")
        except:
            print("Unable to open settings file %s for writing." % inifile)
            return False

        self.cfg.write(cfp)
        cfp.close()

        return True
