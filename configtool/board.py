from __future__ import print_function

import os
import re

from sys import platform
from configtool.data import (
    defineValueFormat,
    defineBoolFormat,
    defineHeaterFormat,
    reHelpTextStart,
    reHelpTextEnd,
    reStartSensors,
    reEndSensors,
    reStartHeaters,
    reEndHeaters,
    reCandHeatPins,
    reCandThermPins,
    reCandProcessors,
    reCandCPUClocks,
    reFloatAttr,
    reDefine,
    reDefineBL,
    reDefQS,
    reDefQSm,
    reDefQSm2,
    reDefBool,
    reDefBoolBL,
    reDefHT,
    reDefTS,
    reDefTT,
    reSensor,
    reHeater3,
    reHeater4,
    reHeater5,
    reTempTable4,
    reTempTable7,
)


class Board:
    def __init__(self, settings):
        self.settings = settings
        self.configFile = None
        self.cfgDir = os.path.join(self.settings.folder, "configtool")

        self.cfgValues = {}
        self.heaters = []
        self.sensors = []
        self.candHeatPins = []
        self.candThermPins = []

    def getValues(self):
        vars = [("sensor." + x[0], x[1:]) for x in self.sensors]
        vars += [("heater." + x[0], x[1:]) for x in self.heaters]
        vars += [(x, self.cfgValues[x]) for x in self.cfgValues]
        return dict(vars)

    def getCPUInfo(self):
        vF_CPU = None
        if "F_CPU" in self.cfgValues.keys():
            vF_CPU = self.cfgValues["F_CPU"][0]

        vCPU = None
        if "CPU" in self.cfgValues.keys():
            vCPU = self.cfgValues["CPU"][0]

        return vF_CPU, vCPU

    def hasData(self):
        return self.configFile != None

    def getFileName(self):
        return self.configFile

    def loadConfigFile(self, fn):
        cfgFn = os.path.join(self.cfgDir, "board.generic.h")
        try:
            self.cfgBuffer = list(open(cfgFn))
        except:
            return False, cfgFn

        try:
            self.userBuffer = list(open(fn))
        except:
            return False, fn

        self.configFile = fn

        self.sensors = []
        self.heaters = []
        self.candHeatPins = []
        self.candThermPins = []
        self.candProcessors = []
        self.candClocks = []
        self.tempTables = {}
        gatheringHelpText = False
        helpTextString = ""
        helpKey = None

        self.cfgValues = {}
        self.cfgNames = set()
        self.helpText = {}

        prevLines = ""
        for ln in self.cfgBuffer:
            if gatheringHelpText:
                if reHelpTextEnd.match(ln):
                    gatheringHelpText = False
                    helpTextString = helpTextString.strip()
                    # Keep paragraphs with double-newline.
                    helpTextString = helpTextString.replace("\n\n  ", "\n\n")
                    # Keep indented lines, typically a list.
                    helpTextString = helpTextString.replace("\n\n  ", "\n\n    ")
                    helpTextString = helpTextString.replace("\n    ", "\n\n    ")
                    # Remove all other newlines and indents.
                    helpTextString = helpTextString.replace("\n  ", " ")
                    hk = helpKey.split()
                    for k in hk:
                        self.helpText[k] = helpTextString
                    helpTextString = ""
                    helpKey = None
                    continue
                else:
                    helpTextString += ln
                    continue

            m = reHelpTextStart.match(ln)
            if m:
                t = m.groups()
                gatheringHelpText = True
                helpKey = t[0]
                continue

            if ln.rstrip().endswith("\\"):
                prevLines += ln.rstrip()[:-1]
                continue

            if prevLines != "":
                ln = prevLines + ln
                prevLines = ""

            self.parseDefineName(ln)
            self.parseDefineValue(ln)

        # Set all boolean generic configuration items to False, so items not yet
        # existing in the user configuration default to disabled.
        #
        # An alternative would be to allow both, enabled and disabled booleans
        # in board.generic.h, which then allows to set an appropriate default for
        # each #define. This was tried but conflicted with config file writing code
        # below (disabled #defines were reset to the default, even when set
        # differently in the GUI), so this would need adjustment, too.
        for k in self.cfgValues.keys():
            if isinstance(self.cfgValues[k], bool):
                self.cfgValues[k] = False

        # Read the user configuration. This usually overwrites all of the items
        # read above, but not those missing in the user configuration, e.g.
        # when reading an older config.
        gatheringHelpText = False
        prevLines = ""
        for ln in self.userBuffer:
            if gatheringHelpText:
                if reHelpTextEnd.match(ln):
                    gatheringHelpText = False
                continue

            if reHelpTextStart.match(ln):
                gatheringHelpText = True
                continue

            if ln.rstrip().endswith("\\"):
                prevLines += ln.rstrip()[:-1]
                continue

            if prevLines != "":
                ln = prevLines + ln
                prevLines = ""

            if self.parseCandidateValues(ln):
                continue

            if self.parseDefineValue(ln):
                continue

            m = reDefTS.search(ln)
            if m:
                t = m.groups()
                if len(t) == 1:
                    s = self.parseSensor(t[0])
                    if s:
                        self.sensors.append(s)
                        continue

            m = reDefHT.search(ln)
            if m:
                t = m.groups()
                if len(t) == 1:
                    s = self.parseHeater(t[0])
                    if s:
                        self.heaters.append(s)
                        continue

        # Parsing done. All parsed stuff is now in these arrays and dicts.
        if self.settings.verbose >= 2:
            print(self.sensors)
            print(self.heaters)
            print(self.candHeatPins)
            print(self.candThermPins)
            print(self.candProcessors)
            print(self.candClocks)
            print(self.tempTables)
            print(self.cfgValues)  # #defines with a value.
            print(self.cfgNames)  # Names found in the generic file.
        if self.settings.verbose >= 3:
            print(self.helpText)

        for k in range(len(self.sensors)):
            tn = self.sensors[k][0].upper()
            if tn in self.tempTables.keys():
                self.sensors[k][3] = self.tempTables[tn]
            else:
                self.sensors[k][3] = None

        return True, None

    def parseDefineName(self, ln):
        m = reDefBool.search(ln)
        if m:
            t = m.groups()
            if len(t) == 1:
                self.cfgNames.add(t[0])
            return True

        return False

    def parseDefineValue(self, ln):
        m = reDefQS.search(ln)
        if m:
            t = m.groups()
            if len(t) == 2:
                m = reDefQSm.search(ln)
                if m:
                    t = m.groups()
                    tt = re.findall(reDefQSm2, t[1])
                    if len(tt) == 1 and (t[0] in self.cfgNames):
                        self.cfgValues[t[0]] = tt[0], True
                        return True
                    elif len(tt) > 1 and (t[0] in self.cfgNames):
                        self.cfgValues[t[0]] = tt, True
                        return True

        m = reDefine.search(ln)
        if m:
            t = m.groups()
            if len(t) == 2 and (t[0] in self.cfgNames):
                if reDefineBL.search(ln):
                    self.cfgValues[t[0]] = t[1], True
                else:
                    self.cfgValues[t[0]] = t[1], False
                return True

        m = reDefBool.search(ln)
        if m:
            t = m.groups()
            # Accept booleans, but not those for which a value exists already.
            # Booleans already existing as values are most likely misconfigured
            # manual edits (or result of a bug).
            if (
                len(t) == 1
                and t[0] in self.cfgNames
                and not (
                    t[0] in self.cfgValues and isinstance(self.cfgValues[t[0]], tuple)
                )
            ):
                if reDefBoolBL.search(ln):
                    self.cfgValues[t[0]] = True
                else:
                    self.cfgValues[t[0]] = False
                return True

        return False

    def parseCandidateValues(self, ln):
        m = reCandThermPins.match(ln)
        if m:
            t = m.groups()
            if len(t) == 1:
                self.candThermPins.append(t[0])
            return True

        m = reCandHeatPins.match(ln)
        if m:
            t = m.groups()
            if len(t) == 1:
                self.candHeatPins.append(t[0])
            return True

        m = reCandProcessors.match(ln)
        if m:
            t = m.groups()
            if len(t) == 1:
                self.candProcessors.append(t[0])
            return True

        m = reCandCPUClocks.match(ln)
        if m:
            t = m.groups()
            if len(t) == 1:
                self.candClocks.append(t[0])
            return True

        m = reDefTT.match(ln)
        if m:
            t = m.groups()
            if len(t) == 2:
                s = self.parseTempTable(t[1])
                if s:
                    self.tempTables[t[0]] = s
            return True

        return False

    def parseSensor(self, s):
        m = reSensor.search(s)
        if m:
            t = m.groups()
            if len(t) == 4:
                return list(t)
        return None

    def parseHeater(self, s):
        m = reHeater5.search(s)
        if m:
            t = m.groups()
            if len(t) == 5:
                return list(t)
        # reHeater4 deprecated, for compatibility with old config files only.
        m = reHeater4.search(s)
        if m:
            t = m.groups()
            if len(t) == 4:
                t = list(t)
                t.insert(5, "100")
                return t
        # reHeater3 deprecated, for compatibility with old config files only.
        m = reHeater3.search(s)
        if m:
            t = m.groups()
            if len(t) == 3:
                t = list(t)
                t.insert(2, "0")
                t.insert(5, "100")
                return t
        # End of deprecated part.
        return None

    def parseTempTable(self, s):
        m = reTempTable4.search(s)
        if m:
            t = m.groups()
            if len(t) == 4:
                return list(t)
        m = reTempTable7.search(s)
        if m:
            t = m.groups()
            if len(t) == 7:
                return list(t)
        return None

    def saveConfigFile(self, path, values):
        if not values:
            values = self.cfgValues

        if self.settings.verbose >= 1:
            print("Saving board: %s." % path)
        if self.settings.verbose >= 2:
            print(values)

        fp = open(path, "w")
        self.configFile = path

        skipToSensorEnd = False
        skipToHeaterEnd = False
        candThermPinsWritten = False
        candHeatPinsWritten = False
        candProcessorsWritten = False
        candCPUClocksWritten = False

        for ln in self.cfgBuffer:
            m = reStartSensors.match(ln)
            if m:
                fp.write(ln)
                fp.write(
                    "//                 name      type           pin    " "additional\n"
                )
                ttString = "\n"
                ttString += "// Beta algorithm      r0      beta  r2    vadc\n"
                ttString += "// Steinhart-Hart      rp      t0    r0      t1    "
                ttString += "r1      t2    r2\n"
                for s in self.sensors:
                    sstr = "%-10s%-15s%-7s" % ((s[0] + ","), (s[1] + ","), (s[2] + ","))
                    if s[3] is None:
                        sstr += "0"
                    else:
                        sstr += "THERMISTOR_%s" % s[0].upper()
                        tt = s[3]
                        if len(tt) == 4:
                            ttString += "//TEMP_TABLE %-8s (%-8s%-6s%-6s%s)\n" % (
                                s[0].upper(),
                                (tt[0] + ","),
                                (tt[1] + ","),
                                (tt[2] + ","),
                                tt[3],
                            )
                        else:
                            ttString += (
                                "//TEMP_TABLE %-8s (%-8s%-6s%-8s%-6s%-8s%-6s%s)\n"
                                % (
                                    s[0].upper(),
                                    (tt[0] + ","),
                                    (tt[1] + ","),
                                    (tt[2] + ","),
                                    (tt[3] + ","),
                                    (tt[4] + ","),
                                    (tt[5] + ","),
                                    tt[6],
                                )
                            )
                    fp.write("DEFINE_TEMP_SENSOR(%s)\n" % sstr)
                fp.write(ttString)
                skipToSensorEnd = True
                continue

            if skipToSensorEnd:
                m = reEndSensors.match(ln)
                if m:
                    fp.write(ln)
                    skipToSensorEnd = False
                continue

            m = reStartHeaters.match(ln)
            if m:
                fp.write(ln)
                fp.write("//            name      pin      invert  pwm     max_pwm\n")
                for s in self.heaters:
                    sstr = "%-10s%-9s%-8s%-7s%s" % (
                        (s[0] + ","),
                        (s[1] + ","),
                        (s[2] + ","),
                        s[3] + ",",
                        s[4],
                    )
                    fp.write("DEFINE_HEATER(%s)\n" % sstr)
                fp.write("\n")
                for s in self.heaters:
                    fp.write(defineHeaterFormat % (s[0].upper(), s[0]))
                skipToHeaterEnd = True
                continue

            if skipToHeaterEnd:
                m = reEndHeaters.match(ln)
                if m:
                    fp.write(ln)
                    skipToHeaterEnd = False
                continue

            if reCandThermPins.match(ln):
                if not candThermPinsWritten:
                    for pin in self.candThermPins:
                        fp.write("//#define TEMP_SENSOR_PIN " + pin + "\n")
                    candThermPinsWritten = True
                continue

            if reCandHeatPins.match(ln):
                if not candHeatPinsWritten:
                    for pin in self.candHeatPins:
                        fp.write("//#define HEATER_PIN " + pin + "\n")
                    candHeatPinsWritten = True
                continue

            if reCandProcessors.match(ln):
                if not candProcessorsWritten:
                    for pin in self.candProcessors:
                        fp.write("//#define CPU_TYPE " + pin + "\n")
                    candProcessorsWritten = True
                continue

            if reCandCPUClocks.match(ln):
                if not candCPUClocksWritten:
                    for pin in self.candClocks:
                        fp.write("//#define F_CPU_OPT " + pin + "\n")
                    candCPUClocksWritten = True
                continue

            m = reDefine.match(ln)
            if m:
                t = m.groups()
                if len(t) == 2 and t[0] in values.keys():
                    v = values[t[0]]
                    self.cfgValues[t[0]] = v
                    if v[1] == False:
                        fp.write("//")
                    fp.write(defineValueFormat % (t[0], v[0]))
                else:
                    if t[0] == "RX_ENABLE_PIN" or t[0] == "TX_ENABLE_PIN":
                        # Known to be absent in the GUI, also won't be added anytime soon.
                        fp.write(ln)
                    else:
                        print("Value key " + t[0] + " not found in GUI.")

                continue

            m = reDefBoolBL.match(ln)
            if m:
                t = m.groups()
                if len(t) == 1 and t[0] in values.keys():
                    v = values[t[0]]
                    self.cfgValues[t[0]] = v
                    if v == "" or v == False:
                        fp.write("//")
                    fp.write(defineBoolFormat % t[0])
                else:
                    if t[0] == "MOTHERBOARD":
                        # Known to be absent in the GUI, also won't be added anytime soon.
                        fp.write(ln)
                    else:
                        print("Boolean key " + t[0] + " not found in GUI.")

                continue

            fp.write(ln)

        fp.close()

        return True
