from __future__ import print_function

import os
import re

from sys import platform
from configtool.data import (
    defineValueFormat,
    defineBoolFormat,
    reHelpTextStart,
    reHelpTextEnd,
    reDefine,
    reDefineBL,
    reDefQS,
    reDefQSm,
    reDefQSm2,
    reDefBool,
    reDefBoolBL,
    reCandHomingOptions,
    reDefHoming,
    reHoming,
)


class Printer:
    def __init__(self, settings):
        self.settings = settings
        self.configFile = None
        self.cfgDir = os.path.join(self.settings.folder, "configtool")

        self.cfgValues = {}
        self.homing = []
        self.candHomingOptions = []
        self.homingKeys = [
            "HOMING_STEP1",
            "HOMING_STEP2",
            "HOMING_STEP3",
            "HOMING_STEP4",
        ]

    def getValues(self):
        vars = [(x, self.cfgValues[x]) for x in self.cfgValues]
        return dict(vars)

    def hasData(self):
        return self.configFile != None

    def getFileName(self):
        return self.configFile

    def loadConfigFile(self, fn):
        cfgFn = os.path.join(self.cfgDir, "printer.generic.h")
        try:
            self.cfgBuffer = list(open(cfgFn))
        except:
            return False, cfgFn

        try:
            self.userBuffer = list(open(fn))
        except:
            return False, fn

        self.configFile = fn

        self.homing = []
        self.candHomingOptions = []
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

            m = reDefHoming.search(ln)
            if m:
                t = m.groups()
                if len(t) == 1:
                    s = self.parseHoming(t[0])
                    if s:
                        self.homing = s
                        continue

        # Parsing done. All parsed stuff is now in these array and dicts.
        if self.settings.verbose >= 2:
            print(self.cfgValues)  # #defines with a value.
            print(self.cfgNames)  # Names found in the generic file.
        if self.settings.verbose >= 3:
            print(self.helpText)

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
        m = reCandHomingOptions.match(ln)
        if m:
            t = m.groups()
            if len(t) == 1:
                self.candHomingOptions.append(t[0])
            return True

    def parseHoming(self, s):
        m = re.findall(reHoming, s)
        if not m:
            return None

        for i, tag in enumerate(self.homingKeys):
            try:
                self.cfgValues[tag] = m[i], True
            except IndexError:
                self.cfgValues[tag] = "none", False

        return m

    def saveConfigFile(self, path, values):
        if not values:
            values = self.cfgValues

        if self.settings.verbose >= 1:
            print("Saving printer: %s." % path)
        if self.settings.verbose >= 2:
            print(values)

        fp = open(path, "w")
        self.configFile = path

        homingWritten = False

        for ln in self.cfgBuffer:
            if reDefHoming.match(ln):
                if not homingWritten:
                    home = []
                    for h in self.homingKeys:
                        home.append(values[h][0])
                    while "none" in home:
                        home.remove("none")
                    if not home:
                        home = ["none"]
                    homing_str = "DEFINE_HOMING({})\n".format(", ".join(home))
                    fp.write(homing_str)
                    homingWritten = True
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
                    if t[0] == "CANNED_CYCLE":
                        # Known to be absent in the GUI. Worse, this value is replaced
                        # by the one in the metadata file.
                        #
                        # TODO: make value reading above recognize wether this value is
                        #       commented out or not. Reading the value its self works
                        #       already. Hint: it's the rule using reDefQS, reDefQSm, etc.
                        #
                        # TODO: add a multiline text field in the GUI to deal with this.
                        #
                        # TODO: write this value out properly. In /* comments */, if
                        #       disabled.
                        #
                        # TODO: currently, the lines beyond the ones with the #define are
                        #       treated like arbitrary comments. Having the former TODOs
                        #       done, this will lead to duplicates.
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
                    print("Boolean key " + t[0] + " not found in GUI.")

                continue

            fp.write(ln)

        fp.close()

        return True
