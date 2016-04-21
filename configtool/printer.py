
import os
import re

from sys import platform
from configtool.data import (defineValueFormat, defineBoolFormat,
                             reHelpTextStart, reHelpTextEnd,
                             reDefine, reDefineBL, reDefQS, reDefQSm,
                             reDefQSm2, reDefBool, reDefBoolBL)

class Printer:
  def __init__(self):
    self.configFile = None

    self.cfgValues = {}

  def hasData(self):
    return (self.configFile != None)

  def getFileName(self):
    return self.configFile

  def loadConfigFile(self, cfgDir, fn):
    cfgFn = os.path.join(cfgDir, "printer.generic.h")
    try:
      self.cfgBuffer = list(open(cfgFn))
    except:
      return False, cfgFn

    try:
      self.userBuffer = list(open(fn))
    except:
      return False, fn

    self.configFile = fn

    gatheringHelpText = False
    helpTextString = ""
    helpKey = None

    self.cfgValues = {}
    self.cfgNames = []
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

      if self.parseDefineName(ln):
        continue

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

      if self.parseDefineValue(ln):
        continue

    # Parsing done. All parsed stuff is now in these array and dicts.
    # Uncomment for debugging.
    #print self.cfgValues  # #defines with a value and booleans.
    #print self.cfgNames   # Names found in the generic file.
    #print self.helpText

    return True, None

  def parseDefineName(self, ln):
    m = reDefBool.search(ln)
    if m:
      t = m.groups()
      if len(t) == 1:
        self.cfgNames.append(t[0])
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
      if len(t) == 1 and (t[0] in self.cfgNames):
        if reDefBoolBL.search(ln):
          self.cfgValues[t[0]] = True
        else:
          self.cfgValues[t[0]] = False
        return True

    return False


  def saveConfigFile(self, path, values):
    fp = file(path, 'w')
    self.configFile = path

    for ln in self.cfgBuffer:
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
          if t[0] == 'CANNED_CYCLE':
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
            print "Value key " + t[0] + " not found in GUI."

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
          print "Boolean key " + t[0] + " not found in GUI."

        continue

      fp.write(ln)

    fp.close()

    return True
