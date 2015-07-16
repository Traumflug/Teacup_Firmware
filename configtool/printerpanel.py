
import os
import wx
import re

from sys import platform
from configtool.decoration import Decoration
from configtool.data import (defineValueFormat, defineBoolFormat, reCommDefBL,
                             reCommDefBoolBL, reHelpTextStart, reHelpTextEnd,
                             reDefine, reDefineBL, reDefQS, reDefQSm,
                             reDefQSm2, reDefBool, reDefBoolBL)
from configtool.mechanicalpage import MechanicalPage
from configtool.accelerationpage import AccelerationPage
from configtool.miscellaneouspage import MiscellaneousPage
from configtool.protectedfiles import protectedFiles


class PrinterPanel(wx.Panel):
  def __init__(self, parent, nb, settings):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    self.parent = parent

    self.deco = Decoration()
    self.configFile = None
    self.protFileLoaded = False

    self.settings = settings

    self.cfgValues = {}
    self.heaters = []
    self.dir = os.path.join(self.settings.folder, "config")
    self.cfgDir = os.path.join(self.settings.folder, "configtool")

    self.SetBackgroundColour(self.deco.getBackgroundColour())
    self.Bind(wx.EVT_PAINT, self.deco.onPaintBackground)
    sz = wx.BoxSizer(wx.HORIZONTAL)

    self.nb = wx.Notebook(self, wx.ID_ANY, size = (21, 21),
                          style = wx.BK_DEFAULT)
    self.nb.SetBackgroundColour(self.deco.getBackgroundColour())
    self.nb.SetFont(self.settings.font)

    self.pages = []
    self.titles = []
    self.pageModified = []
    self.pageValid = []

    self.pgMech = MechanicalPage(self, self.nb, len(self.pages),
                                 self.settings.font)
    text = "Mechanical"
    self.nb.AddPage(self.pgMech, text)
    self.pages.append(self.pgMech)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    self.pgAcc = AccelerationPage(self, self.nb, len(self.pages),
                                  self.settings.font)
    text = "Acceleration"
    self.nb.AddPage(self.pgAcc, text)
    self.pages.append(self.pgAcc)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    self.pgMiscellaneous = MiscellaneousPage(self, self.nb, len(self.pages),
                                             self.settings.font)
    text = "Miscellaneous"
    self.nb.AddPage(self.pgMiscellaneous, text)
    self.pages.append(self.pgMiscellaneous)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    sz.Add(self.nb, 1, wx.EXPAND + wx.ALL, 5)

    self.SetSizer(sz)
    self.Fit()

  def getFileName(self):
    return self.configFile

  def assertModified(self, pg, flag = True):
    self.pageModified[pg] = flag
    self.modifyTab(pg)

  def isModified(self):
    return (True in self.pageModified)

  def isValid(self):
    return not (False in self.pageValid)

  def hasData(self):
    return (self.configFile != None)

  def assertValid(self, pg, flag = True):
    self.pageValid[pg] = flag
    self.modifyTab(pg)

    if False in self.pageValid:
      self.parent.enableSavePrinter(False, False)
    else:
      self.parent.enableSavePrinter(not self.protFileLoaded, True)

  def modifyTab(self, pg):
    if self.pageModified[pg] and not self.pageValid[pg]:
      pfx = "?* "
    elif self.pageModified[pg]:
      pfx = "* "
    elif not self.pageValid[pg]:
      pfx = "? "
    else:
      pfx = ""

    self.nb.SetPageText(pg, pfx + self.titles[pg])
    if True in self.pageModified and False in self.pageValid:
      pfx = "?* "
    elif True in self.pageModified:
      pfx = "* "
    elif False in self.pageValid:
      pfx = "? "
    else:
      pfx = ""
    self.parent.setPrinterTabDecor(pfx)

  def setHeaters(self, ht):
    return self.pgMiscellaneous.setHeaters(ht)

  def onClose(self, evt):
    if not self.confirmLoseChanges("exit"):
      return

    self.Destroy()

  def confirmLoseChanges(self, msg):
    if True not in self.pageModified:
      return True

    dlg = wx.MessageDialog(self, "Are you sure you want to " + msg + "?\n"
                                 "There are changes to your printer "
                                 "configuration that will be lost.",
                           "Changes pending",
                           wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
    rc = dlg.ShowModal()
    dlg.Destroy()

    if rc != wx.ID_YES:
      return False

    return True

  def onLoadConfig(self, evt):
    if not self.confirmLoseChanges("load a new printer configuration"):
      return

    if platform == "darwin":
      # Mac OS X appears to be a bit limited on wildcards.
      wildcard = "Printer configuration (printer.*.h)|*.h"
    else:
      wildcard = "Printer configuration (printer.*.h)|printer.*.h"

    dlg = wx.FileDialog(self, message = "Choose a printer config file",
                        defaultDir = self.dir, defaultFile = "",
                        wildcard = wildcard, style = wx.OPEN | wx.CHANGE_DIR)

    path = None
    if dlg.ShowModal() == wx.ID_OK:
      path = dlg.GetPath()

    dlg.Destroy()
    if path is None:
      return

    self.dir = os.path.dirname(path)
    rc, efn = self.loadConfigFile(path)

    if not rc:
      dlg = wx.MessageDialog(self, "Unable to process file %s." % efn,
                             "File error", wx.OK + wx.ICON_ERROR)
      dlg.ShowModal()
      dlg.Destroy()
      return

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

    self.processors = []
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

    if os.path.basename(fn) in protectedFiles:
      self.parent.enableSavePrinter(False, True)
      self.protFileLoaded = True
    else:
      self.protFileLoaded = False
      self.parent.enableSavePrinter(True, True)
    self.parent.setPrinterTabFile(os.path.basename(fn))

    for pg in self.pages:
      pg.insertValues(self.cfgValues)
      pg.setHelpText(self.helpText)

    k = 'DC_EXTRUDER'
    if k in self.cfgValues.keys() and self.cfgValues[k][1] == True:
      self.pgMiscellaneous.setOriginalHeater(self.cfgValues[k][0])
    else:
      self.pgMiscellaneous.setOriginalHeater(None)

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

    m = reDefBoolBL.search(ln)
    if m:
      t = m.groups()
      if len(t) == 1 and (t[0] in self.cfgNames):
        self.cfgValues[t[0]] = True
        return True

    return False

  def onSaveConfig(self, evt):
    path = self.configFile
    return self.saveConfigFile(path)

  def onSaveConfigAs(self, evt):
    wildcard = "Printer configuration (printer.*.h)|printer.*.h"

    dlg = wx.FileDialog(self, message = "Save as ...", defaultDir = self.dir,
                        defaultFile = "", wildcard = wildcard,
                        style = wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT)

    val = dlg.ShowModal()

    if val != wx.ID_OK:
      dlg.Destroy()
      return

    path = dlg.GetPath()
    dlg.Destroy()

    rc = self.saveConfigFile(path)
    if rc:
      self.parent.setPrinterTabFile(os.path.basename(path))
      self.protFileLoaded = False
      self.parent.enableSavePrinter(True, True)
    return rc

  def saveConfigFile(self, path):
    if os.path.basename(path) in protectedFiles:
      dlg = wx.MessageDialog(self, "It's not allowed to overwrite files "
                             "distributed by Teacup. Choose another name.",
                             "Protected file error", wx.OK + wx.ICON_ERROR)
      dlg.ShowModal()
      dlg.Destroy()
      return False

    if not os.path.basename(path).startswith("printer."):
      dlg = wx.MessageDialog(self, "Illegal file name: %s.\n"
                             "File name must begin with \"printer.\"" % path,
                             "Illegal file name", wx.OK + wx.ICON_ERROR)
      dlg.ShowModal()
      dlg.Destroy()
      return False

    ext = os.path.splitext(os.path.basename(path))[1]
    self.dir = os.path.dirname(path)

    if ext == "":
      path += ".h"

    try:
      fp = file(path, 'w')
    except:
      dlg = wx.MessageDialog(self, "Unable to write to file %s." % path,
                             "File error", wx.OK + wx.ICON_ERROR)
      dlg.ShowModal()
      dlg.Destroy()
      return False

    self.configFile = path

    values = {}

    for pg in self.pages:
      v1 = pg.getValues()
      for k in v1.keys():
        values[k] = v1[k]

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
