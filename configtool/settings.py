
import ConfigParser
import os
import wx
from configtool.data import BSIZESMALL, offsetTcLabel

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

  def saveSettings(self):
    self.section = "configtool"
    try:
      self.cfg.add_section(self.section)
    except ConfigParser.DuplicateSectionError:
      pass

    self.cfg.set(self.section, "arduinodir", str(self.arduinodir))
    self.cfg.set(self.section, "cflags", str(self.cflags))
    self.cfg.set(self.section, "ldflags", str(self.ldflags))
    self.cfg.set(self.section, "objcopyflags", str(self.objcopyflags))
    self.cfg.set(self.section, "programmer", str(self.programmer))
    self.cfg.set(self.section, "port", str(self.port))

    try:
      cfp = open(self.inifile, 'wb')
    except:
      print "Unable to open settings file %s for writing." % self.inifile
      return
    self.cfg.write(cfp)
    cfp.close()


ARDUINODIR = 0
CFLAGS = 1
LDFLAGS = 2
OBJCOPYFLAGS= 3
PROGRAMMER = 4
PORT = 5

class SettingsDlg(wx.Dialog):
  def __init__(self, parent, settings):
    wx.Dialog.__init__(self, parent, wx.ID_ANY, "Modify settings",
                       size = (500, 300))
    self.SetFont(settings.font)
    self.settings = settings

    self.modified = False

    self.Bind(wx.EVT_CLOSE, self.onExit)

    self.fields = [["Arduino Directory", settings.arduinodir, None],
                   ["C Compiler Flags", settings.cflags, None],
                   ["LD Flags", settings.ldflags, None],
                   ["Object Copy Flags", settings.objcopyflags, None],
                   ["AVR Programmer", settings.programmer, None],
                   ["Port", settings.port, None]]

    hsz = wx.BoxSizer(wx.HORIZONTAL)
    hsz.AddSpacer((10, 10))

    sz = wx.BoxSizer(wx.VERTICAL)
    sz.AddSpacer((10, 10))

    labelWidth = 140
    for f in self.fields:
      lsz = wx.BoxSizer(wx.HORIZONTAL)
      t = wx.StaticText(self, wx.ID_ANY, f[0], size = (labelWidth, -1),
                        style = wx.ALIGN_RIGHT)
      t.SetFont(settings.font)
      lsz.Add(t, 1, wx.TOP, offsetTcLabel)

      lsz.AddSpacer((8, 8))

      te = wx.TextCtrl(self, wx.ID_ANY, f[1], size = (600, -1))
      te.Bind(wx.EVT_TEXT, self.onTextCtrl)
      lsz.Add(te)
      f[2] = te

      sz.Add(lsz)
      sz.AddSpacer((10, 10))

    sz.AddSpacer((20, 20))

    bsz = wx.BoxSizer(wx.HORIZONTAL)
    b = wx.Button(self, wx.ID_ANY, "Save", size = BSIZESMALL)
    b.SetFont(settings.font)
    self.Bind(wx.EVT_BUTTON, self.onSave, b)
    bsz.Add(b)
    self.bSave = b
    bsz.AddSpacer((5, 5))

    b = wx.Button(self, wx.ID_ANY, "Exit", size = BSIZESMALL)
    b.SetFont(settings.font)
    self.Bind(wx.EVT_BUTTON, self.onExit, b)
    bsz.Add(b)
    self.bExit = b

    sz.Add(bsz, 1, wx.ALIGN_CENTER_HORIZONTAL)
    sz.AddSpacer((10, 10))

    hsz.Add(sz)
    hsz.AddSpacer((10, 10))

    self.SetSizer(hsz)
    self.setModified(False)

    self.Fit()

  def setModified(self, flag):
    self.modified = flag
    if flag:
      self.bSave.Enable(True)
      self.bExit.SetLabel("Cancel")
    else:
      self.bSave.Enable(False)
      self.bExit.SetLabel("Exit")

  def onTextCtrl(self, evt):
    self.setModified(True)
    evt.Skip()

  def onSave(self, evt):
    self.saveValues()
    self.EndModal(wx.ID_OK)

  def saveValues(self):
    self.settings.arduinodir = self.fields[ARDUINODIR][2].GetValue()
    self.settings.cflags = self.fields[CFLAGS][2].GetValue()
    self.settings.ldflags = self.fields[LDFLAGS][2].GetValue()
    self.settings.objcopyflags = self.fields[OBJCOPYFLAGS][2].GetValue()
    self.settings.programmer = self.fields[PROGRAMMER][2].GetValue()
    self.settings.port = self.fields[PORT][2].GetValue()

    self.settings.saveSettings()

  def onExit(self, evt):
    if not self.confirmLoseChanges("exit"):
      return
    self.EndModal(wx.ID_EXIT)

  def confirmLoseChanges(self, msg):
    if not self.modified:
      return True

    dlg = wx.MessageDialog(self, "Are you sure you want to " + msg + "?\n"
                                 "There are changes to your settings that "
                                 "will be lost.",
                           "Changes pending",
                           wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
    rc = dlg.ShowModal()
    dlg.Destroy()

    if rc != wx.ID_YES:
      return False

    return True
