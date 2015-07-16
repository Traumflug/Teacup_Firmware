
import wx
from configtool.page import Page
from configtool.data import reFloat


class MiscellaneousPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.id = idPg
    self.font = font

    self.labels = {'USE_INTERNAL_PULLUPS': "Use Internal Pullups",
                   'EECONFIG': "Enable EEPROM Storage",
                   'BANG_BANG': "Enable",
                   'BANG_BANG_ON': "On PWM Level:",
                   'BANG_BANG_OFF': "Off PWM Level:",
                   'REPORT_TARGET_TEMPS': "Report Target Temperatures",
                   'MOVEBUFFER_SIZE': "Movebuffer Size:",
                   'DC_EXTRUDER': "Heater:", 'DC_EXTRUDER_PWM': "PWM:",
                   'USE_WATCHDOG': "Use the Watchdog Timer",
                   'TH_COUNT': "Temperature History Size:",
                   'FAST_PWM': "Fast PWM",
                   'ENDSTOP_STEPS': "Endstop Steps:",
                   'PID_SCALE': "PID Scaling Factor:",
                   'TEMP_HYSTERESIS': "Temperature Hysteresis:",
                   'TEMP_RESIDENCY_TIME': "Temperature Residency Time:",
                   'TEMP_EWMA': "Temperature EWMA:",
                   'HEATER_SANITY_CHECK': "Heater Sanity Check"}

    self.heaterNameNone = "<none>"
    self.heaterNames = [self.heaterNameNone]
    self.boardHeaters = []
    self.processors = []

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))
    sz.AddSpacer((40, 40), pos = (0, 2))
    sz.AddSpacer((40, 40), pos = (0, 4))
    sz.AddSpacer((20, 30), pos = (1, 0))
    sz.AddSpacer((20, 30), pos = (2, 0))
    sz.AddSpacer((20, 30), pos = (3, 0))
    sz.AddSpacer((20, 30), pos = (4, 0))
    sz.AddSpacer((20, 30), pos = (5, 0))
    sz.AddSpacer((20, 30), pos = (6, 0))
    sz.AddSpacer((20, 30), pos = (7, 0))
    sz.AddSpacer((20, 30), pos = (8, 0))

    labelWidth = 140

    k = 'EECONFIG'
    cb = self.addCheckBox(k, self.onCheckBox)
    sz.Add(cb, pos = (1, 1))

    k = 'USE_INTERNAL_PULLUPS'
    cb = self.addCheckBox(k, self.onCheckBox)
    sz.Add(cb, pos = (2, 1))

    k = 'USE_WATCHDOG'
    cb = self.addCheckBox(k, self.onCheckBox)
    sz.Add(cb, pos = (3, 1))

    k = 'FAST_PWM'
    cb = self.addCheckBox(k, self.onCheckBox)
    sz.Add(cb, pos = (4, 1))

    k = 'HEATER_SANITY_CHECK'
    cb = self.addCheckBox(k, self.onCheckBox)
    sz.Add(cb, pos = (5, 1))

    k = 'REPORT_TARGET_TEMPS'
    cb = self.addCheckBox(k, self.onCheckBox)
    sz.Add(cb, pos = (6, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "BANG BANG Bed Control")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))

    k = 'BANG_BANG'
    cb = self.addCheckBox(k, self.onCheckBox)
    sbox.Add(cb, 1, wx.LEFT, 60)
    sbox.AddSpacer((5, 20))

    k = 'BANG_BANG_ON'
    tc = self.addTextCtrl(k, 100, self.onTextCtrlInteger)
    sbox.Add(tc)
    sbox.AddSpacer((5, 5))

    k = 'BANG_BANG_OFF'
    tc = self.addTextCtrl(k, 100, self.onTextCtrlInteger)
    sbox.Add(tc)
    sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 3), span = (5, 1), flag = wx.ALIGN_CENTER_HORIZONTAL)

    b = wx.StaticBox(self, wx.ID_ANY, "DC Motor Extruder")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))

    k = 'DC_EXTRUDER'
    ch = self.addChoice(k, self.heaterNames, 0, 60, self.onChoice)
    sbox.Add(ch)
    sbox.AddSpacer((5, 5))

    k = 'DC_EXTRUDER_PWM'
    tc = self.addTextCtrl(k, 60, self.onTextCtrlInteger)
    sbox.Add(tc)
    sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (6, 3), span=(3, 1), flag = wx.ALIGN_CENTER_HORIZONTAL)

    labelWidth = 190;

    k = 'MOVEBUFFER_SIZE'
    tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
    sz.Add(tc, pos = (1, 5))

    k = 'TH_COUNT'
    tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
    sz.Add(tc, pos = (2, 5))

    k = 'ENDSTOP_STEPS'
    tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
    sz.Add(tc, pos = (3, 5))

    k = 'PID_SCALE'
    tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
    sz.Add(tc, pos = (4, 5))

    k = 'TEMP_HYSTERESIS'
    tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
    sz.Add(tc, pos = (6, 5))

    k = 'TEMP_RESIDENCY_TIME'
    tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
    sz.Add(tc, pos = (7, 5))

    k = 'TEMP_EWMA'
    tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlEWMA)
    sz.Add(tc, pos = (8, 5))

    self.SetSizer(sz)
    self.enableAll(False)

  def onTextCtrlEWMA(self, evt):
    self.assertModified(True)
    tc = evt.GetEventObject()
    name = tc.GetName()
    w = tc.GetValue().strip()
    if w == "":
      valid = True
    else:
      m = reFloat.match(w)
      if m:
        v = float(w)
        if v < 0.1 or v > 1.0:
          valid = False
        else:
          valid = True
      else:
        valid = False

    self.setFieldValidity(name, valid)

    if valid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()
    evt.Skip()

  def setHeaters(self, hlist):
    k = 'DC_EXTRUDER'
    v = self.choices[k].GetSelection()
    currentChoice = self.heaterNames[v]
    self.boardHeaters = [s[0] for s in hlist]
    self.heaterNames = [self.heaterNameNone] + self.boardHeaters
    self.choices[k].Clear()
    self.choices[k].SetFont(self.font)
    for h in self.heaterNames:
      self.choices[k].Append(h)

    try:
      v = self.heaterNames.index(currentChoice)
    except:
      v = 0
      dlg = wx.MessageDialog(self,
                             "Printer: Miscellaneous tab:\nDC Extruder heater "
                             "\"%s\" not defined for this board. Please check."
                             % currentChoice, "Warning",
                             wx.OK + wx.ICON_WARNING)

      dlg.ShowModal()
      dlg.Destroy()

    self.choices[k].SetSelection(v)

  def setOriginalHeater(self, h):
    k = 'DC_EXTRUDER'
    if h and h.startswith("HEATER_"):
      hname = h[len("HEATER_"):]
    else:
      hname = h
    if hname and len(self.boardHeaters) != 0:
      if hname not in self.boardHeaters:
        dlg = wx.MessageDialog(self,
                               "Printer: Miscellaneous tab:\nDC Extruder "
                               "heater \"%s\" not defined for this board. "
                               "Please check."
                               % hname, "Warning", wx.OK + wx.ICON_WARNING)

        dlg.ShowModal()
        dlg.Destroy()
      self.heaterNames = [self.heaterNameNone] + self.boardHeaters
    else:
      self.heaterNames = [self.heaterNameNone]
      if hname and hname != self.heaterNameNone:
        self.heaterNames.append(hname)
    self.choices[k].Clear()
    self.choices[k].SetFont(self.font)
    for ht in self.heaterNames:
      self.choices[k].Append(ht)
    if hname:
      try:
        v = self.heaterNames.index(hname)
      except:
        v = 0
    else:
      v = 0
    self.choices[k].SetSelection(v)

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    for k in self.choices.keys():
      if k in cfgValues.keys():
        self.choicesOriginal[k] = cfgValues[k]

  def getValues(self):
    result = Page.getValues(self)

    k = 'DC_EXTRUDER'
    s = self.choices[k].GetSelection()
    v = self.choices[k].GetString(s)
    if v == self.heaterNameNone:
      if k in self.choicesOriginal.keys():
        result[k] = self.choicesOriginal[k][0], False
      else:
        result[k] = "", False
    else:
      result[k] = "HEATER_%s" % v, True

    return result
