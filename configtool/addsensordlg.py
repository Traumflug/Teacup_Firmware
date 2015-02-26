
import wx
from configtool.data import (pinNames, BSIZESMALL, sensorTypes, offsetTcLabel,
                             offsetChLabel, reInteger, reFloat)


class AddSensorDlg(wx.Dialog):
  def __init__(self, parent, names, pins, font, name = "", stype = "",
               pin = "", r0 = "", beta = "", r2 = "", vadc = "",
               modify = False):
    if modify:
      title = "Modify temperature sensor"
    else:
      title = "Add temperature sensor"
    wx.Dialog.__init__(self, parent, wx.ID_ANY, title, size = (400, 204))
    self.SetFont(font)
    self.Bind(wx.EVT_CLOSE, self.onCancel)

    self.names = names
    self.choices = pins
    self.modify = modify

    labelWidth = 160

    self.nameValid = False
    self.R0Valid = False
    self.betaValid = False
    self.R2Valid = False
    self.vadcValid = False

    hsz = wx.BoxSizer(wx.HORIZONTAL)
    hsz.AddSpacer((10, 10))

    csz = wx.BoxSizer(wx.VERTICAL)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Sensor Name:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)

    self.tcName = wx.TextCtrl(self, wx.ID_ANY, name)
    self.tcName.SetFont(font)
    if not modify:
      self.tcName.SetBackgroundColour("pink")
    self.tcName.Bind(wx.EVT_TEXT, self.onNameEntry)
    lsz.Add(self.tcName)
    self.tcName.SetToolTipString("Enter a unique name for this sensor.")

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Sensor Type:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetChLabel)

    sl = sorted(sensorTypes.keys())

    ch = wx.Choice(self, wx.ID_ANY, choices = sl)
    ch.SetFont(font)
    ch.Bind(wx.EVT_CHOICE, self.onSensorType)
    found = False
    for st in sensorTypes.keys():
      if sensorTypes[st] == stype:
        i = ch.FindString(st)
        if i != wx.NOT_FOUND:
          stStart = st
          ch.SetSelection(i)
          found = True
          break

    if not found:
      ch.SetSelection(0)
      stStart = sl[0]
    self.chType = ch
    lsz.Add(ch)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Pin:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetChLabel)

    self.choiceList = pinNames
    self.chPin = wx.Choice(self, wx.ID_ANY, choices = pins)
    self.chPin.SetFont(font)
    self.chPin.Bind(wx.EVT_CHOICE, self.onChoice)
    i = self.chPin.FindString(pin)
    if i == wx.NOT_FOUND:
      self.chPin.SetSelection(0)
    else:
      self.chPin.SetSelection(i)
    lsz.Add(self.chPin)
    self.chPin.SetToolTipString("Choose a pin name for this sensor.")

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "R0:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)

    self.tcR0 = wx.TextCtrl(self, wx.ID_ANY, r0)
    self.tcR0.SetFont(font)
    self.tcR0.Bind(wx.EVT_TEXT, self.onR0Entry)
    lsz.Add(self.tcR0)
    self.tcR0.SetToolTipString("Nominal resistance of the thermistor. "
                               "Typically 10000 ( = 10k) or 100000 ( = 100k).")
        
    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Beta:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)

    self.tcBeta = wx.TextCtrl(self, wx.ID_ANY, beta)
    self.tcBeta.SetFont(font)
    self.tcBeta.Bind(wx.EVT_TEXT, self.onBetaEntry)
    lsz.Add(self.tcBeta)
    self.tcBeta.SetToolTipString("Thermistor beta value. Can be found in the "
                                 "datasheet or measured like described in http"
                                 "://reprap.org/wiki/MeasuringThermistorBeta")

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "R2:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)

    self.tcR2 = wx.TextCtrl(self, wx.ID_ANY, r2)
    self.tcR2.SetFont(font)
    self.tcR2.Bind(wx.EVT_TEXT, self.onR2Entry)
    lsz.Add(self.tcR2)
    self.tcR2.SetToolTipString("Resistance value of the secondary resistor. "
                               "This is not a property of the thermistor, but "
                               "one of the board. Typical values are 4700 "
                               "( = 4k7 ohms) or 1000 ( = 1k ohms).")

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Vadc:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)

    self.tcVadc = wx.TextCtrl(self, wx.ID_ANY, vadc)
    self.tcVadc.SetFont(font)
    self.tcVadc.Bind(wx.EVT_TEXT, self.onVadcEntry)
    lsz.Add(self.tcVadc)
    self.tcVadc.SetToolTipString("Comparison voltage used by the controller. "
                                 "Usually the same as the controller's supply "
                                 "voltage, 3.3 or 5.0 (volts).")

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    bsz = wx.BoxSizer(wx.HORIZONTAL)

    self.bSave = wx.Button(self, wx.ID_ANY, "Save", size = BSIZESMALL)
    self.bSave.SetFont(font)
    self.bSave.Bind(wx.EVT_BUTTON, self.onSave)
    bsz.Add(self.bSave)
    self.bSave.Enable(False)

    bsz.AddSpacer((30, 10))

    self.bCancel = wx.Button(self, wx.ID_ANY, "Cancel", size = BSIZESMALL)
    self.bCancel.SetFont(font)
    self.bCancel.Bind(wx.EVT_BUTTON, self.onCancel)
    bsz.Add(self.bCancel)

    csz.Add(bsz, flag = wx.ALIGN_CENTER_HORIZONTAL)
    csz.AddSpacer((10, 10))

    hsz.Add(csz)
    hsz.AddSpacer((10, 10))

    self.SetSizer(hsz)
    self.Fit()

    self.selectSensorType(stStart)
    self.validateFields()

  def onNameEntry(self, evt):
    tc = evt.GetEventObject()
    self.validateName(tc)
    self.checkDlgValidity()
    evt.Skip()

  def validateName(self, tc):
    w = tc.GetValue().strip()
    if w == "":
      self.nameValid = False
    else:
      if w in self.names and not self.modify:
        self.nameValid = False
      else:
        self.nameValid = True

    if self.nameValid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()

  def checkDlgValidity(self):
    if (self.nameValid and self.R0Valid and self.betaValid and
        self.R2Valid and self.vadcValid):
      self.bSave.Enable(True)
    else:
      self.bSave.Enable(False)

  def onTextCtrlInteger(self, tc, rqd):
    w = tc.GetValue().strip()
    if w == "":
      if rqd:
        valid = False
      else:
        valid = True
    else:
      m = reInteger.match(w)
      if m:
        valid = True
      else:
        valid = False
        
    if valid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")

    tc.Refresh()
    return valid
    
  def onTextCtrlFloat(self, tc, rqd):
    w = tc.GetValue().strip()
    if w == "":
      if rqd:
        valid = False
      else:
        valid = True
    else:
      m = reFloat.match(w)
      if m:
        valid = True
      else:
        valid = False
        
    if valid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()
    return valid

  def onR0Entry(self, evt):
    stype = self.chType.GetString(self.chType.GetSelection())
    if stype in ['Thermistor']:
      rqd = True
    else:
      rqd = False
    self.R0Valid = self.onTextCtrlInteger(self.tcR0, rqd)
    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onBetaEntry(self, evt):
    stype = self.chType.GetString(self.chType.GetSelection())
    if stype in ['Thermistor']:
      rqd = True
    else:
      rqd = False
    self.betaValid = self.onTextCtrlInteger(self.tcBeta, rqd)
    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onR2Entry(self, evt):
    stype = self.chType.GetString(self.chType.GetSelection())
    if stype in ['Thermistor']:
      rqd = True
    else:
      rqd = False
    self.R2Valid = self.onTextCtrlInteger(self.tcR2, rqd)
    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onVadcEntry(self, evt):
    stype = self.chType.GetString(self.chType.GetSelection())
    if stype in ['Thermistor']:
      rqd = True
    else:
      rqd = False
    self.vadcValid = self.onTextCtrlFloat(self.tcVadc, rqd)
    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def selectSensorType(self, lbl):
    if lbl == 'Thermistor':
      flag = True
    else:
      flag = False

    self.tcR0.Enable(flag);
    self.tcBeta.Enable(flag);
    self.tcR2.Enable(flag);
    self.tcVadc.Enable(flag);

  def onChoice(self, evt):
    pass

  def onSensorType(self, evt):
    ch = evt.GetEventObject()
    s = ch.GetSelection()
    label = ch.GetString(s)

    self.selectSensorType(label)
    self.validateFields()
    evt.Skip()

  def validateFields(self):
    self.validateName(self.tcName)
    self.onR0Entry(None)
    self.onBetaEntry(None)
    self.onR2Entry(None)
    self.onVadcEntry(None)

  def getValues(self):
    nm = self.tcName.GetValue()
    pin = self.choices[self.chPin.GetSelection()]
    stype = self.chType.GetString(self.chType.GetSelection())

    if stype in ['Thermistor']:
      addtl = [self.tcR0.GetValue().strip(), self.tcBeta.GetValue().strip(),
               self.tcR2.GetValue().strip(), self.tcVadc.GetValue().strip()]
    else:
      addtl = "NONE"

    return (nm, sensorTypes[stype], pin, addtl)

  def onSave(self, evt):
    self.EndModal(wx.ID_OK)

  def onCancel(self, evt):
    self.EndModal(wx.ID_CANCEL)
