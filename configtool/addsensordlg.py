
import wx
from configtool.data import (pinNames, BSIZESMALL, sensorTypes, offsetTcLabel,
                             offsetChLabel, reInteger, reFloat)
from configtool.thermistorpresets import thermistorPresets

MODE_NONTHERM = 0
MODE_THERMISTOR = 1

METHOD_BETA = 0
METHOD_SH = 1
MethodLabels = ["Beta", "Steinhart-Hart"]

labelWidth = 160


class AddSensorDlg(wx.Dialog):
  def __init__(self, parent, names, pins, heatersPage, font, name = "",
               stype = "", pin = "", params = [], modify = False):
    if modify:
      title = "Modify temperature sensor"
    else:
      title = "Add temperature sensor"
    wx.Dialog.__init__(self, parent, wx.ID_ANY, title, size = (400, 204))
    self.SetFont(font)
    self.Bind(wx.EVT_CLOSE, self.onCancel)

    self.names = names
    self.choices = pins
    self.heatersPage = heatersPage
    self.modify = modify

    if len(params) == 0:
      self.currentMethod = METHOD_BETA
      self.currentMode = MODE_NONTHERM
    else:
      self.currentMode = MODE_THERMISTOR
      if len(params) == 4:
        self.currentMethod = METHOD_BETA
      else:
        self.currentMethod = METHOD_SH

    self.nameValid = False
    self.param0Valid = False
    self.param1Valid = False
    self.param2Valid = False
    self.param3Valid = False
    self.param4Valid = False
    self.param5Valid = False
    self.param6Valid = False

    sizer = wx.BoxSizer(wx.VERTICAL)

    hsz = wx.BoxSizer(wx.HORIZONTAL)
    hsz.AddSpacer((10, 10))

    csz = wx.BoxSizer(wx.VERTICAL)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Heater Name:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)

    nameList = ["noheater"] + self.heatersPage.heaterNames()
    for alreadyDefinedName in names:
      try:
        nameList.remove(alreadyDefinedName)
      except:
        pass
    if modify:
      nameList.insert(0, name)

    if len(nameList) == 0:
      nameList = ["<no free heater name available>"]
      self.nameValid = False
    else:
      self.nameValid = True

    self.tcName = wx.Choice(self, wx.ID_ANY, choices = nameList)
    self.tcName.SetFont(font)
    self.tcName.Bind(wx.EVT_CHOICE, self.onHeaterName)
    lsz.Add(self.tcName)
    self.tcName.SetToolTipString("Choose the name of the corresponding heater. "
                                 "This may require to define that heater "
                                 "first.")
    self.tcName.SetSelection(0)

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
    st = wx.StaticText(self, wx.ID_ANY, "", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    self.label0 = st

    vals = params + ["", "", "", "", "", "", ""]
    self.param0 = wx.TextCtrl(self, wx.ID_ANY, vals[0])
    self.param0.SetFont(font)
    self.param0.Bind(wx.EVT_TEXT, self.onParam0Entry)
    lsz.Add(self.param0)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    self.label1 = st

    self.param1 = wx.TextCtrl(self, wx.ID_ANY, vals[1])
    self.param1.SetFont(font)
    self.param1.Bind(wx.EVT_TEXT, self.onParam1Entry)
    lsz.Add(self.param1)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    self.label2 = st

    self.param2 = wx.TextCtrl(self, wx.ID_ANY, vals[2])
    self.param2.SetFont(font)
    self.param2.Bind(wx.EVT_TEXT, self.onParam2Entry)
    lsz.Add(self.param2)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    self.label3 = st

    self.param3 = wx.TextCtrl(self, wx.ID_ANY, vals[3])
    self.param3.SetFont(font)
    self.param3.Bind(wx.EVT_TEXT, self.onParam3Entry)
    lsz.Add(self.param3)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    self.label4 = st

    self.param4 = wx.TextCtrl(self, wx.ID_ANY, vals[4])
    self.param4.SetFont(font)
    self.param4.Bind(wx.EVT_TEXT, self.onParam4Entry)
    lsz.Add(self.param4)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    self.label5 = st

    self.param5 = wx.TextCtrl(self, wx.ID_ANY, vals[5])
    self.param5.SetFont(font)
    self.param5.Bind(wx.EVT_TEXT, self.onParam5Entry)
    lsz.Add(self.param5)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    self.label6 = st

    self.param6 = wx.TextCtrl(self, wx.ID_ANY, vals[6])
    self.param6.SetFont(font)
    self.param6.Bind(wx.EVT_TEXT, self.onParam6Entry)
    lsz.Add(self.param6)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    csz.AddSpacer((10, 10))

    hsz.Add(csz)
    hsz.AddSpacer((10, 10))

    csz = wx.BoxSizer(wx.VERTICAL)
    csz.AddSpacer((30, 45))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Presets:",
                       size = (70, -1), style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)

    self.thermistorChoices = ["<none>"] + sorted(thermistorPresets.keys())
    ch = wx.Choice(self, wx.ID_ANY, choices = self.thermistorChoices)
    ch.SetFont(font)
    ch.Enable(False)
    ch.SetSelection(0)
    self.chPresets = ch
    ch.Bind(wx.EVT_CHOICE, self.onPresetChoice)
    lsz.Add(ch)

    csz.Add(lsz)
    csz.AddSpacer((10, 50))

    b = wx.StaticBox(self, wx.ID_ANY, "Temp Table Algorithm")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    style = wx.RB_GROUP
    self.rbMethod = []
    for k in MethodLabels:
      rb = wx.RadioButton(self, wx.ID_ANY, k, style = style)
      rb.SetFont(font)
      self.Bind(wx.EVT_RADIOBUTTON, self.onMethodSelect, rb)
      self.rbMethod.append(rb)
      style = 0

      sbox.Add(rb, 1, wx.LEFT + wx.RIGHT, 16)
      sbox.AddSpacer((5, 5))

    self.rbMethod[self.currentMethod].SetValue(True);
    csz.Add(sbox)

    hsz.Add(csz)
    hsz.AddSpacer((10, 10))

    sizer.Add(hsz)

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

    sizer.Add(bsz, flag = wx.ALIGN_CENTER_HORIZONTAL)
    sizer.AddSpacer((10, 10))

    self.SetSizer(sizer)
    self.Fit()

    self.selectSensorType(stStart)
    self.validateFields()

  def onHeaterName(self, evt):
    s = self.tcName.GetSelection()
    label = self.tcName.GetString(s)
    if label.startswith("<"):
      self.nameValid = False
    else:
      self.nameValid = True

    evt.Skip()

  def onMethodSelect(self, evt):
    rb = evt.GetEventObject()
    lbl = rb.GetLabel()
    for i in range(len(MethodLabels)):
      if lbl == MethodLabels[i]:
        self.currentMethod = i
        self.setDialogMode()
        self.validateFields()
        return

  def checkDlgValidity(self):
    if (self.nameValid and self.param0Valid and self.param1Valid and
        self.param2Valid and self.param3Valid and self.param4Valid and
        self.param5Valid and self.param6Valid):
      self.bSave.Enable(True)
    else:
      self.bSave.Enable(False)

  def onTextCtrlInteger(self, tc, rqd):
    if not rqd:
      return True
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
    if not rqd:
      return True
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

  def onParam0Entry(self, evt):
    if self.currentMode == MODE_THERMISTOR:
      self.param0Valid = self.onTextCtrlInteger(self.param0, True)
      self.checkValuesForPreset()
    else:
      self.param0Valid = True

    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onParam1Entry(self, evt):
    if self.currentMode == MODE_THERMISTOR:
      if self.currentMethod == METHOD_BETA:
        self.param1Valid = self.onTextCtrlInteger(self.param1, True)
      else:
        self.param1Valid = self.onTextCtrlFloat(self.param1, True)
      self.checkValuesForPreset()
    else:
      self.param1Valid = True

    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onParam2Entry(self, evt):
    if self.currentMode == MODE_THERMISTOR:
      self.param2Valid = self.onTextCtrlInteger(self.param2, True)
      self.checkValuesForPreset()
    else:
      self.param2Valid = True

    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onParam3Entry(self, evt):
    if self.currentMode == MODE_THERMISTOR:
      self.param3Valid = self.onTextCtrlFloat(self.param3, True)
      self.checkValuesForPreset()
    else:
      self.param3Valid = True

    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onParam4Entry(self, evt):
    if self.currentMode == MODE_THERMISTOR:
      if self.currentMethod == METHOD_BETA:
        self.param4Valid = True
      else:
        self.param4Valid = self.onTextCtrlInteger(self.param4, True)
      self.checkValuesForPreset()
    else:
      self.param4Valid = True

    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onParam5Entry(self, evt):
    if self.currentMode == MODE_THERMISTOR:
      if self.currentMethod == METHOD_BETA:
        self.param5Valid = True
      else:
        self.param5Valid = self.onTextCtrlFloat(self.param5, True)
      self.checkValuesForPreset()
    else:
      self.param5Valid = True

    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def onParam6Entry(self, evt):
    if self.currentMode == MODE_THERMISTOR:
      if self.currentMethod == METHOD_BETA:
        self.param6Valid = True
      else:
        self.param6Valid = self.onTextCtrlInteger(self.param6, True)
      self.checkValuesForPreset()
    else:
      self.param6Valid = True

    self.checkDlgValidity()
    if evt is not None:
      evt.Skip()

  def checkValuesForPreset(self):
    p = []
    p.append(self.param0.GetValue().strip())
    p.append(self.param1.GetValue().strip())
    p.append(self.param2.GetValue().strip())
    p.append(self.param3.GetValue().strip())
    if self.currentMethod != METHOD_BETA:
      p.append(self.param4.GetValue().strip())
      p.append(self.param5.GetValue().strip())
      p.append(self.param6.GetValue().strip())

    for k in thermistorPresets.keys():
      if p == thermistorPresets[k]:
        try:
          i = self.thermistorChoices.index(k)
        except:
          i = 0
        self.chPresets.SetSelection(i)
        return

    self.chPresets.SetSelection(0)

  def selectSensorType(self, lbl):
    if lbl == "Thermistor":
      self.currentMode = MODE_THERMISTOR
    else:
      self.currentMode = MODE_NONTHERM
    self.setDialogMode()

  def setDialogMode(self):
    if self.currentMode == MODE_THERMISTOR:
      if self.currentMethod == METHOD_BETA:
        self.param0.SetToolTipString("Nominal resistance of the thermistor. "
                                     "Typically 10000 ( = 10k) or 100000 "
                                     "( = 100k).")
        self.label0.SetLabel("R0:")
        self.param1.SetToolTipString("Thermistor beta value. Can be found in "
                                     "the datasheet or measured like described "
                                     "in http://reprap.org/wiki/"
                                     "MeasuringThermistorBeta")
        self.label1.SetLabel("Beta:")
        self.param2.SetToolTipString("Resistance value of the secondary "
                                     "resistor. This is not a property of the "
                                     "thermistor, but one of the board. "
                                     "Typical values are 4700 ( = 4k7 ohms) "
                                     "or 1000 ( = 1k ohms).")
        self.label2.SetLabel("R2:")
        self.param3.SetToolTipString("Comparison voltage used by the "
                                     "controller. Usually the same as the "
                                     "controller's supply voltage, 3.3 or 5.0 "
                                     "(volts).")
        self.label3.SetLabel("Vadc:")
        self.label4.SetLabel("")
        self.param4.SetToolTip(None)
        self.param4.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
        self.param4.Refresh()
        self.label5.SetLabel("")
        self.param5.SetToolTip(None)
        self.param5.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
        self.param5.Refresh()
        self.label6.SetLabel("")
        self.param6.SetToolTip(None)
        self.param6.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
        self.param6.Refresh()
        self.param4.Enable(False)
        self.param5.Enable(False)
        self.param6.Enable(False)
      else:
        self.param0.SetToolTipString("Reference resistance value.")
        self.label0.SetLabel("Rp:")
        self.param1.SetToolTipString("First data point, temperature at which "
                                     "resistance is equal to R0.")
        self.label1.SetLabel("T0:")
        self.param2.SetToolTipString("Resistance when temperature is T0.")
        self.label2.SetLabel("R0:")
        self.param3.SetToolTipString("Second data point, temperature at which "
                                     "resistance is equal to R1.")
        self.label3.SetLabel("T1:")
        self.param4.SetToolTipString("Resistance when temperature is T1.")
        self.label4.SetLabel("R1:")
        self.param5.SetToolTipString("Third data point, temperature at which "
                                     "resistance is equal to R2.")
        self.label5.SetLabel("T2:")
        self.param6.SetToolTipString("Resistance when temperature is T2.")
        self.label6.SetLabel("R2:")
        self.param4.Enable(True)
        self.param5.Enable(True)
        self.param6.Enable(True)

      self.label0.SetSize((labelWidth, -1))
      self.label0.SetWindowStyle(wx.ALIGN_RIGHT)
      self.label1.SetSize((labelWidth, -1))
      self.label1.SetWindowStyle(wx.ALIGN_RIGHT)
      self.label2.SetSize((labelWidth, -1))
      self.label2.SetWindowStyle(wx.ALIGN_RIGHT)
      self.label3.SetSize((labelWidth, -1))
      self.label3.SetWindowStyle(wx.ALIGN_RIGHT)
      self.label4.SetSize((labelWidth, -1))
      self.label4.SetWindowStyle(wx.ALIGN_RIGHT)
      self.label5.SetSize((labelWidth, -1))
      self.label5.SetWindowStyle(wx.ALIGN_RIGHT)
      self.label6.SetSize((labelWidth, -1))
      self.label6.SetWindowStyle(wx.ALIGN_RIGHT)

      self.param0.Enable(True);
      self.param1.Enable(True);
      self.param2.Enable(True);
      self.param3.Enable(True);
      self.chPresets.Enable(True);
      for rb in self.rbMethod:
        rb.Enable(True)
    else:
      self.param0.SetToolTip(None)
      self.label0.SetLabel("")
      self.param0.SetBackgroundColour(
            wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
      self.param0.Refresh()

      self.param1.SetToolTip(None)
      self.label1.SetLabel("")
      self.param1.SetBackgroundColour(
            wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
      self.param1.Refresh()

      self.param2.SetToolTip(None)
      self.label2.SetLabel("")
      self.param2.SetBackgroundColour(
            wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
      self.param2.Refresh()

      self.param3.SetToolTip(None)
      self.label3.SetLabel("")
      self.param3.SetBackgroundColour(
            wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
      self.param3.Refresh()

      self.param4.SetToolTip(None)
      self.label4.SetLabel("")
      self.param4.SetBackgroundColour(
            wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
      self.param4.Refresh()

      self.param5.SetToolTip(None)
      self.label5.SetLabel("")
      self.param5.SetBackgroundColour(
            wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
      self.param5.Refresh()

      self.param6.SetToolTip(None)
      self.label6.SetLabel("")
      self.param6.SetBackgroundColour(
            wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
      self.param6.Refresh()

      self.param0.Enable(False);
      self.param1.Enable(False);
      self.param2.Enable(False);
      self.param3.Enable(False);
      self.param4.Enable(False);
      self.param5.Enable(False);
      self.param6.Enable(False);
      self.chPresets.Enable(False);
      for rb in self.rbMethod:
        rb.Enable(False)

  def onChoice(self, evt):
    pass

  def onPresetChoice(self, evt):
    ch = evt.GetEventObject()
    s = ch.GetSelection()
    label = ch.GetString(s)
    if s != 0:
      self.param0.SetValue(thermistorPresets[label][0])
      self.param1.SetValue(thermistorPresets[label][1])
      self.param2.SetValue(thermistorPresets[label][2])
      self.param3.SetValue(thermistorPresets[label][3])
      if len(thermistorPresets[label]) == 7:
        self.param4.SetValue(thermistorPresets[label][4])
        self.param5.SetValue(thermistorPresets[label][5])
        self.param6.SetValue(thermistorPresets[label][6])
        self.currentMethod = METHOD_SH
      else:
        self.currentMethod = METHOD_BETA
      self.rbMethod[self.currentMethod].SetValue(True)
      self.setDialogMode()

    self.validateFields()
    evt.Skip()

  def onSensorType(self, evt):
    ch = evt.GetEventObject()
    s = ch.GetSelection()
    label = ch.GetString(s)

    self.selectSensorType(label)
    self.validateFields()
    evt.Skip()

  def validateFields(self):
    self.onParam0Entry(None)
    self.onParam1Entry(None)
    self.onParam2Entry(None)
    self.onParam3Entry(None)
    self.onParam4Entry(None)
    self.onParam5Entry(None)
    self.onParam6Entry(None)

  def getValues(self):
    nm = self.tcName.GetString(self.tcName.GetSelection())
    pin = self.choices[self.chPin.GetSelection()]
    stype = self.chType.GetString(self.chType.GetSelection())
    if self.currentMode == MODE_THERMISTOR:
      if self.currentMethod == METHOD_BETA:
        addtl = [str(self.param0.GetValue().strip()),
                 str(self.param1.GetValue().strip()),
                 str(self.param2.GetValue().strip()),
                 str(self.param3.GetValue().strip())]
      else:
        addtl = [str(self.param0.GetValue().strip()),
                 str(self.param1.GetValue().strip()),
                 str(self.param2.GetValue().strip()),
                 str(self.param3.GetValue().strip()),
                 str(self.param4.GetValue().strip()),
                 str(self.param5.GetValue().strip()),
                 str(self.param6.GetValue().strip())]
    else:
      addtl = None

    return (nm, sensorTypes[stype], pin, addtl)

  def onSave(self, evt):
    self.EndModal(wx.ID_OK)

  def onCancel(self, evt):
    self.EndModal(wx.ID_CANCEL)
