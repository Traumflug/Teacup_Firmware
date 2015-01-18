
import wx
from configtool.data import pinNames, BSIZESMALL, sensorTypes


class AddSensorDlg(wx.Dialog):
  def __init__(self, parent, names, pins, font):
    wx.Dialog.__init__(self, parent, wx.ID_ANY, "Add temperature sensor",
                       size = (400, 204))
    self.SetFont(font)
    self.Bind(wx.EVT_CLOSE, self.onCancel)

    self.names = names
    self.choices = pins

    labelWidth = 160

    self.nameValid = False

    hsz = wx.BoxSizer(wx.HORIZONTAL)
    hsz.AddSpacer((10, 10))

    csz = wx.BoxSizer(wx.VERTICAL)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Sensor Name:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st)

    self.tcName = wx.TextCtrl(self, wx.ID_ANY, "")
    self.tcName.SetFont(font)
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
    lsz.Add(st)

    sl = sorted(sensorTypes.keys())

    ch = wx.Choice(self, wx.ID_ANY, choices = sl)
    ch.SetFont(font)
    ch.Bind(wx.EVT_CHOICE, self.onSensorType)
    ch.SetSelection(0)
    self.chType = ch
    lsz.Add(ch)

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Pin:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st)

    self.choiceList = pinNames
    self.chPin = wx.Choice(self, wx.ID_ANY, choices = pins)
    self.chPin.SetFont(font)
    self.chPin.Bind(wx.EVT_CHOICE, self.onChoice)
    self.chPin.SetSelection(0)
    lsz.Add(self.chPin)
    self.chPin.SetToolTipString("Choose a pin name for this sensor.")

    csz.Add(lsz)
    csz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Additional:", size = (labelWidth, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st)

    self.tcAddtl = wx.TextCtrl(self, wx.ID_ANY, "")
    self.tcAddtl.SetFont(font)
    self.tcAddtl.Bind(wx.EVT_TEXT, self.onAddtlEntry)
    self.selectSensorType(sl[0])
    lsz.Add(self.tcAddtl)
    self.tcAddtl.SetToolTipString("Enter additional information required "
                                  "by the sensor type.")

    csz.Add(lsz)

    csz.AddSpacer((10,10))

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

  def onNameEntry(self, evt):
    tc = evt.GetEventObject()
    w = tc.GetValue().strip()
    if w == "":
      self.nameValid = False
    else:
      if w in self.names:
        self.nameValid = False
      else:
        self.nameValid = True

    if self.nameValid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()

    self.checkDlgValidity()
    evt.Skip()

  def checkDlgValidity(self):
    if self.nameValid:
      self.bSave.Enable(True)
    else:
      self.bSave.Enable(False)

  def onAddtlEntry(self, evt):
    evt.Skip()

  def selectSensorType(self, lbl):
    if lbl == 'Thermistor':
      self.tcAddtl.Enable(True);
    else:
      self.tcAddtl.Enable(False);

  def onChoice(self, evt):
    pass

  def onSensorType(self, evt):
    ch = evt.GetEventObject()
    s = ch.GetSelection()
    label = ch.GetString(s)

    self.selectSensorType(label)

    evt.Skip()

  def getValues(self):
    nm = self.tcName.GetValue()
    pin = self.choices[self.chPin.GetSelection()]
    addtl = self.tcAddtl.GetValue()
    stype = self.chType.GetString(self.chType.GetSelection())

    if stype in ['Thermistor']:
      return (nm, sensorTypes[stype], pin, addtl)
    else:
      return (nm, sensorTypes[stype], pin)


  def onSave(self, evt):
    self.EndModal(wx.ID_OK)

  def onCancel(self, evt):
    self.EndModal(wx.ID_CANCEL)
