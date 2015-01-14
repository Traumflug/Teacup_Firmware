
import wx
from configtool.data import pinNames, BSIZESMALL


class AddSensorDlg(wx.Dialog):
  def __init__(self, parent, names, sl, pins):
    wx.Dialog.__init__(self, parent, wx.ID_ANY, "Add temperature sensor",
                       size = (400, 204))
    self.Bind(wx.EVT_CLOSE, self.onCancel)

    self.names = names

    self.nameValid = False

    sz = wx.BoxSizer(wx.VERTICAL)

    csz = wx.GridBagSizer()
    csz.AddSpacer((10, 10), pos = (0, 0))

    b = wx.StaticBox(self, wx.ID_ANY, "Sensor Type")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    style = wx.RB_GROUP
    self.rbs = {}
    for k in sl:
      rb = wx.RadioButton(self, wx.ID_ANY, k, style = style)
      self.rbs[k] = rb
      self.Bind(wx.EVT_RADIOBUTTON, self.onSensorType, rb)
      style = 0

      sbox.Add(rb, 1, wx.LEFT + wx.RIGHT, 16)
      sbox.AddSpacer((5, 5))

    csz.Add(sbox, pos = (1, 3))

    vsz = wx.BoxSizer(wx.VERTICAL)
    vsz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Sensor Name:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    self.tcName = wx.TextCtrl(self, wx.ID_ANY, "")
    self.tcName.SetBackgroundColour("pink")
    self.tcName.Bind(wx.EVT_TEXT, self.onNameEntry)
    lsz.Add(self.tcName)
    self.tcName.SetToolTipString("Enter a unique name for this sensor.")

    vsz.Add(lsz)
    vsz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Pin:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    self.choiceList = pinNames
    self.chPin = wx.Choice(self, wx.ID_ANY, choices = pins)
    self.chPin.Bind(wx.EVT_CHOICE, self.onChoice)
    self.chPin.SetSelection(0)
    lsz.Add(self.chPin)
    self.chPin.SetToolTipString("Choose a pin name for this sensor.")

    vsz.Add(lsz)
    vsz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Additional:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    self.tcAddtl = wx.TextCtrl(self, wx.ID_ANY, "")
    self.tcAddtl.Bind(wx.EVT_TEXT, self.onAddtlEntry)
    self.selectSensorType(sl[0])
    lsz.Add(self.tcAddtl)
    self.tcAddtl.SetToolTipString("Enter additional information required "
                                  "by the sensor type.")

    vsz.Add(lsz)
    csz.Add(vsz, pos = (1, 1))

    csz.AddSpacer((10, 10), pos = (1, 4))

    sz.Add(csz)
    sz.AddSpacer((30, 30))

    bsz = wx.BoxSizer(wx.HORIZONTAL)

    self.bSave = wx.Button(self, wx.ID_ANY, "Save", size = BSIZESMALL)
    self.bSave.Bind(wx.EVT_BUTTON, self.onSave)
    bsz.Add(self.bSave)
    self.bSave.Enable(False)

    bsz.AddSpacer(30, 100)

    self.bCancel = wx.Button(self, wx.ID_ANY, "Cancel", size = BSIZESMALL)
    self.bCancel.Bind(wx.EVT_BUTTON, self.onCancel)
    bsz.Add(self.bCancel)

    sz.Add(bsz, flag = wx.ALIGN_CENTER_HORIZONTAL)
    self.SetSizer(sz)

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
    if lbl == 'TT_THERMISTOR':
      self.tcAddtl.Enable(True);
    else:
      self.tcAddtl.Enable(False);

  def onChoice(self, evt):
    pass

  def onSensorType(self, evt):
    rb = evt.GetEventObject()
    label = rb.GetLabel()

    self.selectSensorType(label)

    evt.Skip()

  def getValues(self):
    nm = self.tcName.GetValue()
    pin = pinNames[self.chPin.GetSelection()]
    addtl = self.tcAddtl.GetValue()

    for k in self.rbs:
      if self.rbs[k].GetValue():
        stype = k
        break

    if stype is None:
      stype = "??"

    if stype in ['TT_THERMISTOR']:
      return (nm, stype, pin, addtl)
    else:
      return (nm, stype, pin)


  def onSave(self, evt):
    self.EndModal(wx.ID_OK)

  def onCancel(self, evt):
    self.EndModal(wx.ID_CANCEL)
