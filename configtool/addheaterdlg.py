
import wx
from configtool.data import BSIZESMALL, offsetChLabel, offsetTcLabel


class AddHeaterDlg(wx.Dialog):
  def __init__(self, parent, names, pins, font,
               name = "", pin = "", invert = "0", pwm = "1"):
    wx.Dialog.__init__(self, parent, wx.ID_ANY, "Add heater", size = (400, 204))
    self.SetFont(font)
    self.Bind(wx.EVT_CLOSE, self.onCancel)

    self.names = names
    self.choices = pins

    self.nameValid = (name != "")

    sz = wx.BoxSizer(wx.VERTICAL)
    gsz = wx.GridBagSizer()
    gsz.AddSpacer((20, 20), pos = (0, 0))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Heater Name:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)

    self.tcName = wx.TextCtrl(self, wx.ID_ANY, name)
    self.tcName.SetFont(font)
    if not name:
      self.tcName.SetBackgroundColour("pink")
    self.tcName.Bind(wx.EVT_TEXT, self.onNameEntry)
    lsz.Add(self.tcName)
    self.tcName.SetToolTipString("Enter a unique name for this heater.")

    gsz.Add(lsz, pos = (1, 1))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Pin:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetChLabel)

    self.chPin = wx.Choice(self, wx.ID_ANY, choices = pins)
    self.chPin.SetFont(font)
    self.chPin.Bind(wx.EVT_CHOICE, self.onChoice)
    i = self.chPin.FindString(pin)
    if i == wx.NOT_FOUND:
      self.chPin.SetSelection(0)
    else:
      self.chPin.SetSelection(i)
    lsz.Add(self.chPin)
    self.chPin.SetToolTipString("Choose a pin for this heater.")

    gsz.Add(lsz, pos = (3, 1))

    self.cbInv = wx.CheckBox(self, wx.ID_ANY, "Invert")
    self.cbInv.SetFont(font)
    self.cbInv.SetValue(int(invert) != 0)
    self.cbInv.SetToolTipString("Invert the pin signal.")

    gsz.Add(self.cbInv, pos = (3, 3))

    self.cbPwm = wx.CheckBox(self, wx.ID_ANY, "PWM")
    self.cbPwm.SetFont(font)
    self.cbPwm.SetValue(int(pwm) != 0)
    self.cbPwm.SetToolTipString("Use Pulse Width Modulation in case the "
                                "choosen pin allows to do so.")

    gsz.AddSpacer((50, 15), pos = (1, 2))
    gsz.Add(self.cbPwm, pos = (1, 3))
    gsz.AddSpacer((20, 20), pos = (4, 4))

    sz.Add(gsz)
    sz.AddSpacer((30, 30))

    bsz = wx.BoxSizer(wx.HORIZONTAL)

    self.bSave = wx.Button(self, wx.ID_ANY, "Save", size = BSIZESMALL)
    self.bSave.SetFont(font)
    self.bSave.Bind(wx.EVT_BUTTON, self.onSave)
    bsz.Add(self.bSave)

    bsz.AddSpacer(30, 100)

    self.bCancel = wx.Button(self, wx.ID_ANY, "Cancel", size = BSIZESMALL)
    self.bCancel.SetFont(font)
    self.bCancel.Bind(wx.EVT_BUTTON, self.onCancel)
    bsz.Add(self.bCancel)

    sz.Add(bsz, flag = wx.ALIGN_CENTER_HORIZONTAL)
    sz.AddSpacer((10, 10))
    self.SetSizer(sz)

    self.checkDlgValidity()
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

  def onChoice(self, evt):
    pass

  def checkDlgValidity(self):
    if self.nameValid:
      self.bSave.Enable(True)
    else:
      self.bSave.Enable(False)

  def getValues(self):
    nm = self.tcName.GetValue()
    pin = self.choices[self.chPin.GetSelection()]

    if self.cbInv.IsChecked():
      invert = "1"
    else:
      invert = "0"

    if self.cbPwm.IsChecked():
      pwm = "1"
    else:
      pwm = "0"

    return (nm, pin, invert, pwm)

  def onSave(self, evt):
    self.EndModal(wx.ID_OK)

  def onCancel(self, evt):
    self.EndModal(wx.ID_CANCEL)
