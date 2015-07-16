
import wx
from configtool.page import Page


class AccelerationPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.id = idPg

    self.accTypeKeys = ['ACCELERATION_REPRAP', 'ACCELERATION_RAMPING',
                        'ACCELERATION_TEMPORAL']
    self.jerkKeys = ['MAX_JERK_X', 'MAX_JERK_Y', 'MAX_JERK_Z', 'MAX_JERK_E']

    self.labels = {'ACCELERATION_REPRAP': "RepRap",
                   'ACCELERATION_RAMPING': "Ramping",
                   'ACCELERATION_TEMPORAL': "Temporal",
                   'ACCELERATION': "Acceleration:",
                   'LOOKAHEAD': "Look Ahead",
                   'MAX_JERK_X': "X:", 'MAX_JERK_Y': "Y:", 'MAX_JERK_Z': "Z:",
                   'MAX_JERK_E': "E:"}

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))
    b = wx.StaticBox(self, wx.ID_ANY, "Acceleration Type")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    style = wx.RB_GROUP
    for k in self.accTypeKeys:
      rb = self.addRadioButton(k, style, self.onAccTypeSelect, b)
      style = 0

      sbox.Add(rb, 1, wx.LEFT + wx.RIGHT, 16)
      sbox.AddSpacer((5, 5))

    self.rbNone = wx.RadioButton(self, wx.ID_ANY, "None", style = style)
    self.rbNone.SetFont(font)
    self.rbNone.SetValue(True)
    self.Bind(wx.EVT_RADIOBUTTON, self.onAccTypeSelect, self.rbNone)
    sbox.Add(self.rbNone, 1, wx.LEFT + wx.RIGHT, 16)
    sbox.AddSpacer((5, 5))
    sz.Add(sbox, pos = (1, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "Ramping Parameters")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))

    k = 'ACCELERATION'
    tc = self.addTextCtrl(k, 80, self.onTextCtrlFloat)
    self.textControls[k].Enable(False)

    sbox.Add(tc)
    sbox.AddSpacer((5, 5))

    k = 'LOOKAHEAD'
    cb = self.addCheckBox(k, self.onCheckBox)
    self.checkBoxes[k].Enable(False)

    sbox.Add(cb, 1, wx.ALIGN_CENTER_HORIZONTAL)
    sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 3))

    b = wx.StaticBox(self, wx.ID_ANY, "Maximum Jerk")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.jerkKeys:
      tc = self.addTextCtrl(k, 40, self.onTextCtrlInteger)

      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.AddSpacer((80, 20), pos = (1, 4))
    sz.Add(sbox, pos = (1, 5))

    self.SetSizer(sz)
    self.enableAll(False)

  def enableAll(self, flag = True):
    self.rbNone.Enable(flag)
    Page.enableAll(self, flag)

  def onAccTypeSelect(self, evt):
    self.assertModified(True)
    rb = evt.GetEventObject()
    label = rb.GetLabel()

    if label == self.labels['ACCELERATION_RAMPING']:
      ena = True
    else:
      ena = False

    self.checkBoxes['LOOKAHEAD'].Enable(ena)
    self.textControls['ACCELERATION'].Enable(ena)
    evt.Skip()

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    self.checkBoxes['LOOKAHEAD'].Enable(False)
    self.textControls['ACCELERATION'].Enable(False)
    for tag in self.accTypeKeys:
      if tag in cfgValues.keys() and cfgValues[tag]:
        self.radioButtons[tag].SetValue(True)
        if tag == 'ACCELERATION_RAMPING':
          self.checkBoxes['LOOKAHEAD'].Enable(True)
          self.textControls['ACCELERATION'].Enable(True)

  def getValues(self):
    result = Page.getValues(self)

    for tag in self.accTypeKeys:
      result[tag] = self.radioButtons[tag].GetValue()

    return result
