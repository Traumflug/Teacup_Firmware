
import wx
from configtool.data import BSIZE
from configtool.page import Page
from configtool.calcbelt import CalcBelt
from configtool.calcscrew import CalcScrew


class MechanicalPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.id = idPg
    self.parent = parent
    self.font = font

    self.spmKeys = ['STEPS_PER_M_X', 'STEPS_PER_M_Y', 'STEPS_PER_M_Z',
                    'STEPS_PER_M_E']

    self.mfrKeys = ['MAXIMUM_FEEDRATE_X', 'MAXIMUM_FEEDRATE_Y',
                    'MAXIMUM_FEEDRATE_Z', 'MAXIMUM_FEEDRATE_E']

    self.msrKeys = ['SEARCH_FEEDRATE_X', 'SEARCH_FEEDRATE_Y',
                    'SEARCH_FEEDRATE_Z']

    self.eclKeys = ['ENDSTOP_CLEARANCE_X', 'ENDSTOP_CLEARANCE_Y',
                    'ENDSTOP_CLEARANCE_Z']

    self.minmaxKeys = ['X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX', 'Z_MIN', 'Z_MAX']

    self.kinematicsKeys = ['KINEMATICS_STRAIGHT', 'KINEMATICS_COREXY']

    self.labels = {'STEPS_PER_M_X': "X:", 'STEPS_PER_M_Y': "Y:",
                   'STEPS_PER_M_Z': "Z:", 'STEPS_PER_M_E' : "E:",
                   'MAXIMUM_FEEDRATE_X': "X:", 'MAXIMUM_FEEDRATE_Y': "Y:",
                   'MAXIMUM_FEEDRATE_Z': "Z:", 'MAXIMUM_FEEDRATE_E': "E:",
                   'SEARCH_FEEDRATE_X': "X:", 'SEARCH_FEEDRATE_Y': "Y:",
                   'SEARCH_FEEDRATE_Z': "Z:",
                   'ENDSTOP_CLEARANCE_X': "X:", 'ENDSTOP_CLEARANCE_Y': "Y:",
                   'ENDSTOP_CLEARANCE_Z': "Z:",
                   'X_MIN': "Min X:", 'X_MAX': "Max X:", 'Y_MIN': "Min Y:",
                   'Y_MAX': "Max Y:", 'Z_MIN': "Min Z:", 'Z_MAX': "Max Z:",
                   'E_ABSOLUTE': "Absolute E Coordinates",
                   'KINEMATICS_STRAIGHT': "Straight",
                   'KINEMATICS_COREXY': "CoreXY"}

    labelWidth = 40;

    sz = wx.GridBagSizer()
    sz.AddSpacer((10, 10), pos = (0, 0))
    sz.AddSpacer((90, 10), pos = (0, 4))

    b = wx.StaticBox(self, wx.ID_ANY, "Steps Per Meter")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.spmKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "Maximum Feedrate")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.mfrKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 5))

    b = wx.StaticBox(self, wx.ID_ANY, "Search Feedrate")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.msrKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 7))

    b = wx.StaticBox(self, wx.ID_ANY, "Endstop Clearance")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.eclKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (3, 5))

    b = wx.StaticBox(self, wx.ID_ANY, "Travel Limits")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.minmaxKeys:
      tc = self.addTextCtrl(k, labelWidth + 20, self.onTextCtrlFloat)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (3, 7))

    vsz = wx.BoxSizer(wx.VERTICAL)

    b = wx.StaticBox(self, wx.ID_ANY, "Kinematics")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    style = wx.RB_GROUP
    for k in self.kinematicsKeys:
      rb = self.addRadioButton(k, style, self.onKinematicsSelect, b)
      style = 0

      sbox.Add(rb, 1, wx.LEFT + wx.RIGHT, 16)
      sbox.AddSpacer((5, 5))

    vsz.Add(sbox, 1, wx.LEFT, 40)

    cb = self.addCheckBox('E_ABSOLUTE', self.onCheckBox)

    vsz.Add(cb, 1, wx.LEFT, 40)

    sz.Add(vsz, pos = (3, 1))

    bsz = wx.BoxSizer(wx.VERTICAL)
    b = wx.Button(self, wx.ID_ANY, "Calculate\nBelt Driven", size = BSIZE)
    b.SetBackgroundColour(self.deco.getBackgroundColour())
    b.SetFont(font)
    b.SetToolTipString("Open the calculator for axes that are belt-driven.")
    self.Bind(wx.EVT_BUTTON, self.onCalcBelt, b)
    self.bCalcBelt = b

    bsz.Add(b, 1, wx.ALL, 5)
    b = wx.Button(self, wx.ID_ANY, "Calculate\nScrew Driven", size = BSIZE)
    b.SetBackgroundColour(self.deco.getBackgroundColour())
    b.SetFont(font)
    bsz.Add(b, 1, wx.ALL, 5)
    b.SetToolTipString("Open the calculator for axes that are screw-driven.")
    self.Bind(wx.EVT_BUTTON, self.onCalcScrew, b)
    self.bCalcScrew = b

    sz.Add(bsz, pos = (1, 3))
    self.enableAll(False)
    self.SetSizer(sz)

  def enableAll(self, flag = True):
    self.bCalcBelt.Enable(flag)
    self.bCalcScrew.Enable(flag)
    Page.enableAll(self, flag)

  def onKinematicsSelect(self, evt):
    self.assertModified(True)
    evt.Skip()

  def onCalcBelt(self, evt):
    dlg = CalcBelt(self, self.font, self.cbCalcBelt)
    dlg.ShowModal()
    dlg.Destroy()

  def cbCalcBelt(self, field, value):
    s = "%d" % value
    self.textControls[field].SetValue(s)

  def onCalcScrew(self, evt):
    dlg = CalcScrew(self, self.font, self.cbCalcScrew)
    dlg.ShowModal()
    dlg.Destroy()

  def cbCalcScrew(self, field, value):
    s = "%d" % value
    self.textControls[field].SetValue(s)

  def setHelpText(self, ht):
    Page.setHelpText(self, ht)
    if 'KINEMATICS' in ht.keys():
      for k in self.kinematicsKeys:
        self.radioButtons[k].SetToolTipString(ht['KINEMATICS'])

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    k = 'KINEMATICS'
    if k in cfgValues.keys():
      k = cfgValues[k]
      if k in self.kinematicsKeys:
        self.radioButtons[k].SetValue(True)
      else:
        self.radioButtons[self.kinematicsKeys[0]].SetValue(True)
    else:
      self.radioButtons[self.kinematicsKeys[0]].SetValue(True)

  def getValues(self):
    result = Page.getValues(self)

    for tag in self.kinematicsKeys:
      rb = self.radioButtons[tag]
      if rb.GetValue():
        result['KINEMATICS'] = tag, True
        break

    return result
