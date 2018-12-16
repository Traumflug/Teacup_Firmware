
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

    self.homingKeys = [('HOMING_STEP1', 2), ('HOMING_STEP2', 2),
                       ('HOMING_STEP3', 2), ('HOMING_STEP4', 2)]

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
                   'KINEMATICS_COREXY': "CoreXY",
                   'HOMING_STEP1': "Step 1:",
                   'HOMING_STEP2': "Step 2:",
                   'HOMING_STEP3': "Step 3:",
                   'HOMING_STEP4': "Step 4:",
                   'none': "-",
                   'x_negative': "X min",
                   'x_positive': "X max",
                   'y_negative': "Y min",
                   'y_positive': "Y max",
                   'z_negative': "Z min",
                   'z_positive': "Z max"}

    labelWidth = 40;
    labelWidthHoming = 60;

    sz = wx.GridBagSizer()
    sz.Add((10, 10), pos = (0, 0))
    sz.Add((90, 10), pos = (0, 4))

    b = wx.StaticBox(self, wx.ID_ANY, "Steps Per Meter")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.Add((5, 5))
    for k in self.spmKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.Add((5, 5))

    sz.Add(sbox, pos = (1, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "Maximum Feedrate")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.Add((5, 5))
    for k in self.mfrKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.Add((5, 5))

    sz.Add(sbox, pos = (1, 5))

    b = wx.StaticBox(self, wx.ID_ANY, "Search Feedrate")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.Add((5, 5))
    for k in self.msrKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.Add((5, 5))

    sz.Add(sbox, pos = (1, 7))

    b = wx.StaticBox(self, wx.ID_ANY, "Endstop Clearance")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.Add((5, 5))
    for k in self.eclKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.Add((5, 5))

    sz.Add(sbox, pos = (3, 5))

    b = wx.StaticBox(self, wx.ID_ANY, "Travel Limits")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.Add((5, 5))
    for k in self.minmaxKeys:
      tc = self.addTextCtrl(k, labelWidth + 20, self.onTextCtrlFloat)
      sbox.Add(tc)
      sbox.Add((5, 5))

    sz.Add(sbox, pos = (3, 7))

    vsz = wx.BoxSizer(wx.VERTICAL)

    b = wx.StaticBox(self, wx.ID_ANY, "Kinematics")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.Add((5, 5))
    style = wx.RB_GROUP
    for k in self.kinematicsKeys:
      rb = self.addRadioButton(k, style, self.onKinematicsSelect, b)
      style = 0

      sbox.Add(rb, 1, wx.LEFT + wx.RIGHT, 16)
      sbox.Add((5, 5))

    vsz.Add(sbox, 1, wx.LEFT, 10)

    cb = self.addCheckBox('E_ABSOLUTE', self.onCheckBox)

    vsz.Add(cb, 1, wx.LEFT, 10)

    sz.Add(vsz, pos = (3, 1))

    bsz = wx.BoxSizer(wx.VERTICAL)
    b = wx.Button(self, wx.ID_ANY, "Calculate\nBelt Driven", size = BSIZE)
    b.SetBackgroundColour(self.deco.getBackgroundColour())
    b.SetFont(font)
    b.SetToolTip("Open the calculator for axes that are belt-driven.")
    self.Bind(wx.EVT_BUTTON, self.onCalcBelt, b)
    self.bCalcBelt = b

    bsz.Add(b, 1, wx.ALL, 5)
    b = wx.Button(self, wx.ID_ANY, "Calculate\nScrew Driven", size = BSIZE)
    b.SetBackgroundColour(self.deco.getBackgroundColour())
    b.SetFont(font)
    bsz.Add(b, 1, wx.ALL, 5)
    b.SetToolTip("Open the calculator for axes that are screw-driven.")
    self.Bind(wx.EVT_BUTTON, self.onCalcScrew, b)
    self.bCalcScrew = b

    sz.Add(bsz, pos = (1, 3))

    b = wx.StaticBox(self, wx.ID_ANY, "Homing Order")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.Add((5, 5))
    for k, ctype in self.homingKeys:
      if ctype == 2:
        tc = self.addChoice(k, [], 0, labelWidthHoming, self.onChoice)
        sbox.Add(tc)
      sbox.Add((5, 5))
    sz.Add(sbox, pos = (3, 3))

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
        self.radioButtons[k].SetToolTip(ht['KINEMATICS'])

  def prepareHomingValues(self, name, i, cfgValues):
    self.choices[name].Clear()
    for p in cfgValues['HOMING_OPTS']:
      self.choices[name].Append(self.labels[p])
    i = cfgValues['HOMING_OPTS'].index(cfgValues[name])
    self.choices[name].SetSelection(i)

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    self.prepareHomingValues('HOMING_STEP1', 0, cfgValues)
    self.prepareHomingValues('HOMING_STEP2', 1, cfgValues)
    self.prepareHomingValues('HOMING_STEP3', 2, cfgValues)
    self.prepareHomingValues('HOMING_STEP4', 3, cfgValues)

    for tag in self.kinematicsKeys:
      if tag in cfgValues.keys() and cfgValues[tag]:
        self.radioButtons[tag].SetValue(True)

  def getHomingValue(self, name, result):
    return (self.labels.keys()[self.labels.values().index(result[name][0])], True)

  def getValues(self):
    result = Page.getValues(self)

    for tag in self.kinematicsKeys:
      result[tag] = self.radioButtons[tag].GetValue()

    result['HOMING_STEP1'] = self.getHomingValue('HOMING_STEP1', result)
    result['HOMING_STEP2'] = self.getHomingValue('HOMING_STEP2', result)
    result['HOMING_STEP3'] = self.getHomingValue('HOMING_STEP3', result)
    result['HOMING_STEP4'] = self.getHomingValue('HOMING_STEP4', result)

    return result
