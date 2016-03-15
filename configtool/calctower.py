
import wx
from configtool.data import (BSIZESMALL, reFloat, reInteger, offsetChLabel,
                             offsetTcLabel)
import math

class CalcTower(wx.Dialog):
  def __init__(self, parent, font, cbUse):
    wx.Dialog.__init__(self, parent, wx.ID_ANY,
                       "Tower trigonometry constants calculator",
                       size = (360, 300))
    self.SetFont(font)
    self.Bind(wx.EVT_CLOSE, self.onExit)

    self.use = cbUse
    labelWidth = 130

    hsz = wx.BoxSizer(wx.HORIZONTAL)
    hsz.AddSpacer((10, 10))

    sz = wx.BoxSizer(wx.VERTICAL)
    sz.AddSpacer((10, 10))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Tower X Angle:",
                       size = (labelWidth, -1), style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    lsz.AddSpacer((5, 5))

    tc = wx.TextCtrl(self, wx.ID_ANY, "210", style = wx.TE_RIGHT)
    tc.SetFont(font)
    tc.Bind(wx.EVT_TEXT, self.onTextCtrlFloat)
    lsz.Add(tc)
    self.tcXAngle = tc

    sz.Add(lsz)
    sz.AddSpacer((10, 10))
    
    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Tower Y Angle:",
                       size = (labelWidth, -1), style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    lsz.AddSpacer((5, 5))

    tc = wx.TextCtrl(self, wx.ID_ANY, "330", style = wx.TE_RIGHT)
    tc.SetFont(font)
    tc.Bind(wx.EVT_TEXT, self.onTextCtrlFloat)
    lsz.Add(tc)
    self.tcYAngle = tc

    sz.Add(lsz)
    sz.AddSpacer((10, 10))
    
    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Tower Z Angle:",
                       size = (labelWidth, -1), style = wx.ALIGN_RIGHT)
    st.SetFont(font)
    lsz.Add(st, 1, wx.TOP, offsetTcLabel)
    lsz.AddSpacer((5, 5))

    tc = wx.TextCtrl(self, wx.ID_ANY, "90", style = wx.TE_RIGHT)
    tc.SetFont(font)
    tc.Bind(wx.EVT_TEXT, self.onTextCtrlFloat)
    lsz.Add(tc)
    self.tcZAngle = tc

    sz.Add(lsz)
    sz.AddSpacer((30, 30))

    bsz = wx.BoxSizer(wx.HORIZONTAL)

    b = wx.Button(self, wx.ID_ANY, "Calculate", size = BSIZESMALL)
    b.SetFont(font)
    self.Bind(wx.EVT_BUTTON, self.onCalculate, b)
    bsz.Add(b)
    self.bCalculate = b

    sz.Add(bsz)
    sz.AddSpacer((10, 10))

    hsz.Add(sz)
    hsz.AddSpacer((10, 10))

    self.enableUseButtons(False)

    self.SetSizer(hsz)

    self.Fit()

    self.calculate()


  def calculate(self):
    self.enableUseButtons(False)

    try:
      xAngle = float(self.tcXAngle.GetValue())
    except:
      return
      
    try:
      yAngle = float(self.tcYAngle.GetValue())
    except:
      return
    
    try:
      zAngle = float(self.tcZAngle.GetValue())
    except:
      return
      
    self.cosX = float(math.cos(math.radians(xAngle)))
    self.sinX = float(math.sin(math.radians(xAngle)))
    self.cosY = float(math.cos(math.radians(yAngle)))
    self.sinY = float(math.sin(math.radians(yAngle)))
    self.cosZ = float(math.cos(math.radians(zAngle)))
    self.sinZ = float(math.sin(math.radians(zAngle)))

    self.enableUseButtons(True)

  def enableUseButtons(self, flag):
    self.bCalculate.Enable(flag)

  def onCalculate(self, evt):
    self.use('COS_X', self.cosX)
    self.use('SIN_X', self.sinX)
    self.use('COS_Y', self.cosY)
    self.use('SIN_Y', self.sinY)
    self.use('COS_Z', self.cosZ)
    self.use('SIN_Z', self.sinZ)

  def onPresetChoice(self, evt):
    s = self.tcPresets.GetSelection()
    sv = self.beltPresetValues[s]
    if sv < 0:
      return

    s = "%f" % sv
    s = s.rstrip("0")
    if s[-1] == ".":
      s += "0"
    self.tcBeltPitch.SetValue(s)

  def onChoice(self, evt):
    self.calculate()

  def onTextCtrlInteger(self, evt):
    tc = evt.GetEventObject()
    w = tc.GetValue().strip()
    if w == "":
      valid = False
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
    self.calculate()
    evt.Skip()

  def onTextCtrlFloat(self, evt):
    tc = evt.GetEventObject()
    w = tc.GetValue().strip()
    if w == "":
      valid = False
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
    self.calculate()
    evt.Skip()

  def onExit(self, evt):
    self.EndModal(wx.ID_OK)
