#!/bin/env python

import wx
import re

try:
  from agw import customtreectrl as CT
except ImportError:
  import wx.lib.agw.customtreectrl as CT

from config_helptext import helpText

BSIZE = (90, 60)


class MyFrame(wx.Frame):

  def __init__(self):
    self.reDefine = re.compile("\s*#define\s+(\S+)\s+(\S+)")
    self.reDefQS = re.compile("\s*#define\s+(\S+)\s+(\"[^\"]*\")")
    self.reDefTS = re.compile("\s*#define\s+(DEFINE_TEMP_SENSOR\\([^)]*\\))")
    self.reDefHT = re.compile("\s*#define\s+(DEFINE_HEATER\\([^)]*\\))")
    self.reDefBool = re.compile("\s*#define\s+(\S+)\s+")

    self.t = 0
    self.seq = 1
    wx.Frame.__init__(self, None, -1, "Teacup Firmware Configurator",
                      size = (450, 600))
    self.Bind(wx.EVT_CLOSE, self.onClose)

    self.cfgValues = {}

    panel = wx.Panel(self, -1)

    sz = wx.BoxSizer(wx.HORIZONTAL)
    bsz = wx.BoxSizer(wx.VERTICAL)

    b = wx.Button(panel, wx.ID_ANY, "Load\nConfig", size = BSIZE)
    panel.Bind(wx.EVT_BUTTON, self.onLoadConfig, b)
    bsz.Add(b)

    style = wx.SUNKEN_BORDER | wx.WANTS_CHARS
    agwStyle = CT.TR_HAS_BUTTONS | CT.TR_HAS_VARIABLE_ROW_HEIGHT | CT.TR_ALIGN_WINDOWS

    self.tree = CT.CustomTreeCtrl(panel, wx.ID_ANY, wx.DefaultPosition,
                                  (100, 100), style, agwStyle)
    self.tree.Bind(CT.EVT_TREE_ITEM_CHECKED, self.onItemChecked)
    self.tree.Bind(CT.EVT_TREE_ITEM_GETTOOLTIP, self.onToolTip)
    self.root = self.tree.AddRoot("Teacup Configuration")

    self.stMech = self.mechSubTree(self.tree, self.root)
    self.stAcc = self.accSubTree(self.tree, self.root)

    pins    = self.tree.AppendItem(self.root, "3. Pinouts",
                                   data = "Pinouts")
    sensors = self.tree.AppendItem(self.root, "4. Temperature Sensors",
                                   data = "Sensors")
    heaters = self.tree.AppendItem(self.root, "5. Heaters",
                                   data = "Heaters")
    comm    = self.tree.AppendItem(self.root, "6. Communications",
                                   data = "Communications")
    misc    = self.tree.AppendItem(self.root, "7. Miscellaneous",
                                   data = "Miscellaneous")

    sz.Add(self.tree, 1, wx.EXPAND + wx.ALL, 5)
    sz.Add(bsz, 0, wx.ALL, 5)

    panel.SetSizer(sz)

  def mechSubTree(self, tree, root):
    spmKeys = ['SMX', 'SMY', 'SMZ', 'SME']

    mfrKeys = ['MFRX', 'MFRY', 'MFRZ', 'MFRE']

    msrKeys = ['MSRX', 'MSRY', 'MSRZ']

    eclKeys = ['ECX', 'ECY', 'ECZ']

    minmaxKeys = ['MINX', 'MAXX', 'MINY', 'MAXY', 'MINZ', 'MAXZ']

    mechLabels = {'SPM': "Steps/Meter",
                  'SMX': "X:", 'SMY': "Y:", 'SMZ': "Z:", 'SME' : "E:",
                  'MFR': "Max Feed Rate",
                  'MFRX': "X:", 'MFRY': "Y:", 'MFRZ': "Z:", 'MFRE': "E:",
                  'MSR': "Search Feed Rate",
                  'MSRX': "X:", 'MSRY': "Y:", 'MSRZ': "Z:",
                  'ECL': "Endstop Clearance",
                  'ECX': "X:", 'ECY': "Y:", 'ECZ': "Z:",
                  'MINMAX': "Minimum/Maximum X/Y/Z",
                  'MINX': "Min X:", 'MAXX': "Max X:", 'MINY': "Min Y:",
                  'MAXY': "Max Y:", 'MINZ': "Min Z:", 'MAXZ': "Max Z:",
                  'ABSE': "Absolute E Coordinates"}

    self.mechCfgKeys = {'SMX': "STEPS_PER_M_X", 'SMY': "STEPS_PER_M_Y",
                        'SMZ': "STEPS_PER_M_Z", 'SME' : "STEPS_PER_M_E",
                        'MFRX': "MAXIMUM_FEEDRATE_X",
                        'MFRY': "MAXIMUM_FEEDRATE_Y",
                        'MFRZ': "MAXIMUM_FEEDRATE_Z",
                        'MFRE': "MAXIMUM_FEEDRATE_E",
                        'MSRX': "SEARCH_FEEDRATE_X",
                        'MSRY': "SEARCH_FEEDRATE_Y",
                        'MSRZ': "SEARCH_FEEDRATE_Z",
                        'ECX': "ENDSTOP_CLEARANCE_X",
                        'ECY': "ENDSTOP_CLEARANCE_Y",
                        'ECZ': "ENDSTOP_CLEARANCE_Z",
                        'MINX': "X_MIN", 'MAXX': "X_MAX", 'MINY': "Y_MIN",
                        'MAXY': "Y_MAX", 'MINZ': "Z_MIN", 'MAXZ': "Z_MAX",
                        'ABSE': "E_ABSOLUTE"}

    self.tcMech = {}
    self.brMech = {}
    mech = tree.AppendItem(root, "1. Mechanical/Hardware", data = "Mechanical")

    for tag, tbl, cttype in [('SPM', spmKeys, 0), ('MFR', mfrKeys, 0),
                             ('MSR', msrKeys, 0), ('ECL', eclKeys, 0),
                             ('MINMAX', minmaxKeys, 1)]:
      st = tree.AppendItem(mech, mechLabels[tag], ct_type = cttype, data = tag)
      self.brMech[tag] = st
      for k in tbl:
        ck = self.mechCfgKeys[k]
        if ck in self.cfgValues.keys():
          v = self.cfgValues[ck]
        else:
          v = ""
        tc = wx.TextCtrl(tree, wx.ID_ANY, v, style = wx.TE_RIGHT)
        if k in helpText.keys():
          tc.SetToolTipString(helpText[k])
        tc.Bind(wx.EVT_CHAR, self.onTextCtrl)
        self.tcMech[k] = tc
        self.brMech[k] = tree.AppendItem(st, mechLabels[k], ct_type = 0,
                                         wnd = tc, data = k)

    allAbsent = True
    for i in minmaxKeys:
      k = self.mechCfgKeys[i]
      if k in self.cfgValues.keys():
        allAbsent = False
        break

    if allAbsent:
      self.tree.CheckItem(self.brMech['MINMAX'], False)
    else:
      self.tree.CheckItem(self.brMech['MINMAX'], True)

    br = tree.AppendItem(mech, mechLabels['ABSE'], ct_type = 1, data = "ABSE")
    ck = self.mechCfgKeys['ABSE']
    if ck in self.cfgValues.keys() and self.cfgValues[ck]:
      self.tree.CheckItem(br, True)
    else:
      self.tree.CheckItem(br, False)
    self.brMech['ABSE'] = br
    return mech

  def insertMechValues(self):
    for k in self.mechCfgKeys.keys():
      if k in self.tcMech.keys():
        ck = self.mechCfgKeys[k]
        if ck in self.cfgValues.keys():
          self.tcMech[k].SetValue(self.cfgValues[ck])

    allAbsent = True
    for i in ['MINX', 'MAXX', 'MINY', 'MAXY', 'MINZ', 'MAXZ']:
      k = self.mechCfgKeys[i]
      if k in self.cfgValues.keys():
        allAbsent = False
        break

    if allAbsent:
      self.tree.CheckItem(self.brMech['MINMAX'], False)
    else:
      self.tree.CheckItem(self.brMech['MINMAX'], True)

    br = self.brMech['ABSE']
    ck = self.mechCfgKeys['ABSE']
    if ck in self.cfgValues.keys() and self.cfgValues[ck]:
      self.tree.CheckItem(br, True)
    else:
      self.tree.CheckItem(br, False)

  def accSubTree(self, tree, root):
    accLabels = {'ACTYPE': "Acceleration Type:",
                 'ACRR': "RepRap", 'ACRP': "Ramping", 'ACTP': "Temporal",
                 'ACCEL' : "Acceleration", 'LKAH': "Look Ahead",
                 'JERK': "Maximum Jerk:",
                 'JERKX': "X", 'JERKY': "Y", 'JERKZ': "Z", 'JERKE': "E"}

    self.accCfgKeys = {'ACRR': "ACCELERATION_REPRAP",
                       'ACRP': "ACCELERATION_RAMPING",
                       'ACTP': "ACCELERATION_TEMPORAL",
                       'ACCEL': "ACCELERATION", 'LKAH': "LOOKAHEAD",
                       'JERKX': "MAX_JERK_X", 'JERKY': "MAX_JERK_Y",
                       'JERKZ': "MAX_JERK_Z", 'JERKE': "MAX_JERK_E"}

    self.tcAcc = {}
    self.brAcc = {}

    acc = tree.AppendItem(root, "2. Acceleration", data = "Acceleration")

    st = tree.AppendItem(acc, accLabels['ACTYPE'], data = "ACTYPE")
    self.brAcc['ACTYPE'] = st

    for tag in ['ACRR', 'ACRP', 'ACTP']:
      br = tree.AppendItem(st, accLabels[tag], ct_type = 2, data = tag)
      self.brAcc[tag] = br
      ck = self.accCfgKeys[tag]
      if ck in self.cfgValues.keys() and self.cfgValues[ck]:
        self.tree.CheckItem(br, True)

    tag = 'ACRP'
    ck = self.accCfgKeys['ACCEL']
    if ck in self.cfgValues.keys():
      v = self.cfgValues[ck]
    else:
      v = ""
    tc = wx.TextCtrl(tree, wx.ID_ANY, v, style = wx.TE_RIGHT)
    tc.SetToolTipString(helpText['ACCEL'])
    tc.Bind(wx.EVT_CHAR, self.onTextCtrl)
    self.tcAcc['ACCEL'] = tc

    br = self.brAcc[tag]
    self.brAcc['ACCEL'] = tree.AppendItem(br, accLabels['ACCEL'], ct_type = 0,
                                          wnd = self.tcAcc['ACCEL'],
                                          data = 'ACCEL')
    self.brAcc['LKAH'] = tree.AppendItem(br, accLabels['LKAH'], ct_type = 1,
                                         data = 'LKAH')

    br = self.brAcc['LKAH']
    ck = self.accCfgKeys['LKAH']
    if ck in self.cfgValues.keys() and self.cfgValues[ck]:
      self.tree.CheckItem(br, True)
    else:
      self.tree.CheckItem(br, False)

    st = tree.AppendItem(acc, accLabels['JERK'], data = 'JERK')
    self.brAcc['JERK'] = st

    for k in ['JERKX', 'JERKY', 'JERKZ', 'JERKE']:
      ck = self.accCfgKeys[k]
      if ck in self.cfgValues.keys():
        v = self.cfgValues[ck]
      else:
        v = ""
      tc = wx.TextCtrl(tree, wx.ID_ANY, v, style = wx.TE_RIGHT)
      if k in helpText.keys():
        tc.SetToolTipString(helpText[k])
      tc.Bind(wx.EVT_CHAR, self.onTextCtrl)
      self.tcAcc[k] = tc
      self.brAcc[k] = tree.AppendItem(st, accLabels[k], ct_type = 0,
                                      wnd = tc, data = k)

    return acc

  def insertAccValues(self):
    for k in self.accCfgKeys.keys():
      if k in self.tcAcc.keys():
        ck = self.accCfgKeys[k]
        if ck in self.cfgValues.keys():
          self.tcAcc[k].SetValue(self.cfgValues[ck])

    for tag in ['ACRR', 'ACRP', 'ACTP']:
      br = self.brAcc[tag]
      ck = self.accCfgKeys[tag]
      if ck in self.cfgValues.keys() and self.cfgValues[ck]:
        self.tree.CheckItem(br, True)

    br = self.brAcc['LKAH']
    ck = self.accCfgKeys['LKAH']
    if ck in self.cfgValues.keys() and self.cfgValues[ck]:
      self.tree.CheckItem(br, True)
    else:
      self.tree.CheckItem(br, False)

  def onItemChecked(self, evt):
    item = evt.GetItem()
    match = None
    for k in self.brMech.keys():
      if item == self.brMech[k]:
        match = k
        break

    if match is None:
      return

    if match == 'MINMAX':
      self.tree.EnableChildren(item, self.tree.IsItemChecked(item))

  def onToolTip(self, evt):
    item = evt.GetItem()
    if item:
      data = self.tree.GetPyData(item)
      if data and data in helpText.keys():
        evt.SetToolTip(wx.ToolTip(helpText[data]))

  def onClose(self, evt):
    self.Destroy()

  def onTextCtrl(self, event):
    char = chr(event.GetKeyCode())
    event.Skip()

  def onLoadConfig(self, evt):
    wildcard = "C Header files (*.h)|*.h"

    dlg = wx.FileDialog(self, message = "Choose a Config file",
                        defaultDir = ".", defaultFile = "",
                        wildcard = wildcard, style = wx.OPEN | wx.CHANGE_DIR)

    path = None
    if dlg.ShowModal() == wx.ID_OK:
      path = dlg.GetPath()

    dlg.Destroy()
    if path is None:
      return

    self.loadConfigFile(path)
    self.insertMechValues()
    self.insertAccValues()

  def loadConfigFile(self, fn):
    try:
      lst = list(open(fn))
    except:
      return False

    self.cfgValues = {}
    self.cfgValues['E_ABSOLUTE'] = False
    for ln in lst:
      if ln.lstrip().startswith("#define"):
        m = self.reDefTS.search(ln)
        if m:
          t = m.groups()
          if len(t) == 1:
            print "TSkey (%s)" % t[0]
            continue

        m = self.reDefHT.search(ln)
        if m:
          t = m.groups()
          if len(t) == 1:
            print "HTkey (%s)" % t[0]
            continue

        m = self.reDefQS.search(ln)
        if m:
          t = m.groups()
          if len(t) == 2:
            self.cfgValues[t[0]] = t[1]
            continue

        m = self.reDefine.search(ln)
        if m:
          t = m.groups()
          if len(t) == 2:
            self.cfgValues[t[0]] = t[1]
            continue

        m = self.reDefBool.search(ln)
        if m:
          t = m.groups()
          if len(t) == 1:
            self.cfgValues[t[0]] = True

    return True


if __name__ == '__main__':
  app = wx.PySimpleApp()
  frame = MyFrame()
  frame.Show(True)
  app.MainLoop()
