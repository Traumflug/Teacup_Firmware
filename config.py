#!/bin/env python

import os
import wx
import re
import time

VERSION = "0.1"

try:
  from agw import customtreectrl as CT
except ImportError:
  import wx.lib.agw.customtreectrl as CT

from config_helptext import helpText

supportedCPUs = ['ATmega168', 'ATmega328P', 'ATmega644P', 'ATmega644PA',
                 'ATmega1280', 'ATmega1284P', 'ATmega2560', 'AT90USB1286']

BSIZE = (90, 60)
BSIZESMALL = (90, 30)

reDefine = re.compile("\s*#define\s+(\S+)\s+(\S+)")
reDefQS = re.compile("\s*#define\s+(\S+)\s+(\"[^\"]*\")")
reDefTS = re.compile("\s*(DEFINE_TEMP_SENSOR\\([^)]*\\))")
reDefHT = re.compile("\s*(DEFINE_HEATER\\([^)]*\\))")
reDefBool = re.compile("\s*#define\s+(\S+)\s+")
reIncArduino = re.compile("\s*#include\s+\"arduino.h\"")

reSensor3 = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")
reSensor4 = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")
reHeater = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")

reInteger = re.compile("^\d+$")
reFloat = re.compile("^\d+(\.\d+)?$")
rePin = re.compile("^[AD]IO\d+$")

reAVR = re.compile("__AVR_(\w+)__")

defineValueFormat =      "#define %-30.30s %s\n"
defineBoolFormat =       "#define %s\n"
defineHeaterFormat =     "#define HEATER_%s HEATER_%s\n"
defineULFormat =         "#define %-30.30s %sUL\n"
defineDCExtruderFormat = "#define %-30.30s HEATER_%s\n"


class Page:
  def __init__(self):
    self.modified = False
    self.valid = True
    self.fieldValid = {}
    self.textControls = {}
    self.checkBoxes = {}
    self.radioButtons = {}
    self.choices = {}

  def addTextCtrl(self, name, labelWidth, ht, validator):
    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, self.labels[name],
                       size = (labelWidth, -1), style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    tc = wx.TextCtrl(self, wx.ID_ANY, "", style = wx.TE_RIGHT, name = name)
    self.fieldValid[name] = True
    tc.Bind(wx.EVT_TEXT, validator)
    self.textControls[name] = tc
    lsz.Add(tc)
    if ht:
      hts = ht
    else:
      if name in helpText.keys():
        hts = helpText[name]
      else:
        hts = ""
    tc.SetToolTipString(hts)

    return lsz

  def addCheckBox(self, name, ht, validator):
    if name in self.labels.keys():
      lbl = self.labels[name]
    else:
      lbl = name
    cb = wx.CheckBox(self, wx.ID_ANY, lbl)
    cb.Bind(wx.EVT_CHECKBOX, validator)
    self.checkBoxes[name] = cb
    if ht:
      hts = ht
    else:
      if name in helpText.keys():
        hts = helpText[name]
      else:
        hts = ""
    cb.SetToolTipString(hts)

    return cb

  def addRadioButton(self, name, style, ht, validator):
    rb = wx.RadioButton(self, wx.ID_ANY, self.labels[name], style = style)
    self.Bind(wx.EVT_RADIOBUTTON, validator, rb)
    self.radioButtons[name] = rb
    if ht:
      hts = ht
    else:
      if name in helpText.keys():
        hts = helpText[name]
      else:
        hts = ""
    rb.SetToolTipString(hts)

    return rb

  def addChoice(self, name, choices, selection, labelWidth, ht, validator):
    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, self.labels[name],
                       size = (labelWidth, -1), style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    ch = wx.Choice(self, wx.ID_ANY, choices = choices, name = name)
    ch.Bind(wx.EVT_CHOICE, validator)
    ch.SetSelection(selection)
    lsz.Add(ch)
    self.choices[name] = ch
    if ht:
      hts = ht
    else:
      if name in helpText.keys():
        hts = helpText[name]
      else:
        hts = ""
    ch.SetToolTipString(hts)
    return lsz

  def setChoice(self, name, cfgValues, choices, default):
    if name in cfgValues.keys():
      bv = cfgValues[name]
    else:
      bv = self.defaultClock

    try:
      s = choices.index(bv)
    except:
      try:
        s = choices.index(default)
      except:
        s = 0

    self.choices[name].SetSelection(s)

  def onTextCtrlInteger(self, evt):
    self.assertModified(True)
    tc = evt.GetEventObject()
    name = tc.GetName()
    w = tc.GetValue().strip()
    if w == "":
      valid = True
    else:
      m = reInteger.match(w)
      if m:
        valid = True
      else:
        valid = False

    self.setFieldValidity(name, valid)

    if valid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()
    evt.Skip()

  def onTextCtrlFloat(self, evt):
    self.assertModified(True)
    tc = evt.GetEventObject()
    name = tc.GetName()
    w = tc.GetValue().strip()
    if w == "":
      valid = True
    else:
      m = reFloat.match(w)
      if m:
        valid = True
      else:
        valid = False

    self.setFieldValidity(name, valid)

    if valid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()
    evt.Skip()

  def onTextCtrlPin(self, evt):
    self.assertModified(True)
    tc = evt.GetEventObject()
    self.validatePin(tc)
    evt.Skip()

  def onTextCtrl(self, evt):
    self.assertModified(True)
    evt.Skip()

  def onChoice(self, evt):
    self.assertModified(True)
    evt.Skip()

  def validatePin(self, tc):
    name = tc.GetName()
    w = tc.GetValue().strip()

    if w == "":
      valid = True
    else:
      m = reInteger.match(w)
      if m:
        valid = True
      else:
        if self.includeArduino:
          m = rePin.match(w)
          if m:
            valid = True
          else:
            valid = False
        else:
          valid = False

    self.setFieldValidity(name, valid)

    if valid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()

  def onCheckBox(self, evt):
    self.assertModified(True)
    evt.Skip()

  def assertModified(self, flag):
    if flag != self.modified:
      self.parent.assertModified(self.id, flag)
      self.modified = flag

  def setFieldValidity(self, name, flag):
    self.fieldValid[name] = flag

    pgValid = True
    for k in self.fieldValid.keys():
      if not self.fieldValid[k]:
        pgValid = False
        break

    self.assertValid(pgValid)

  def assertValid(self, flag):
    if flag != self.valid:
      self.parent.assertValid(self.id, flag)
      self.valid = flag


class MechanicalPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.id = idPg
    self.parent = parent

    self.spmKeys = ['STEPS_PER_M_X', 'STEPS_PER_M_Y', 'STEPS_PER_M_Z',
                    'STEPS_PER_M_E']

    self.mfrKeys = ['MAXIMUM_FEEDRATE_X', 'MAXIMUM_FEEDRATE_Y',
                    'MAXIMUM_FEEDRATE_Z', 'MAXIMUM_FEEDRATE_E']

    self.msrKeys = ['SEARCH_FEEDRATE_X', 'SEARCH_FEEDRATE_Y',
                    'SEARCH_FEEDRATE_Z']

    self.eclKeys = ['ENDSTOP_CLEARANCE_X', 'ENDSTOP_CLEARANCE_Y',
                    'ENDSTOP_CLEARANCE_Z']

    self.minmaxKeys = ['X_MIN', 'X_MAX', 'Y_MIN', 'Y_MAX', 'Z_MIN', 'Z_MAX']

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
                   'E_ABSOLUTE': "Absolute E Coordinates"}

    labelWidth = 40;

    sz = wx.GridBagSizer()
    sz.AddSpacer((10, 10), pos = (0, 0))

    b = wx.StaticBox(self, wx.ID_ANY, "Steps Per Meter")
    kh = 'STEPS_PER_M'
    ht = None
    if kh in helpText.keys():
      ht = helpText[kh]
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.spmKeys:
      tc = self.addTextCtrl(k, labelWidth, ht, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "Maximum Feedrate")
    kh = 'MAXIMUM_FEEDRATE'
    ht = None
    if kh in helpText.keys():
      ht = helpText[kh]
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.mfrKeys:
      tc = self.addTextCtrl(k, labelWidth, ht, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 5))

    b = wx.StaticBox(self, wx.ID_ANY, "Search Feedrate")
    kh = 'SEARCH_FEEDRATE'
    ht = None
    if kh in helpText.keys():
      ht = helpText[kh]
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.msrKeys:
      tc = self.addTextCtrl(k, labelWidth, ht, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 7))

    b = wx.StaticBox(self, wx.ID_ANY, "Endstop Clearance")
    kh = 'ENDSTOP_CLEARANCE'
    ht = None
    if kh in helpText.keys():
      ht = helpText[kh]
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.eclKeys:
      tc = self.addTextCtrl(k, labelWidth, ht, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (3, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "Travel Limits")
    kh = 'MINMAX'
    ht = None
    if kh in helpText.keys():
      ht = helpText[kh]
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.minmaxKeys:
      tc = self.addTextCtrl(k, labelWidth, ht, self.onTextCtrlInteger)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (3, 3))

    cb = self.addCheckBox('E_ABSOLUTE', None, self.onCheckBox)

    sz.Add(cb, pos = (3, 7), flag = wx.ALIGN_CENTER_VERTICAL)

    bsz = wx.BoxSizer(wx.VERTICAL)
    b = wx.Button(self, wx.ID_ANY, "Calculate\nBelt Driven", size = BSIZE)
    b.SetToolTipString("Open the calculator for Axes that are belt-driven")

    bsz.Add(b, 1, wx.ALL, 5)
    b = wx.Button(self, wx.ID_ANY, "Calculate\nScrew Driven", size = BSIZE)
    bsz.Add(b, 1, wx.ALL, 5)
    b.SetToolTipString("Open the calculator for Axes that are screw-driven")

    sz.Add(bsz, pos = (1, 3))
    self.SetSizer(sz)

  def insertValues(self, cfgValues):
    self.assertValid(True)
    for k in self.fieldValid.keys():
      self.fieldValid[k] = True
    for k in self.textControls.keys():
      if k in cfgValues.keys():
        self.textControls[k].SetValue(cfgValues[k])
      else:
        self.textControls[k].SetValue("")

    for k in self.checkBoxes.keys():
      if k in cfgValues.keys() and cfgValues[k]:
        self.checkBoxes[k].SetValue(True)
      else:
        self.checkBoxes[k].SetValue(False)
    self.assertModified(False)

  def saveValues(self, fp):
    self.assertModified(False)
    fp.write(
"\n/***************************************************************************\\\n\
*                                                                           *\n\
* 1. MECHANICAL/HARDWARE                                                    *\n\
*                                                                           *\n\
\\***************************************************************************/\n")

    for k in self.spmKeys + self.mfrKeys + self.msrKeys + self.eclKeys + \
             self.minmaxKeys:
      v = self.textControls[k].GetValue()
      if v != "":
        fp.write(defineValueFormat % (k, v))

    k = 'E_ABSOLUTE'
    cb = self.checkBoxes[k]
    if cb.IsChecked():
      fp.write(defineBoolFormat % k)


class AccelerationPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.parent = parent
    self.id = idPg

    self.accTypeKeys = ['ACCELERATION_REPRAP', 'ACCELERATION_RAMPING',
                        'ACCELERATION_TEMPORAL']
    self.jerkKeys = ['MAX_JERK_X', 'MAX_JERK_Y', 'MAX_JERK_Z', 'MAX_JERK_E']

    self.labels = {'ACCELERATION_REPRAP': "RepRap",
                   'ACCELERATION_RAMPING': "Ramping",
                   'ACCELERATION_TEMPORAL': "Temporal",
                   'ACCELERATION' : "Acceleration:", 'LOOKAHEAD': "Look Ahead",
                   'MAX_JERK_X': "X:", 'MAX_JERK_Y': "Y:",
                   'MAX_JERK_Z': "Z:", 'MAX_JERK_E': "E:"}

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))
    b = wx.StaticBox(self, wx.ID_ANY, "Acceleration Type:")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    style = wx.RB_GROUP
    for k in self.accTypeKeys:
      rb = self.addRadioButton(k, style, None, self.onAccTypeSelect)
      style = 0

      sbox.Add(rb, 1, wx.LEFT + wx.RIGHT, 16)
      sbox.AddSpacer((5, 5))

    rb = wx.RadioButton(self, wx.ID_ANY, "None", style = style)
    rb.SetValue(True)
    self.Bind(wx.EVT_RADIOBUTTON, self.onAccTypeSelect, rb)
    sbox.Add(rb, 1, wx.LEFT + wx.RIGHT, 16)
    sbox.AddSpacer((5, 5))
    sz.Add(sbox, pos = (1, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "Ramping Parameters:")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))

    k = 'ACCELERATION'
    tc = self.addTextCtrl(k, 80, None, self.onTextCtrlInteger)
    self.textControls[k].Enable(False)

    sbox.Add(tc)
    sbox.AddSpacer((5, 5))

    k = 'LOOKAHEAD'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    self.checkBoxes[k].Enable(False)

    sbox.Add(cb, 1, wx.ALIGN_CENTER_HORIZONTAL)
    sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 3))

    b = wx.StaticBox(self, wx.ID_ANY, "Maximum Jerk")
    kh = 'MAX_JERK'
    ht = None
    if kh in helpText.keys():
      ht = helpText[kh]
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.jerkKeys:
      tc = self.addTextCtrl(k, 40, ht, self.onTextCtrlInteger)

      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.AddSpacer((80, 20), pos = (1, 4))
    sz.Add(sbox, pos = (1, 5))

    self.SetSizer(sz)

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
    self.assertValid(True)
    for k in self.fieldValid.keys():
      self.fieldValid[k] = True
    self.checkBoxes['LOOKAHEAD'].Enable(False)
    self.textControls['ACCELERATION'].Enable(False)
    for k in self.textControls.keys():
      if k in cfgValues.keys():
        self.textControls[k].SetValue(cfgValues[k])
      else:
        self.textControls[k].SetValue("")

    for tag in ['ACCELERATION_REPRAP', 'ACCELERATION_RAMPING',
                'ACCELERATION_TEMPORAL']:
      if tag in cfgValues.keys() and cfgValues[tag]:
        self.radioButtons[tag].SetValue(True)
        if tag == 'ACCELERATION_RAMPING':
          self.checkBoxes['LOOKAHEAD'].Enable(True)
          self.textControls['ACCELERATION'].Enable(True)

    k = 'LOOKAHEAD'
    if k in cfgValues.keys() and cfgValues[k]:
      self.checkBoxes[k].SetValue(True)
    else:
      self.checkBoxes[k].SetValue(False)
    self.assertModified(False)

  def saveValues(self, fp):
    self.assertModified(False)
    fp.write(
"\n/***************************************************************************\\\n\
*                                                                           *\n\
* 2. ACCELERATION                                                           *\n\
*                                                                           *\n\
\\***************************************************************************/\n")

    for tag in ['ACCELERATION_REPRAP', 'ACCELERATION_RAMPING',
                'ACCELERATION_TEMPORAL']:
      rb = self.radioButtons[tag]
      if rb.GetValue():
        fp.write(defineBoolFormat % tag)

    k = 'ACCELERATION'
    v = self.textControls[k].GetValue()
    if v != "":
      fp.write(defineValueFormat % (k, v))

    k = 'LOOKAHEAD'
    cb = self.checkBoxes[k]
    if cb.IsChecked():
      fp.write(defineBoolFormat % k)

    for k in ['MAX_JERK_X', 'MAX_JERK_Y', 'MAX_JERK_Z', 'MAX_JERK_E']:
      v = self.textControls[k].GetValue()
      if v != "":
        fp.write(defineValueFormat % (k, v))


class PinoutsPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.parent = parent
    self.id = idPg

    self.pinXKeys = [('X_STEP_PIN', 0), ('X_DIR_PIN', 0), ('X_MIN_PIN', 0),
                     ('X_MAX_PIN', 0), ('X_ENABLE_PIN', 0),
                     ('X_INVERT_DIR', 1), ('X_INVERT_MIN', 1),
                     ('X_INVERT_MAX', 1), ('X_INVERT_ENABLE', 1)]
    self.pinYKeys = [('Y_STEP_PIN', 0), ('Y_DIR_PIN', 0), ('Y_MIN_PIN', 0),
                     ('Y_MAX_PIN', 0), ('Y_ENABLE_PIN', 0),
                     ('Y_INVERT_DIR', 1), ('Y_INVERT_MIN', 1),
                     ('Y_INVERT_MAX', 1), ('Y_INVERT_ENABLE', 1)]
    self.pinZKeys = [('Z_STEP_PIN', 0), ('Z_DIR_PIN', 0), ('Z_MIN_PIN', 0),
                     ('Z_MAX_PIN', 0), ('Z_ENABLE_PIN', 0),
                     ('Z_INVERT_DIR', 1), ('Z_INVERT_MIN', 1),
                     ('Z_INVERT_MAX', 1), ('Z_INVERT_ENABLE', 1)]
    self.pinEKeys = [('E_STEP_PIN', 0), ('E_DIR_PIN', 0), ('E_ENABLE_PIN', 0),
                     ('E_INVERT_DIR', 1), ('E_INVERT_ENABLE', 1)]
    self.pinMiscKeys = [('USE_INTERNAL_PULLUPS', 1), ('TX_ENABLE_PIN', 0),
                        ('RX_ENABLE_PIN', 0), ('PS_ON_PIN', 0),
                        ('PS_MOSFET_PIN', 0), ('STEPPER_ENABLE_PIN', 0),
                        ('STEPPER_INVERT_ENABLE', 1), ('SD_CARD_DETECT', 0),
                        ('SD_WRITE_PROTECT', 0), ('DEBUG_LED_PIN', 0)]

    self.labels = {'INCLUDE_ARDUINO': "Include arduino.h header file",
                   'USE_INTERNAL_PULLUPS': "Use Internal Pullups",

                   'TX_ENABLE_PIN': "Tx Enable Pin:",
                   'RX_ENABLE_PIN': "Rx Enable Pin:",

                   'X_STEP_PIN': "Step Pin:", 'X_DIR_PIN': "Direction Pin:",
                   'X_MIN_PIN': "Minimum Pin:", 'X_MAX_PIN': "Maximum Pin:",
                   'X_ENABLE_PIN': "Enable Pin:",
                   'X_INVERT_DIR': "Invert Direction",
                   'X_INVERT_MIN': "Invert Minimum",
                   'X_INVERT_MAX': "Invert Maximum",
                   'X_INVERT_ENABLE': "Invert Enable",

                   'Y_STEP_PIN': "Step Pin:", 'Y_DIR_PIN': "Direction Pin:",
                   'Y_MIN_PIN': "Minimum Pin:", 'Y_MAX_PIN': "Maximum Pin:",
                   'Y_ENABLE_PIN': "Enable Pin:",
                   'Y_INVERT_DIR': "Invert Direction",
                   'Y_INVERT_MIN': "Invert Minimum",
                   'Y_INVERT_MAX': "Invert Maximum",
                   'Y_INVERT_ENABLE': "Invert Enable",

                   'Z_STEP_PIN': "Step Pin:", 'Z_DIR_PIN': "Direction Pin:",
                   'Z_MIN_PIN': "Minimum Pin:", 'Z_MAX_PIN': "Maximum Pin:",
                   'Z_ENABLE_PIN': "Enable Pin:",
                   'Z_INVERT_DIR': "Invert Direction",
                   'Z_INVERT_MIN': "Invert Minimum",
                   'Z_INVERT_MAX': "Invert Maximum",
                   'Z_INVERT_ENABLE': "Invert Enable",

                   'E_STEP_PIN': "Step Pin:", 'E_DIR_PIN': "Direction Pin:",
                   'E_ENABLE_PIN': "Enable Pin:",
                   'E_INVERT_DIR': "Invert Direction",
                   'E_INVERT_ENABLE': "Invert Enable",

                   'PS_ON_PIN': "PSU On Pin:",
                   'PS_MOSFET_PIN': "PSU MOSFET Pin:",

                   'STEPPER_ENABLE_PIN': "Stepper Enable Pin:",
                   'STEPPER_INVERT_ENABLE': "Stepper Invert Enable",

                   'SD_CARD_DETECT': "SD Detect Pin:",
                   'SD_WRITE_PROTECT': "Write Protect Pin:",
                   'DEBUG_LED_PIN': "Debug LED Pin:"}

    labelWidth = 80
    labelWidthM = 100
    self.includeArduino = False

    sz = wx.GridBagSizer()
    sz.AddSpacer((10, 10), pos = (0, 0))

    b = wx.StaticBox(self, wx.ID_ANY, "X Axis")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k, ctype in self.pinXKeys:
      if ctype == 0:
        tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlPin)
        sbox.Add(tc)
      else:
        cb = self.addCheckBox(k, None, self.onCheckBox)
        sbox.Add(cb, 1, wx.LEFT, 30)

      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "Y Axis")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k, ctype in self.pinYKeys:
      if ctype == 0:
        tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlPin)
        sbox.Add(tc)
      else:
        cb = self.addCheckBox(k, None, self.onCheckBox)
        sbox.Add(cb, 1, wx.LEFT, 30)

      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 3))

    b = wx.StaticBox(self, wx.ID_ANY, "Z Axis")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k, ctype in self.pinZKeys:
      if ctype == 0:
        tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlPin)
        sbox.Add(tc)
      else:
        cb = self.addCheckBox(k, None, self.onCheckBox)
        sbox.Add(cb, 1, wx.LEFT, 30)

      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 5))

    b = wx.StaticBox(self, wx.ID_ANY, "E Axis")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k, ctype in self.pinEKeys:
      if ctype == 0:
        tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlPin)
        sbox.Add(tc)
      else:
        cb = self.addCheckBox(k, None, self.onCheckBox)
        sbox.Add(cb, 1, wx.LEFT, 30)

      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 7))

    k = 'USE_INTERNAL_PULLUPS'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (3, 1), flag = wx.ALIGN_CENTER_HORIZONTAL)

    k = 'TX_ENABLE_PIN'
    tc = self.addTextCtrl(k, labelWidthM, None, self.onTextCtrlPin)
    sz.Add(tc, pos = (5, 1))

    k = 'RX_ENABLE_PIN'
    tc = self.addTextCtrl(k, labelWidthM, None, self.onTextCtrlPin)
    sz.Add(tc, pos = (6, 1))

    k = 'PS_ON_PIN'
    tc = self.addTextCtrl(k, labelWidthM, None, self.onTextCtrlPin)
    sz.Add(tc, pos = (3, 3))

    k = 'PS_MOSFET_PIN'
    tc = self.addTextCtrl(k, labelWidthM, None, self.onTextCtrlPin)
    sz.Add(tc, pos = (4, 3))

    k = 'STEPPER_ENABLE_PIN'
    tc = self.addTextCtrl(k, labelWidthM, None, self.onTextCtrlPin)
    sz.Add(tc, pos = (6, 3))

    k = 'STEPPER_INVERT_ENABLE'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (7, 3), flag = wx.ALIGN_CENTER_HORIZONTAL)

    k = 'SD_CARD_DETECT'
    tc = self.addTextCtrl(k, labelWidthM, None, self.onTextCtrlPin)
    sz.Add(tc, pos = (3, 5))

    k = 'SD_WRITE_PROTECT'
    tc = self.addTextCtrl(k, labelWidthM, None, self.onTextCtrlPin)
    sz.Add(tc, pos = (4, 5))

    k = 'DEBUG_LED_PIN'
    tc = self.addTextCtrl(k, labelWidthM, None, self.onTextCtrlPin)
    sz.Add(tc, pos = (3, 7))

    k = 'INCLUDE_ARDUINO'
    cb = self.addCheckBox(k, None, self.onCheckBoxArduino)
    sz.Add(cb, pos = (5, 7), flag = wx.ALIGN_CENTER_HORIZONTAL)

    self.SetSizer(sz)

  def revalidatePins(self):
    for k in self.textControls.keys():
      self.validatePin(self.textControls[k])
    self.parent.revalidatePins(self.includeArduino)

  def onCheckBoxArduino(self, evt):
    self.assertModified(True)
    cb = evt.GetEventObject()
    self.includeArduino = cb.IsChecked()
    self.revalidatePins()
    evt.Skip()

  def setIncludeArduino(self, flag):
    self.includeArduino = flag

  def insertValues(self, cfgValues):
    self.assertValid(True)
    for k in self.fieldValid.keys():
      self.fieldValid[k] = True
    for k in self.textControls.keys():
      if k in cfgValues.keys():
        self.textControls[k].SetValue(cfgValues[k])
      else:
        self.textControls[k].SetValue("")

    for k in self.checkBoxes.keys():
      if k in cfgValues.keys() and cfgValues[k]:
        self.checkBoxes[k].SetValue(True)
      elif (k == 'INCLUDE_ARDUINO') and self.includeArduino:
        self.checkBoxes[k].SetValue(True)
      else:
        self.checkBoxes[k].SetValue(False)
    self.revalidatePins()
    self.assertModified(False)

  def saveValues(self, fp):
    self.assertModified(False)
    fp.write(
"\n/***************************************************************************\\\n\
*                                                                           *\n\
* 3. PINOUTS                                                                *\n\
*                                                                           *\n\
\\***************************************************************************/\n")

    if self.includeArduino:
      fp.write("\n#include \"arduino.h\"\n\n")

    for k, cttype in self.pinXKeys + self.pinYKeys + self.pinZKeys + \
                     self.pinEKeys + self.pinMiscKeys:
      if cttype == 1:
        if self.checkBoxes[k].IsChecked():
          fp.write(defineBoolFormat % k)

      else:
        v = self.textControls[k].GetValue()
        if v != "":
          fp.write(defineValueFormat % (k, v))


class SensorsPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.parent = parent
    self.id = idPg

    self.includeArduino = False

    self.sensorTypeKeys = ['TEMP_MAX6675', 'TEMP_THERMISTOR', 'TEMP_AD595',
                           'TEMP_PT100', 'TEMP_INTERCOM']
    self.sensorType = {'TEMP_MAX6675': 'TT_MAX6675',
                       'TEMP_THERMISTOR': 'TT_THERMISTOR',
                       'TEMP_AD595': 'TT_AD595',
                       'TEMP_PT100': 'TT_PT100',
                       'TEMP_INTERCOM': 'TT_INTERCOM'}
    self.labels = {'TEMP_HYSTERESIS': "Temperature hysteresis:",
                   'TEMP_RESIDENCY_TIME': "Temperature residency time:",
                   'TEMP_EWMA': "Temperature EMWA:",
                   'TEMP_MAX6675': "MAX6675",
                   'TEMP_THERMISTOR': "Thermistor",
                   'TEMP_AD595': 'AD595',
                   'TEMP_PT100': "PT100", 'TEMP_INTERCOM': "Intercom"}

    labelWidth = 160

    sz = wx.GridBagSizer()
    sz.AddSpacer((30, 30), pos = (0, 0))

    self.sensors = []

    b = wx.StaticBox(self, wx.ID_ANY, "Sensor Types")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    ht = None
    if 'TEMP_TYPES' in helpText.keys():
      ht = helpText['TEMP_TYPES']
    for k in self.sensorTypeKeys:
      cb = self.addCheckBox(k, ht, self.onCheckBoxSensorType)
      sbox.Add(cb, 1, wx.LEFT + wx.RIGHT, 10)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 1), span = (5, 1))

    k = 'TEMP_HYSTERESIS'
    tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlInteger)
    sz.Add(tc, pos = (1, 3))

    k = 'TEMP_RESIDENCY_TIME'
    tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlInteger)
    sz.Add(tc, pos = (3, 3))

    k = 'TEMP_EWMA'
    tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlEWMA)
    sz.Add(tc, pos = (5, 3))

    self.lb = SensorList(self)
    sz.Add(self.lb, pos = (7, 1), span = (1, 3))

    bsz = wx.BoxSizer(wx.VERTICAL)
    self.bAdd = wx.Button(self, wx.ID_ANY, "Add", size = BSIZESMALL)
    self.Bind(wx.EVT_BUTTON, self.doAdd, self.bAdd)
    self.bAdd.Enable(False)
    if 'ADDSENSOR' in helpText.keys():
      self.bAdd.SetToolTipString(helpText['ADDSENSOR'])

    bsz.Add(self.bAdd)

    bsz.AddSpacer((10, 10))
    self.bDelete = wx.Button(self, wx.ID_ANY, "Delete", size = BSIZESMALL)
    self.bDelete.Enable(False)
    self.Bind(wx.EVT_BUTTON, self.doDelete, self.bDelete)
    bsz.Add(self.bDelete)
    if 'DELSENSOR' in helpText.keys():
      self.bDelete.SetToolTipString(helpText['DELSENSOR'])

    sz.Add(bsz, pos = (7, 4))

    self.SetSizer(sz)

  def onCheckBoxSensorType(self, evt):
    self.assertModified(True)

    ct = 0
    for tt in self.sensorTypeKeys:
      if self.checkBoxes[tt].IsChecked():
        ct += 1

    if ct == 0:
      self.bAdd.Enable(False)
    else:
      self.bAdd.Enable(True)

    evt.Skip()

  def onTextCtrlEWMA(self, evt):
    self.assertModified(True)
    tc = evt.GetEventObject()
    name = tc.GetName()
    w = tc.GetValue().strip()
    if w == "":
      valid = True
    else:
      m = reFloat.match(w)
      if m:
        v = float(w)
        if v < 0.1 or v > 1.0:
          valid = False
        else:
          valid = True
      else:
        valid = False

    self.setFieldValidity(name, valid)

    if valid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()
    evt.Skip()

  def revalidatePins(self, flag):
    self.includeArduino = flag

    tableValid = True
    for i in range(len(self.sensors)):
      pn = self.sensors[i][2]
      if pn == "":
        valid = False
      else:
        m = reInteger.match(pn)
        if m:
          valid = True
        else:
          if self.includeArduino:
            m = rePin.match(pn)
            if m:
              valid = True
            else:
              valid = False
          else:
            valid = False
      self.lb.setItemValidity(i, valid)
      if not valid:
        tableValid = False

    self.setFieldValidity('SENSORS', tableValid)

  def setItemSelected(self, n):
    self.selection = n
    if n is None:
      self.bDelete.Enable(False)
    else:
      self.bDelete.Enable(True)

  def doAdd(self, evt):
    sl = []
    for tt in self.sensorTypeKeys:
      if self.checkBoxes[tt].IsChecked():
        sl.append(self.sensorType[tt])

    nm = []
    for s in self.sensors:
      nm.append(s[0])

    dlg = AddSensorDlg(self, nm, sl, self.includeArduino)
    rc = dlg.ShowModal()
    if rc == wx.ID_OK:
      tt = dlg.getValues()

    dlg.Destroy()

    if rc != wx.ID_OK:
      return

    self.sensors.append(tt)
    self.lb.updateList(self.sensors)
    self.limitSensorTypeControls()

  def doDelete(self, evt):
    if self.selection is None:
      return

    self.assertModified(True)

    del self.sensors[self.selection]
    self.lb.updateList(self.sensors)
    self.limitSensorTypeControls()

  def limitSensorTypeControls(self):
    using = {}
    for s in self.sensors:
      using[s[1]] = True

    for k in self.sensorTypeKeys:
      self.checkBoxes[k].Enable(True)

    for tt in using.keys():
      if not tt.startswith("TT_"):
        continue

      k = "TEMP_" + tt[3:]
      if k in self.sensorTypeKeys:
        self.checkBoxes[k].Enable(False)

  def insertValues(self, cfgValues):
    for k in self.textControls.keys():
      if k in cfgValues.keys():
        self.textControls[k].SetValue(cfgValues[k])
      else:
        self.textControls[k].SetValue("")

    for k in self.checkBoxes.keys():
      if k in cfgValues.keys() and cfgValues[k]:
        self.checkBoxes[k].SetValue(True)
      else:
        self.checkBoxes[k].SetValue(False)

    ct = 0
    for k in self.sensorTypeKeys:
      if self.checkBoxes[k].IsChecked(): ct += 1
    self.bAdd.Enable(ct != 0)

    self.assertModified(False)

  def setSensors(self, sensors):
    self.sensors = sensors
    self.lb.updateList(self.sensors)
    self.revalidatePins(self.includeArduino)
    self.limitSensorTypeControls()

  def saveValues(self, fp):
    self.assertModified(False)
    fp.write(
"\n/***************************************************************************\\\n\
*                                                                           *\n\
* 4. TEMPERATURE SENSORS                                                    *\n\
*                                                                           *\n\
\\***************************************************************************/\n")

    for k in self.checkBoxes.keys():
      if self.checkBoxes[k].IsChecked():
        fp.write(defineBoolFormat % k)

    for k in self.textControls.keys():
      v = self.textControls[k].GetValue()
      if v != "":
        fp.write(defineValueFormat % (k, v))

    for s in self.sensors:
      sstr = ",".join(s)
      fp.write("DEFINE_TEMP_SENSOR(" + sstr + ")\n")


class SensorList(wx.ListCtrl):
  def __init__(self, parent):
    self.parent = parent
    self.currentItem = None
    wx.ListCtrl.__init__(self, parent, wx.ID_ANY, size = (495 + 4, 100),
            style = wx.LC_REPORT | wx.LC_VIRTUAL | wx.LC_HRULES | wx.LC_VRULES)

    self.valid = []

    self.InsertColumn(0, "Name")
    self.InsertColumn(1, "Sensor Type")
    self.InsertColumn(2, "Pin")
    self.InsertColumn(3, "Additional")
    self.SetColumnWidth(0, 55)
    self.SetColumnWidth(1, 105)
    self.SetColumnWidth(2, 55)
    self.SetColumnWidth(3, 280)

    self.SetItemCount(0)

    self.attr2 = wx.ListItemAttr()
    self.attr2.SetBackgroundColour("light blue")
    self.attr3 = wx.ListItemAttr()
    self.attr3.SetBackgroundColour("pink")

    self.Bind(wx.EVT_LIST_ITEM_SELECTED, self.OnItemSelected)
    self.Bind(wx.EVT_LIST_ITEM_DESELECTED, self.OnItemDeselected)

  def updateList(self, sensorList):
    self.sensorList = sensorList
    self.valid = [True] * len(sensorList)
    self.currentItem = None
    self.parent.setItemSelected(None)
    i = self.GetFirstSelected()
    while i != -1:
      self.Select(i, False)
      i = self.GetFirstSelected()

    self.SetItemCount(len(sensorList))

  def setItemValidity(self, i, flag = False):
    if i < 0 or i >= len(self.sensorList):
      return

    self.valid[i] = flag
    self.Refresh()

  def OnItemSelected(self, event):
    self.currentItem = event.m_itemIndex
    self.parent.setItemSelected(self.currentItem)

  def OnItemDeselected(self, event):
    self.currentItem = None
    self.parent.setItemSelected(None)

  def getColumnText(self, index, col):
    item = self.GetItem(index, col)
    return item.GetText()

  def OnGetItemText(self, item, col):
    if item < 0 or item >= len(self.sensorList):
      return "Error - no sensors"

    s = self.sensorList[item]

    if col == 0:
      return s[0]
    elif col == 1:
      return s[1]
    elif col == 2:
      return s[2]
    elif len(s) == 3:
      return ""
    else:
      return s[3]

  def OnGetItemAttr(self, item):
    if not self.valid[item]:
      return self.attr3

    if item % 2 == 1:
      return self.attr2
    else:
      return None


class AddSensorDlg(wx.Dialog):
  def __init__(self, parent, names, sl, arduinoFlag):
    wx.Dialog.__init__(self, parent, wx.ID_ANY, "Add Temperature Sensor",
                       size = (400, 204))
    self.Bind(wx.EVT_CLOSE, self.onCancel)

    self.names = names
    self.arduinoFlag = arduinoFlag

    self.nameValid = False
    self.pinValid = False

    sz = wx.BoxSizer(wx.VERTICAL)

    csz = wx.GridBagSizer()
    csz.AddSpacer((10, 10), pos = (0, 0))


    b = wx.StaticBox(self, wx.ID_ANY, "Sensor Type:")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    style = wx.RB_GROUP
    self.rbs = {}
    for k in sl:
      rb = wx.RadioButton(self, wx.ID_ANY, k, style = style)
      self.rbs[k] = rb
      self.Bind(wx.EVT_RADIOBUTTON, self.onSensorType, rb)
      if k in helpText.keys():
        hts = helpText[k]
      else:
        hts = ""
      rb.SetToolTipString(hts)
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
    self.tcName.SetToolTipString("Enter a unique name for this sensor")

    vsz.Add(lsz)

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Pin:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    self.tcPin = wx.TextCtrl(self, wx.ID_ANY, "", style = wx.TE_RIGHT)
    self.tcPin.Bind(wx.EVT_TEXT, self.onPinEntry)
    self.tcPin.SetBackgroundColour("pink")
    lsz.Add(self.tcPin)
    self.tcPin.SetToolTipString("Enter a pin number/name for this sensor")

    vsz.Add(lsz)

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Additional:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    self.tcAddtl = wx.TextCtrl(self, wx.ID_ANY, "")
    self.tcAddtl.Bind(wx.EVT_TEXT, self.onAddtlEntry)
    self.selectSensorType(sl[0])
    lsz.Add(self.tcAddtl)
    self.tcAddtl.SetToolTipString("Enter additional information required by "
                                  "the sensor type")

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

  def onPinEntry(self, evt):
    tc = evt.GetEventObject()
    self.validatePin(tc)
    self.checkDlgValidity()
    evt.Skip()

  def validatePin(self, tc):
    w = tc.GetValue().strip()
    if w == "":
      self.pinValid = False
    else:
      m = reInteger.match(w)
      if m:
        self.pinValid = True
      else:
        if self.arduinoFlag:
          m = rePin.match(w)
          if m:
            self.pinValid = True
          else:
            self.pinValid = False
        else:
          self.pinValid = False

    if self.pinValid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()

  def checkDlgValidity(self):
    if self.nameValid and self.pinValid:
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

  def onSensorType(self, evt):
    rb = evt.GetEventObject()
    label = rb.GetLabel()

    self.selectSensorType(label)

    evt.Skip()

  def getValues(self):
    nm = self.tcName.GetValue()
    pin = self.tcPin.GetValue()
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


class HeatersPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.parent = parent
    self.id = idPg

    self.includeArduino = False

    self.labels = {'HEATER_SANITY_CHECK': "Heater Sanity Check"}

    sz = wx.GridBagSizer()
    sz.AddSpacer((30, 30), pos = (0, 0))

    self.heaters = []

    k = 'HEATER_SANITY_CHECK'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (3, 1), flag = wx.ALIGN_CENTER_HORIZONTAL)

    self.lb = HeaterList(self)
    sz.Add(self.lb, pos = (1, 1), span = (1, 3))

    bsz = wx.BoxSizer(wx.VERTICAL)
    self.bAdd = wx.Button(self, wx.ID_ANY, "Add", size = BSIZESMALL)
    self.Bind(wx.EVT_BUTTON, self.doAdd, self.bAdd)
    if 'ADDHEATER' in helpText.keys():
      self.bAdd.SetToolTipString(helpText['ADDHEATER'])

    bsz.Add(self.bAdd)

    bsz.AddSpacer((10, 10))
    self.bDelete = wx.Button(self, wx.ID_ANY, "Delete", size = BSIZESMALL)
    self.bDelete.Enable(False)
    self.Bind(wx.EVT_BUTTON, self.doDelete, self.bDelete)
    bsz.Add(self.bDelete)
    if 'DELHEATER' in helpText.keys():
      self.bDelete.SetToolTipString(helpText['DELHEATER'])

    sz.Add(bsz, pos = (1, 4))

    self.SetSizer(sz)

  def revalidatePins(self, flag):
    self.includeArduino = flag

    tableValid = True
    for i in range(len(self.heaters)):
      pn = self.heaters[i][1]
      if pn == "":
        valid = False
      else:
        m = reInteger.match(pn)
        if m:
          valid = True
        else:
          if self.includeArduino:
            m = rePin.match(pn)
            if m:
              valid = True
            else:
              valid = False
          else:
            valid = False
      self.lb.setItemValidity(i, valid)
      if not valid:
        tableValid = False

    self.setFieldValidity('HEATERS', tableValid)

  def setItemSelected(self, n):
    self.selection = n
    if n is None:
      self.bDelete.Enable(False)
    else:
      self.bDelete.Enable(True)

  def doAdd(self, evt):
    nm = []
    for s in self.heaters:
      nm.append(s[0])

    dlg = AddHeaterDlg(self, nm, self.includeArduino)
    rc = dlg.ShowModal()
    if rc == wx.ID_OK:
      ht = dlg.getValues()

    dlg.Destroy()

    if rc != wx.ID_OK:
      return

    self.heaters.append(ht)
    self.lb.updateList(self.heaters)
    self.revalidatePins(self.includeArduino)
    self.parent.setHeaters(self.heaters)

  def doDelete(self, evt):
    if self.selection is None:
      return

    self.assertModified(True)

    del self.heaters[self.selection]
    self.lb.updateList(self.heaters)
    self.revalidatePins(self.includeArduino)
    self.parent.setHeaters(self.heaters)

  def insertValues(self, cfgValues):
    for k in self.checkBoxes.keys():
      if k in cfgValues.keys() and cfgValues[k]:
        self.checkBoxes[k].SetValue(True)
      else:
        self.checkBoxes[k].SetValue(False)

    self.assertModified(False)

  def setHeaters(self, heaters):
    self.heaters = heaters
    self.lb.updateList(self.heaters)
    self.revalidatePins(self.includeArduino)

  def saveValues(self, fp):
    self.assertModified(False)
    fp.write(
"\n/***************************************************************************\\\n\
*                                                                           *\n\
* 5. HEATERS                                                                *\n\
*                                                                           *\n\
\\***************************************************************************/\n")

    for k in self.checkBoxes.keys():
      if self.checkBoxes[k].IsChecked():
        fp.write(defineBoolFormat % k)

    names = []
    for s in self.heaters:
      names.append(s[0])
      sstr = ",".join(s)
      fp.write("DEFINE_HEATER(" + sstr + ")\n")

    for n in names:
      if n != n.upper():
        fp.write(defineHeaterFormat % (n.upper(), n))


class HeaterList(wx.ListCtrl):
  def __init__(self, parent):
    self.parent = parent
    self.currentItem = None
    wx.ListCtrl.__init__(self, parent, wx.ID_ANY, size = (165 + 4, 100),
           style = wx.LC_REPORT | wx.LC_VIRTUAL | wx.LC_HRULES | wx.LC_VRULES)

    self.valid = []

    self.InsertColumn(0, "Name")
    self.InsertColumn(1, "Pin")
    self.InsertColumn(2, "PWM")
    self.SetColumnWidth(0, 55)
    self.SetColumnWidth(1, 55)
    self.SetColumnWidth(2, 55)

    self.SetItemCount(0)

    self.attr2 = wx.ListItemAttr()
    self.attr2.SetBackgroundColour("light blue")
    self.attr3 = wx.ListItemAttr()
    self.attr3.SetBackgroundColour("pink")

    self.Bind(wx.EVT_LIST_ITEM_SELECTED, self.OnItemSelected)
    self.Bind(wx.EVT_LIST_ITEM_DESELECTED, self.OnItemDeselected)

  def updateList(self, heaterList):
    self.heaterList = heaterList
    self.valid = [True] * len(heaterList)
    self.currentItem = None
    self.parent.setItemSelected(None)
    i = self.GetFirstSelected()
    while i != -1:
      self.Select(i, False)
      i = self.GetFirstSelected()

    self.SetItemCount(len(heaterList))

  def setItemValidity(self, i, flag = False):
    if i < 0 or i >= len(self.heaterList):
      return

    self.valid[i] = flag
    self.Refresh()

  def OnItemSelected(self, event):
    self.currentItem = event.m_itemIndex
    self.parent.setItemSelected(self.currentItem)

  def OnItemDeselected(self, event):
    self.currentItem = None
    self.parent.setItemSelected(None)

  def getColumnText(self, index, col):
    item = self.GetItem(index, col)
    return item.GetText()

  def OnGetItemText(self, item, col):
    if item < 0 or item >= len(self.heaterList):
      return "Error - no heaters"

    s = self.heaterList[item]

    if col == 0:
      return s[0]
    elif col == 1:
      return s[1]
    elif col == 2:
      if s[2] == "1":
        return "True"
      else:
        return "False"

  def OnGetItemAttr(self, item):
    if not self.valid[item]:
      return self.attr3

    if item % 2 == 1:
      return self.attr2
    else:
      return None


class AddHeaterDlg(wx.Dialog):
  def __init__(self, parent, names, arduinoFlag):
    wx.Dialog.__init__(self, parent, wx.ID_ANY, "Add Heater", size = (400, 204))
    self.Bind(wx.EVT_CLOSE, self.onCancel)

    self.names = names
    self.arduinoFlag = arduinoFlag

    self.nameValid = False
    self.pinValid = False

    sz = wx.BoxSizer(wx.VERTICAL)
    gsz = wx.GridBagSizer()
    gsz.AddSpacer((20, 20), pos = (0, 0))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Heater Name:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    self.tcName = wx.TextCtrl(self, wx.ID_ANY, "")
    self.tcName.SetBackgroundColour("pink")
    self.tcName.Bind(wx.EVT_TEXT, self.onNameEntry)
    lsz.Add(self.tcName)
    self.tcName.SetToolTipString("Enter a unique name for this heater")

    gsz.Add(lsz, pos = (1, 1))

    lsz = wx.BoxSizer(wx.HORIZONTAL)
    st = wx.StaticText(self, wx.ID_ANY, "Pin:", size = (80, -1),
                       style = wx.ALIGN_RIGHT)
    lsz.Add(st)

    self.tcPin = wx.TextCtrl(self, wx.ID_ANY, "", style = wx.TE_RIGHT)
    self.tcPin.Bind(wx.EVT_TEXT, self.onPinEntry)
    self.tcPin.SetBackgroundColour("pink")
    lsz.Add(self.tcPin)
    self.tcPin.SetToolTipString("Enter a pin number/name for this heater")

    gsz.Add(lsz, pos = (3, 1))

    self.cbPwm = wx.CheckBox(self, wx.ID_ANY, "PWM")
    self.cbPwm.SetToolTipString("Help for PWM")

    gsz.AddSpacer((50, 15), pos = (1, 2))
    gsz.Add(self.cbPwm, pos = (1, 3))
    gsz.AddSpacer((20, 20), pos = (4, 4))

    sz.Add(gsz)
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

  def onPinEntry(self, evt):
    tc = evt.GetEventObject()
    self.validatePin(tc)
    self.checkDlgValidity()
    evt.Skip()

  def validatePin(self, tc):
    w = tc.GetValue().strip()
    if w == "":
      self.pinValid = False
    else:
      m = reInteger.match(w)
      if m:
        self.pinValid = True
      else:
        if self.arduinoFlag:
          m = rePin.match(w)
          if m:
            self.pinValid = True
          else:
            self.pinValid = False
        else:
          self.pinValid = False

    if self.pinValid:
      tc.SetBackgroundColour(wx.SystemSettings_GetColour(wx.SYS_COLOUR_WINDOW))
    else:
      tc.SetBackgroundColour("pink")
    tc.Refresh()

  def checkDlgValidity(self):
    if self.nameValid and self.pinValid:
      self.bSave.Enable(True)
    else:
      self.bSave.Enable(False)

  def getValues(self):
    nm = self.tcName.GetValue()
    pin = self.tcPin.GetValue()
    if self.cbPwm.IsChecked():
      pwm = "1"
    else:
      pwm = "0"

    return (nm, pin, pwm)

  def onSave(self, evt):
    self.EndModal(wx.ID_OK)

  def onCancel(self, evt):
    self.EndModal(wx.ID_CANCEL)


class CommunicationsPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.parent = parent
    self.id = idPg
    self.defaultBaud = '115200'

    self.bauds = ['19200', '38400', '57600', self.defaultBaud]

    self.labels = {'BAUD': "Baud Rate:", 'USB_SERIAL': "USB Serial",
                   'XONXOFF': "Use XON/XOFF Flow Control"}

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    ch = self.addChoice('BAUD', self.bauds, self.bauds.index(self.defaultBaud),
                        60, None, self.onChoice)
    sz.Add(ch, pos = (1, 1))

    k = 'USB_SERIAL'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (3, 1))

    k = 'XONXOFF'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (5, 1))

    self.SetSizer(sz)

  def insertValues(self, cfgValues):
    self.assertValid(True)
    for k in self.fieldValid.keys():
      self.fieldValid[k] = True

    for k in self.checkBoxes.keys():
      if k in cfgValues.keys() and cfgValues[k]:
        self.checkBoxes[k].SetValue(True)
      else:
        self.checkBoxes[k].SetValue(False)

    self.setChoice('BAUD', cfgValues, self.bauds, self.defaultBaud)

    self.assertModified(False)

  def saveValues(self, fp):
    self.assertModified(False)
    fp.write(
"\n/***************************************************************************\\\n\
*                                                                           *\n\
* 6. COMMUNICATION                                                          *\n\
*                                                                           *\n\
\\***************************************************************************/\n")

    k = 'BAUD'
    v = self.choices[k].GetSelection()
    if v != "":
      fp.write(defineValueFormat % (k, self.bauds[v]))

    for k in self.checkBoxes.keys():
      cb = self.checkBoxes[k]
      if cb.IsChecked():
        fp.write(defineBoolFormat % k)


class MiscellaneousPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.parent = parent
    self.id = idPg

    self.labels = {'MOTHERBOARD': "Motherboard", 'F_CPU': "CPU Clock Rate:",
                   'EECONFIG': "Enable EEPROM Storage",
                   'DEBUG': "Turn on debugging", 'BANG_BANG': "Enable",
                   'BANG_BANG_ON': "On PWM level:",
                   'BANG_BANG_OFF': "Off PWM level:",
                   'MOVEBUFFER_SIZE': "Move buffer size:",
                   'DC_EXTRUDER': "Heater:", 'DC_EXTRUDER_PWM': "PWM:",
                   'USE_WATCHDOG': "Use the watchdog timer",
                   'REFERENCE': "Analog Reference:",
                   'STEP_INTERRUPT_INTERRUPTIBLE': "STEP Interrupt",
                   'TH_COUNT': "Temperature History size:",
                   'FAST_PWM': "Fast PWM", 'ENDSTOP_STEPS': "Endstop steps:",
                   'CANNED_CYCLE': "Canned Cycle:"}

    self.prefixKeys = ['MOTHERBOARD', 'F_CPU']

    self.defaultClock = '16000000'
    self.clocks = ['8000000', self.defaultClock, '20000000']
    self.heaterNameNone = "<none>"
    self.heaterNames = [self.heaterNameNone]
    self.processors = []

    self.defaultRef = 'REFERENCE_AVCC'
    self.references = [self.defaultRef, 'REFERENCE_AREF',
                       'REFERENCE_1V1', 'REFERENCE_2V56']

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))
    sz.AddSpacer((40, 40), pos = (0, 2))
    sz.AddSpacer((40, 40), pos = (0, 4))
    sz.AddSpacer((20, 30), pos = (1, 0))
    sz.AddSpacer((20, 30), pos = (2, 0))
    sz.AddSpacer((20, 30), pos = (3, 0))
    sz.AddSpacer((20, 30), pos = (4, 0))
    sz.AddSpacer((20, 30), pos = (5, 0))
    sz.AddSpacer((20, 30), pos = (6, 0))

    labelWidth = 140

    k = 'MOTHERBOARD'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (1, 1))

    k = 'EECONFIG'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (2, 1))

    k = 'DEBUG'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (3, 1))

    k = 'USE_WATCHDOG'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (4, 1))

    k = 'STEP_INTERRUPT_INTERRUPTIBLE'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (5, 1))

    k = 'FAST_PWM'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sz.Add(cb, pos = (6, 1))

    b = wx.StaticBox(self, wx.ID_ANY, "BANG BANG Bed Control")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))

    k = 'BANG_BANG'
    cb = self.addCheckBox(k, None, self.onCheckBox)
    sbox.Add(cb, 1, wx.LEFT, 60)
    sbox.AddSpacer((5, 20))

    k = 'BANG_BANG_ON'
    tc = self.addTextCtrl(k, 80, None, self.onTextCtrlInteger)
    sbox.Add(tc)
    sbox.AddSpacer((5, 5))

    k = 'BANG_BANG_OFF'
    tc = self.addTextCtrl(k, 80, None, self.onTextCtrlInteger)
    sbox.Add(tc)
    sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (7, 1), flag = wx.ALIGN_CENTER_HORIZONTAL)

    k = 'F_CPU'
    ch = self.addChoice(k, self.clocks, self.clocks.index(self.defaultClock),
                        labelWidth, None, self.onChoice)
    sz.Add(ch, pos = (1, 3))

    k = 'REFERENCE'
    ch = self.addChoice(k, self.references, self.references.index(self.defaultRef),
                        labelWidth, None, self.onChoice)
    sz.Add(ch, pos = (2, 3))

    b = wx.StaticBox(self, wx.ID_ANY, "DC Motor Extruder")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))

    k = 'DC_EXTRUDER'
    ch = self.addChoice(k, self.heaterNames, 0, 60, None, self.onChoice)
    sbox.Add(ch)
    sbox.AddSpacer((5, 5))

    k = 'DC_EXTRUDER_PWM'
    tc = self.addTextCtrl(k, 60, None, self.onTextCtrlInteger)
    sbox.Add(tc)
    sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (7, 3), flag = wx.ALIGN_CENTER_HORIZONTAL)

    k = 'MOVEBUFFER_SIZE'
    tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlInteger)
    sz.Add(tc, pos = (1, 5))

    k = 'TH_COUNT'
    tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlInteger)
    sz.Add(tc, pos = (2, 5))

    k = 'ENDSTOP_STEPS'
    tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrlInteger)
    sz.Add(tc, pos = (3, 5))

    k = 'CANNED_CYCLE'
    tc = self.addTextCtrl(k, labelWidth, None, self.onTextCtrl)
    sz.Add(tc, pos = (4, 5))

    b = wx.StaticBox(self, wx.ID_ANY, "Processor Type(s):")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((125, 5))

    ht = "Choose the processor(s) this configuration will work with"
    for k in supportedCPUs:
      cb = self.addCheckBox(k, ht, self.onCheckBox)
      sbox.Add(cb)
      sbox.AddSpacer((5, 5))

    sbox.AddSpacer((5, 5))
    sz.Add(sbox, pos = (6, 5), span = (3, 1), flag = wx.ALIGN_CENTER_HORIZONTAL)

    self.SetSizer(sz)

  def setHeaters(self, hlist, cfgValues = None):
    k = 'DC_EXTRUDER'
    if cfgValues and k in cfgValues.keys():
      currentChoice = cfgValues[k][len("HEATER_"):]

    else:
      v = self.choices[k].GetSelection()
      currentChoice = self.heaterNames[v]
    h = [s[0] for s in hlist]
    self.heaterNames = [self.heaterNameNone] + h
    self.choices[k].Clear()
    for h in self.heaterNames:
      self.choices[k].Append(h)

    try:
      v = self.heaterNames.index(currentChoice)
    except:
      v = 0
    self.choices[k].SetSelection(v)

  def setProcessors(self, plist):
    self.processors = plist
    for p in supportedCPUs:
      if p in self.processors:
        self.checkBoxes[p].SetValue(True)
      else:
        self.checkBoxes[p].SetValue(False)

  def insertValues(self, cfgValues):
    self.assertValid(True)
    for k in self.fieldValid.keys():
      self.fieldValid[k] = True

    for k in self.checkBoxes.keys():
      if k in cfgValues.keys() and cfgValues[k]:
        self.checkBoxes[k].SetValue(True)
      else:
        self.checkBoxes[k].SetValue(False)

    for k in self.textControls.keys():
      if k in cfgValues.keys():
        self.textControls[k].SetValue(cfgValues[k])
      else:
        self.textControls[k].SetValue("")

    self.setChoice('F_CPU', cfgValues, self.clocks, self.defaultClock)
    self.setChoice('REFERENCE', cfgValues, self.references, self.defaultRef)

    self.assertModified(False)

  def savePrefixValues(self, fp):
    if len(self.processors) != 0:
      nest = 0
      for i in self.processors:
        fp.write("%s#ifndef __AVR_%s__\n" % (" " * nest, i))
        nest += 1
      fp.write("%s#error incorrect cpu type in Makefile!\n" % (" " * nest))
      for i in self.processors:
        nest -= 1
        fp.write("%s#endif\n" % (" " * nest))

      fp.write("\n\n")

    for k in self.checkBoxes.keys():
      if k not in self.prefixKeys:
        continue

      cb = self.checkBoxes[k]
      if cb.IsChecked():
        fp.write(defineBoolFormat % k)

    for k in self.textControls.keys():
      if k not in self.prefixKeys:
        continue

      v = self.textControls[k].GetValue()
      if v != "":
        fp.write(defineValueFormat % (k, v))

    k = 'F_CPU'
    v = self.choices[k].GetSelection()
    fp.write(defineULFormat % (k, self.clocks[v]))

  def saveValues(self, fp):
    self.assertModified(False)
    fp.write(
"\n/***************************************************************************\\\n\
*                                                                           *\n\
* 7. MISCELLANEOUS                                                          *\n\
*                                                                           *\n\
\\***************************************************************************/\n")
    for k in self.checkBoxes.keys():
      if k in self.prefixKeys:
        continue

      cb = self.checkBoxes[k]
      if cb.IsChecked():
        fp.write(defineBoolFormat % k)

    for k in self.textControls.keys():
      if k in self.prefixKeys:
        continue

      v = self.textControls[k].GetValue()
      if v != "":
        fp.write(defineValueFormat % (k, v))

    k = 'REFERENCE'
    v = self.choices[k].GetSelection()
    fp.write(defineValueFormat % (k, self.references[v]))

    k = 'DC_EXTRUDER'
    v = self.choices[k].GetSelection()
    if self.heaterNames[v] != self.heaterNameNone:
      fp.write(defineDCExtruderFormat % (k, self.heaterNames[v]))


class MyFrame(wx.Frame):
  def __init__(self):
    wx.Frame.__init__(self, None, -1, "Teacup Firmware Configurator",
                      size = (980, 450))
    self.Bind(wx.EVT_CLOSE, self.onClose)

    self.cfgValues = {}
    self.heaters = []
    self.dir = "."

    panel = wx.Panel(self, -1)

    sz = wx.BoxSizer(wx.HORIZONTAL)
    bsz = wx.BoxSizer(wx.VERTICAL)

    self.bLoadConfig = wx.Button(panel, wx.ID_ANY, "Load\nConfig", size = BSIZE)
    panel.Bind(wx.EVT_BUTTON, self.onLoadConfig, self.bLoadConfig)
    bsz.Add(self.bLoadConfig)
    self.bLoadConfig.SetToolTipString("Choose a configuration file and load its contents")

    bsz.AddSpacer((5, 5))

    self.bSaveConfig = wx.Button(panel, wx.ID_ANY, "Save\nConfig", size = BSIZE)
    panel.Bind(wx.EVT_BUTTON, self.onSaveConfig, self.bSaveConfig)
    bsz.Add(self.bSaveConfig)
    self.bSaveConfig.SetToolTipString("Save the current configuration into a C header file")

    self.nb = wx.Notebook(panel, wx.ID_ANY, size = (21, 21), style = wx.BK_DEFAULT)

    self.pages = []
    self.titles = []
    self.pageModified = []
    self.pageValid = []

    self.pgMech = MechanicalPage(self, self.nb, len(self.pages))
    text = "Mechanical"
    self.nb.AddPage(self.pgMech, text)
    self.pages.append(self.pgMech)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    self.pgAcc = AccelerationPage(self, self.nb, len(self.pages))
    text = "Acceleration"
    self.nb.AddPage(self.pgAcc, text)
    self.pages.append(self.pgAcc)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    self.pgPins = PinoutsPage(self, self.nb, len(self.pages))
    text = "Pinouts"
    self.nb.AddPage(self.pgPins, text)
    self.pages.append(self.pgPins)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    self.pgSensors = SensorsPage(self, self.nb, len(self.pages))
    text = "Temperature Sensors"
    self.nb.AddPage(self.pgSensors, text)
    self.pages.append(self.pgSensors)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    self.pgHeaters = HeatersPage(self, self.nb, len(self.pages))
    text = "Heaters"
    self.nb.AddPage(self.pgHeaters, text)
    self.pages.append(self.pgHeaters)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    self.pgCommunications = CommunicationsPage(self, self.nb, len(self.pages))
    text = "Communications"
    self.nb.AddPage(self.pgCommunications, text)
    self.pages.append(self.pgCommunications)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    self.pgMiscellaneous = MiscellaneousPage(self, self.nb, len(self.pages))
    text = "Miscellaneous"
    self.nb.AddPage(self.pgMiscellaneous, text)
    self.pages.append(self.pgMiscellaneous)
    self.titles.append(text)
    self.pageModified.append(False)
    self.pageValid.append(True)

    sz.Add(self.nb, 1, wx.EXPAND + wx.ALL, 5)
    sz.Add(bsz, 0, wx.ALL, 5)

    panel.SetSizer(sz)

  def buildHeaterPage(self, nb):
    pg = wx.Panel(nb, wx.ID_ANY)
    return pg

  def buildCommPage(self, nb):
    pg = wx.Panel(nb, wx.ID_ANY)
    return pg

  def buildMiscPage(self, nb):
    pg = wx.Panel(nb, wx.ID_ANY)
    return pg

  def assertModified(self, pg, flag = True):
    self.pageModified[pg] = flag
    self.modifyTab(pg)

  def assertValid(self, pg, flag = True):
    self.pageValid[pg] = flag
    self.modifyTab(pg)

    if False in self.pageValid:
      self.bSaveConfig.Enable(False)
    else:
      self.bSaveConfig.Enable(True)

  def modifyTab(self, pg):
    if self.pageModified[pg] and not self.pageValid[pg]:
      pfx = "?* "
    elif self.pageModified[pg]:
      pfx = "* "
    elif not self.pageValid[pg]:
      pfx = "? "
    else:
      pfx = ""

    self.nb.SetPageText(pg, pfx + self.titles[pg])

  def revalidatePins(self, flag):
    self.pgSensors.revalidatePins(flag)
    self.pgHeaters.revalidatePins(flag)

  def setHeaters(self, hlist):
    self.pgMiscellaneous.setHeaters(hlist)

  def onClose(self, evt):
    if not self.confirmLoseChanges("Exit"):
      return

    self.Destroy()

  def confirmLoseChanges(self, msg):
    if True not in self.pageModified:
      return True

    dlg = wx.MessageDialog(self, "Are you sure you want to " + msg +
                                 "?\nThere are changes that will be lost.",
                           "Changes pending",
                           wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
    rc = dlg.ShowModal()
    dlg.Destroy()

    if rc != wx.ID_YES:
      return False

    return True

  def onLoadConfig(self, evt):
    if not self.confirmLoseChanges("Load a new Configuration"):
      return

    wildcard = "C Header files (*.h)|*.h"

    dlg = wx.FileDialog(self, message = "Choose a Config file",
                        defaultDir = self.dir, defaultFile = "",
                        wildcard = wildcard, style = wx.OPEN | wx.CHANGE_DIR)

    path = None
    if dlg.ShowModal() == wx.ID_OK:
      path = dlg.GetPath()

    dlg.Destroy()
    if path is None:
      return

    self.dir = os.path.dirname(path)
    self.loadConfigFile(path)

    for pg in self.pages:
      pg.insertValues(self.cfgValues)

    self.pgSensors.setSensors(self.sensors)
    self.pgHeaters.setHeaters(self.heaters)
    self.pgMiscellaneous.setHeaters(self.heaters, self.cfgValues)
    self.pgMiscellaneous.setProcessors(self.processors)

  def loadConfigFile(self, fn):
    try:
      lst = list(open(fn))
    except:
      return False

    self.pgPins.setIncludeArduino(False)
    self.sensors = []
    self.heaters = []
    self.processors = []

    self.cfgValues = {}
    self.cfgValues['E_ABSOLUTE'] = False

    prevLines = ""
    for ln in lst:
      if ln.rstrip().endswith("\\"):
        prevLines += ln.rstrip()[:-1]
        continue

      if prevLines != "":
        ln = prevLines + ln
        prevLines = ""

      if ln.lstrip().startswith("//"):
        continue

      if ln.lstrip().startswith("#if"):
        m = re.findall(reAVR, ln)
        inv = []
        for p in m:
          if p in supportedCPUs:
            self.processors.append(p)
          else:
            inv.append(p)
        if len(inv) > 0:
          if len(inv) == 1:
            a = " is"
            b = "it"
          else:
            a = "s are"
            b = "them"
          dlg = wx.MessageDialog(self,
            "The following processor type %s not supported:\n   %s\n\
Please add %s to \"supportedCPUs\"" % (a, ", ".join(inv), b),
            "Unsupported Processor Type(s)",
            wx.OK + wx.ICON_INFORMATION)

          dlg.ShowModal()
          dlg.Destroy()
        continue

      if ln.lstrip().startswith("#define"):
        m = reDefQS.search(ln)
        if m:
          t = m.groups()
          if len(t) == 2:
            self.cfgValues[t[0]] = t[1]
            continue

        m = reDefine.search(ln)
        if m:
          t = m.groups()
          if len(t) == 2:
            self.cfgValues[t[0]] = t[1]
            continue

        m = reDefBool.search(ln)
        if m:
          t = m.groups()
          if len(t) == 1:
            self.cfgValues[t[0]] = True
      else:
        m = reDefTS.search(ln)
        if m:
          t = m.groups()
          if len(t) == 1:
            s = self.parseSensor(t[0])
            if s:
              self.sensors.append(s)
            continue

        m = reDefHT.search(ln)
        if m:
          t = m.groups()
          if len(t) == 1:
            s = self.parseHeater(t[0])
            if s:
              self.heaters.append(s)
            continue

        m = reIncArduino.search(ln)
        if m:
          self.pgPins.setIncludeArduino(True)

    return True

  def parseSensor(self, s):
    m = reSensor4.search(s)
    if m:
      t = m.groups()
      if len(t) == 4:
        return t
    m = reSensor3.search(s)
    if m:
      t = m.groups()
      if len(t) == 3:
        return t
    return None

  def parseHeater(self, s):
    m = reHeater.search(s)
    if m:
      t = m.groups()
      if len(t) == 3:
        return t
    return None

  def onSaveConfig(self, evt):
    wildcard = "C Header files (*.h)|*.h"

    dlg = wx.FileDialog(self, message = "Save as ...", defaultDir = self.dir,
                        defaultFile = "", wildcard = wildcard,
                        style = wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT)

    val = dlg.ShowModal()

    if val != wx.ID_OK:
      dlg.Destroy()
      return

    path = dlg.GetPath()
    dlg.Destroy()

    ext = os.path.splitext(os.path.basename(path))[1]
    self.dir = os.path.dirname(path)

    if ext == "":
      path += ".h"

    fp = file(path, 'w')

    timestamp = time.strftime("%Y-%b-%d %H:%M:%S", time.localtime())

    fp.write("// Generated by Teacup Configurator version %s - %s\n\n" %
             (VERSION, timestamp))

    self.pgMiscellaneous.savePrefixValues(fp)

    for pg in self.pages:
      pg.saveValues(fp)

    fp.close()

    dlg = wx.MessageDialog(self, "File \"%s\" successfully written" % path,
                           "Save Successful", wx.OK + wx.ICON_INFORMATION)
    dlg.ShowModal()
    dlg.Destroy()


if __name__ == '__main__':
  app = wx.PySimpleApp()
  frame = MyFrame()
  frame.Show(True)
  app.MainLoop()
