
import wx
from configtool.page import Page
from configtool.data import pinNames, BSIZESMALL
from sensorlist import SensorList
from addsensordlg import AddSensorDlg


class SensorsPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.parent = parent
    self.id = idPg

    self.sensorTypeKeys = ['TEMP_MAX6675', 'TEMP_THERMISTOR', 'TEMP_AD595',
                           'TEMP_PT100', 'TEMP_INTERCOM']
    self.sensorType = {'TEMP_MAX6675': "TT_MAX6675",
                        'TEMP_THERMISTOR': "TT_THERMISTOR",
                        'TEMP_AD595': "TT_AD595",
                        'TEMP_PT100': "TT_PT100",
                        'TEMP_INTERCOM': "TT_INTERCOM"}
    self.labels = {'TEMP_MAX6675': "MAX6675", 'TEMP_THERMISTOR': "Thermistor",
                   'TEMP_AD595': "AD595", 'TEMP_PT100': "PT100",
                   'TEMP_INTERCOM': "Intercom"}

    self.validPins = pinNames

    sz = wx.GridBagSizer()
    sz.AddSpacer((30, 30), pos = (0, 0))

    self.sensors = []

    b = wx.StaticBox(self, wx.ID_ANY, "Sensor Types")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.sensorTypeKeys:
      cb = self.addCheckBox(k, self.onCheckBoxSensorType)
      sbox.Add(cb, 1, wx.LEFT + wx.RIGHT, 10)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 1), span = (5, 1))

    self.lb = SensorList(self)
    sz.Add(self.lb, pos = (7, 1), span = (1, 3))

    bsz = wx.BoxSizer(wx.VERTICAL)
    self.bAdd = wx.Button(self, wx.ID_ANY, "Add", size = BSIZESMALL)
    self.Bind(wx.EVT_BUTTON, self.doAdd, self.bAdd)
    self.bAdd.Enable(False)
    self.bAdd.SetToolTipString("Add a sensor to the configuration.")

    bsz.Add(self.bAdd)

    bsz.AddSpacer((10, 10))
    self.bDelete = wx.Button(self, wx.ID_ANY, "Delete", size = BSIZESMALL)
    self.bDelete.Enable(False)
    self.Bind(wx.EVT_BUTTON, self.doDelete, self.bDelete)
    bsz.Add(self.bDelete)
    self.bDelete.SetToolTipString("Remove the selected temperature sensor "
                                  "from the configuration.")

    sz.Add(bsz, pos = (7, 4))

    self.SetSizer(sz)
    self.enableAll(False)

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

    dlg = AddSensorDlg(self, nm, sl, self.validPins)
    rc = dlg.ShowModal()
    if rc == wx.ID_OK:
      tt = dlg.getValues()

    dlg.Destroy()

    if rc != wx.ID_OK:
      return

    self.sensors.append(tt)
    self.lb.updateList(self.sensors)
    self.validateTable()
    self.limitSensorTypeControls()

  def doDelete(self, evt):
    if self.selection is None:
      return

    self.assertModified(True)

    del self.sensors[self.selection]
    self.lb.updateList(self.sensors)
    self.validateTable()
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
    self.enableAll(True)
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
    self.validateTable()
    self.limitSensorTypeControls()

  def setCandidatePins(self, plist):
    if not plist or len(plist) == 0:
      self.validPins = pinNames
    else:
      self.validPins = plist

    self.validateTable()

  def validateTable(self):
    self.lb.setTableValidity(True)
    self.setFieldValidity('SENSORLIST', True)
    for i in range(len(self.sensors)):
      if self.sensors[i][2] not in self.validPins:
        self.lb.setRowValidity(i, False)
        self.setFieldValidity('SENSORLIST', False)

  def setHelpText(self, ht):
    Page.setHelpText(self, ht)

    k = 'DEFINE_TEMP_SENSOR'
    if k in ht.keys():
      self.bAdd.SetToolTipString(ht[k])
