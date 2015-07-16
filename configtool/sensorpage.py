
import wx
from configtool.page import Page
from configtool.data import pinNames, BSIZESMALL
from sensorlist import SensorList
from addsensordlg import AddSensorDlg


class SensorsPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, heatersPage, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.heatersPage = heatersPage
    self.font = font
    self.id = idPg

    self.sensorTypeKeys = {'TT_MAX6675': 'TEMP_MAX6675',
                           'TT_THERMISTOR': 'TEMP_THERMISTOR',
                           'TT_AD595': 'TEMP_AD595', 'TT_PT100': 'TEMP_PT100',
                           'TT_INTERCOM': 'TEMP_INTERCOM'}
    self.labels = {'TEMP_MAX6675': "MAX6675", 'TEMP_THERMISTOR': "Thermistor",
                   'TEMP_AD595': "AD595", 'TEMP_PT100': "PT100",
                   'TEMP_INTERCOM': "Intercom"}

    self.validPins = pinNames

    sz = wx.GridBagSizer()
    sz.AddSpacer((10, 10), pos = (0, 0))

    self.sensors = []

    self.lb = SensorList(self, font)
    sz.Add(self.lb, pos = (1, 1))
    sz.AddSpacer((20, 20), pos = (1, 2))

    bsz = wx.BoxSizer(wx.VERTICAL)
    self.bAdd = wx.Button(self, wx.ID_ANY, "Add", size = BSIZESMALL)
    self.bAdd.SetBackgroundColour(self.deco.getBackgroundColour())
    self.bAdd.SetFont(font)
    self.Bind(wx.EVT_BUTTON, self.doAdd, self.bAdd)
    self.bAdd.Enable(False)
    self.bAdd.SetToolTipString("Add a sensor to the configuration.")

    bsz.Add(self.bAdd)

    bsz.AddSpacer((10, 10))
    self.bModify = wx.Button(self, wx.ID_ANY, "Modify", size = BSIZESMALL)
    self.bModify.SetBackgroundColour(self.deco.getBackgroundColour())
    self.bModify.SetFont(font)
    self.bModify.Enable(False)
    self.Bind(wx.EVT_BUTTON, self.doModify, self.bModify)
    bsz.Add(self.bModify)
    self.bModify.SetToolTipString("Modify the selected temperature sensor.")

    bsz.AddSpacer((10, 10))
    self.bDelete = wx.Button(self, wx.ID_ANY, "Delete", size = BSIZESMALL)
    self.bDelete.SetBackgroundColour(self.deco.getBackgroundColour())
    self.bDelete.SetFont(font)
    self.bDelete.Enable(False)
    self.Bind(wx.EVT_BUTTON, self.doDelete, self.bDelete)
    bsz.Add(self.bDelete)
    self.bDelete.SetToolTipString("Remove the selected temperature sensor "
                                  "from the configuration.")

    sz.Add(bsz, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def setItemSelected(self, n):
    self.selection = n
    if n is None:
      self.bDelete.Enable(False)
      self.bModify.Enable(False)
    else:
      self.bDelete.Enable(True)
      self.bModify.Enable(True)

  def doAdd(self, evt):
    nm = []
    for s in self.sensors:
      nm.append(s[0])

    dlg = AddSensorDlg(self, nm, self.validPins, self.heatersPage, self.font)
    rc = dlg.ShowModal()
    if rc == wx.ID_OK:
      tt = dlg.getValues()

    dlg.Destroy()

    if rc != wx.ID_OK:
      return

    self.sensors.append(tt)
    self.lb.updateList(self.sensors)
    self.validateTable()
    self.assertModified(True)

  def doModify(self, evt):
    if self.selection is None:
      return
    nm = []
    for s in self.sensors:
      nm.append(s[0])

    s = self.sensors[self.selection]
    if s[3] is None:
      params = []
    else:
      params = s[3]

    dlg = AddSensorDlg(self, nm, self.validPins, self.heatersPage, self.font,
                       name = s[0], stype = s[1], pin = s[2],
                       params = params, modify = True)
    rc = dlg.ShowModal()
    if rc == wx.ID_OK:
      tt = dlg.getValues()

    dlg.Destroy()

    if rc != wx.ID_OK:
      return

    self.assertModified(True)

    self.sensors[self.selection] = tt
    self.lb.updateList(self.sensors)
    self.validateTable()
    self.assertModified(True)

  def doDelete(self, evt):
    if self.selection is None:
      return

    self.assertModified(True)

    del self.sensors[self.selection]
    self.lb.updateList(self.sensors)
    self.validateTable()
    self.assertModified(True)

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    self.bAdd.Enable(True)

  def setSensors(self, sensors):
    self.sensors = sensors
    self.lb.updateList(self.sensors)
    self.validateTable()

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

  def getValues(self):
    result = Page.getValues(self)

    values = {}
    for k in self.sensorTypeKeys.values():
      values[k] = False

    for s in self.sensors:
      values[self.sensorTypeKeys[s[1]]] = True

    for v in values.keys():
      result[v] = values[v]

    return result
