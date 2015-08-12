
import wx
from configtool.page import Page
from configtool.data import pinNames, BSIZESMALL
from configtool.heaterlist import HeaterList
from configtool.addheaterdlg import AddHeaterDlg


class HeatersPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.font = font
    self.id = idPg

    sz = wx.GridBagSizer()
    sz.AddSpacer((30, 30), pos = (0, 0))

    self.heaters = []
    self.validPins = pinNames

    self.lb = HeaterList(self, font)
    sz.Add(self.lb, pos = (1, 1))
    sz.AddSpacer((20, 20), pos = (1, 2))

    bsz = wx.BoxSizer(wx.VERTICAL)

    self.bAdd = wx.Button(self, wx.ID_ANY, "Add", size = BSIZESMALL)
    self.bAdd.SetBackgroundColour(self.deco.getBackgroundColour())
    self.bAdd.SetFont(font)
    self.Bind(wx.EVT_BUTTON, self.doAdd, self.bAdd)
    self.bAdd.SetToolTipString("Add a heater to the configuration.")
    bsz.Add(self.bAdd)

    bsz.AddSpacer((10, 10))

    self.bModify = wx.Button(self, wx.ID_ANY, "Modify", size = BSIZESMALL)
    self.bModify.SetBackgroundColour(self.deco.getBackgroundColour())
    self.bModify.SetFont(font)
    self.bModify.Enable(False)
    self.Bind(wx.EVT_BUTTON, self.doModify, self.bModify)
    self.bModify.SetToolTipString("Modify the selected heater.")
    bsz.Add(self.bModify)

    bsz.AddSpacer((10, 10))

    self.bDelete = wx.Button(self, wx.ID_ANY, "Delete", size = BSIZESMALL)
    self.bDelete.SetBackgroundColour(self.deco.getBackgroundColour())
    self.bDelete.SetFont(font)
    self.bDelete.Enable(False)
    self.Bind(wx.EVT_BUTTON, self.doDelete, self.bDelete)
    self.bDelete.SetToolTipString("Remove the selected heater from the "
                                  "configuration.")
    bsz.Add(self.bDelete)

    sz.Add(bsz, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def enableAll(self, flag = True):
    self.bAdd.Enable(flag)
    Page.enableAll(self, flag)

  def setItemSelected(self, n):
    self.selection = n
    if n is None:
      self.bDelete.Enable(False)
      self.bModify.Enable(False)
    else:
      self.bDelete.Enable(True)
      self.bModify.Enable(True)

  def getFreePins(self):
    freePins = [] + self.validPins
    usedPins = []
    for s in self.heaters:
      usedPins.append(s[1])

    for p in usedPins:
      if p in freePins:
        freePins.remove(p)

    return freePins

  def doAdd(self, evt):
    nm = []
    for s in self.heaters:
      nm.append(s[0])

    dlg = AddHeaterDlg(self, nm, self.getFreePins(), self.font)
    rc = dlg.ShowModal()
    if rc == wx.ID_OK:
      ht = dlg.getValues()

    dlg.Destroy()

    if rc != wx.ID_OK:
      return

    self.heaters.append(ht)
    self.lb.updateList(self.heaters)
    self.validateTable()
    self.parent.setHeaters(self.heaters)
    self.assertModified(True)

  def doModify(self, evt):
    if self.selection is None:
      return

    nm = []
    for s in self.heaters:
      nm.append(s[0])

    h = self.heaters[self.selection]

    dlg = AddHeaterDlg(self, nm, [h[1]] + self.getFreePins(), self.font,
                       name = h[0], pin = h[1], invert = h[2], pwm = h[3])
    rc = dlg.ShowModal()
    if rc == wx.ID_OK:
      ht = dlg.getValues()

    dlg.Destroy()

    if rc != wx.ID_OK:
      return

    self.heaters[self.selection] = ht
    self.lb.updateList(self.heaters)
    self.validateTable()
    self.parent.setHeaters(self.heaters)
    self.assertModified(True)

  def doDelete(self, evt):
    if self.selection is None:
      return

    self.assertModified(True)

    del self.heaters[self.selection]
    self.lb.updateList(self.heaters)
    self.validateTable()
    self.parent.setHeaters(self.heaters)
    self.assertModified(True)

  def setHeaters(self, heaters):
    self.heaters = heaters
    self.lb.updateList(self.heaters)
    self.validateTable()
    self.parent.setHeaters(self.heaters)

  def setCandidatePins(self, plist):
    if not plist or len(plist) == 0:
      self.validPins = pinNames
    else:
      self.validPins = plist

    self.validateTable()

  def heaterNames(self):
    heaterNames = []
    for heater in self.heaters:
      heaterNames.append(heater[0])

    return heaterNames

  def validateTable(self):
    self.lb.setTableValidity(True)
    self.setFieldValidity('HEATERLIST', True)
    for i in range(len(self.heaters)):
      if self.heaters[i][1] not in self.validPins:
        self.lb.setRowValidity(i, False)
        self.setFieldValidity('HEATERLIST', False)

  def setHelpText(self, ht):
    Page.setHelpText(self, ht)

    k = 'DEFINE_HEATER'
    if k in ht.keys():
      self.bAdd.SetToolTipString(ht[k])
