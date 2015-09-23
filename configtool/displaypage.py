
import wx
from configtool.page import Page

class DisplayPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.id = idPg

    self.labels = {'DISPLAY_BUS': "Display Bus:",
                   'DISPLAY_TYPE': "Display Type:"}
    self.buses = []
    self.types = []

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    k = 'DISPLAY_BUS'
    ch = self.addChoice(k, self.buses, 0, 100, self.onChoice, size = (140, -1))
    sz.Add(ch, pos = (1, 1))
    sz.AddSpacer((100, 10), pos = (1, 2))

    k = 'DISPLAY_TYPE'
    ch = self.addChoice(k, self.types, 0, 100, self.onChoice,
                        size = (140, -1))
    sz.Add(ch, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def setCandidateDisplayBuses(self, busList):
    k = 'DISPLAY_BUS'
    self.choices[k].Clear()
    for p in busList:
      self.choices[k].Append(p)
    self.buses = busList

  def setCandidateDisplayTypes(self, typeList):
    k = 'DISPLAY_TYPE'
    self.choices[k].Clear()
    for c in typeList:
      self.choices[k].Append(c)
    self.types = typeList

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    if len(self.buses) > 0:
      self.setChoice('DISPLAY_BUS', cfgValues, self.buses[0])
    if len(self.types) > 0:
      self.setChoice('DISPLAY_TYPE', cfgValues, self.types[0])
