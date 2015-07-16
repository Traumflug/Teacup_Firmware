
import wx
from configtool.page import Page


class CpuPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.id = idPg

    self.labels = {'F_CPU': "CPU Clock Rate:", 'CPU': "Processor Type:"}
    self.clocks = []
    self.processors = []

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    k = 'F_CPU'
    ch = self.addChoice(k, self.clocks, 0, 100, self.onChoice, size = (140, -1))
    sz.Add(ch, pos = (1, 1))
    sz.AddSpacer((100, 10), pos = (1, 2))

    k = 'CPU'
    ch = self.addChoice(k, self.processors, 0, 100, self.onChoice,
                        size = (140, -1))
    sz.Add(ch, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def setCandidateProcessors(self, plist):
    k = 'CPU'
    self.choices[k].Clear()
    for p in plist:
      self.choices[k].Append(p)
    self.processors = plist

  def setCandidateClocks(self, clist):
    k = 'F_CPU'
    self.choices[k].Clear()
    for c in clist:
      self.choices[k].Append(c)
    self.clocks = clist

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    if len(self.clocks) > 0:
      self.setChoice('F_CPU', cfgValues, self.clocks[0])
    if len(self.processors) > 0:
      self.setChoice('CPU', cfgValues, self.processors[0])
