
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

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    ch = self.addBoolChoice('DISPLAY_BUS', True, 100, self.onBusChoice,
                            size = (140, -1))
    sz.Add(ch, pos = (1, 1))
    sz.AddSpacer((100, 10), pos = (1, 2))

    ch = self.addBoolChoice('DISPLAY_TYPE', False, 100, self.onChoice,
                            size = (200, -1))
    sz.Add(ch, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def onBusChoice(self, evt):
    if self.boolChoices['DISPLAY_BUS'].GetStringSelection().startswith('('):
      self.boolChoices['DISPLAY_TYPE'].Enable(False)
    else:
      self.boolChoices['DISPLAY_TYPE'].Enable(True)

    Page.onChoice(self, evt)
