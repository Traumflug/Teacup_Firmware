
import wx
from configtool.page import Page

class DisplayPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.id = idPg

    self.displayBusKeys = ['DISPLAY_BUS_4BIT', 'DISPLAY_BUS_8BIT',
                           'DISPLAY_BUS_I2C', 'DISPLAY_BUS_SPI']

    self.labels = {'DISPLAY_BUS': "Display Bus:",
                   'DISPLAY_TYPE': "Display Type:"}
    self.types = []

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    k = 'DISPLAY_BUS'
    ch = self.addChoice(k, ['(disabled)'] + self.displayBusKeys, 0, 100,
                        self.onChoice, size = (140, -1))
    sz.Add(ch, pos = (1, 1))
    sz.AddSpacer((100, 10), pos = (1, 2))

    k = 'DISPLAY_TYPE'
    ch = self.addChoice(k, self.types, 0, 100, self.onChoice,
                        size = (140, -1))
    sz.Add(ch, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def setCandidateDisplayTypes(self, typeList):
    k = 'DISPLAY_TYPE'
    self.choices[k].Clear()
    for c in typeList:
      self.choices[k].Append(c)
    self.types = typeList

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    k = 'DISPLAY_BUS'
    for tag in self.displayBusKeys:
      if tag in cfgValues.keys() and cfgValues[tag]:
        self.setChoice(k, cfgValues, tag)
        break

    if len(self.types) > 0:
      self.setChoice('DISPLAY_TYPE', cfgValues, self.types[0])

  def getValues(self):
    result = Page.getValues(self)

    # Convert values to a set of booleans.
    for k in ('DISPLAY_BUS', ):
      del result[k]

      choice = self.choices[k]
      for i in range(choice.GetCount()):
        result[choice.GetString(i)] = (i == choice.GetSelection())

    return result
