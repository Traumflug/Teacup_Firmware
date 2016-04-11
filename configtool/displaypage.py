
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

    self.displayTypeKeys = ['DISPLAY_TYPE_SSD1306', 'DISPLAY_TYPE_LCD1302']

    self.labels = {'DISPLAY_BUS': "Display Bus:",
                   'DISPLAY_TYPE': "Display Type:"}

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    k = 'DISPLAY_BUS'
    ch = self.addChoice(k, ['(disabled)'] + self.displayBusKeys, 0, 100,
                        self.onChoice, size = (140, -1))
    sz.Add(ch, pos = (1, 1))
    sz.AddSpacer((100, 10), pos = (1, 2))

    k = 'DISPLAY_TYPE'
    ch = self.addChoice(k, self.displayTypeKeys, 0, 100, self.onChoice,
                        size = (200, -1))
    sz.Add(ch, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    k = 'DISPLAY_BUS'
    for tag in self.displayBusKeys:
      if tag in cfgValues.keys() and cfgValues[tag]:
        self.setChoice(k, cfgValues, tag)
        break

    k = 'DISPLAY_TYPE'
    for tag in self.displayTypeKeys:
      if tag in cfgValues.keys() and cfgValues[tag]:
        self.setChoice(k, cfgValues, tag)
        break

  def getValues(self):
    result = Page.getValues(self)

    # Convert values to a set of booleans.
    for k in self.choices:
      del result[k]

      choice = self.choices[k]
      for i in range(choice.GetCount()):
        result[choice.GetString(i)] = (i == choice.GetSelection())

    return result
