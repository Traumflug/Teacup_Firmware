
# coding=utf-8

import wx
from configtool.page import Page

class DisplayPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.id = idPg

    self.labels = {'DISPLAY_BUS': "Display Bus:",
                   'DISPLAY_TYPE': "Display Type:",
                   'DISPLAY_BUS_4BIT': "Direct with 4 pins",
                   'DISPLAY_BUS_8BIT': "Direct with 8 pins",
                   'DISPLAY_BUS_I2C': "I²C ( = TWI)",
                   'DISPLAY_BUS_SPI': "SPI",
                   'DISPLAY_TYPE_SSD1306': "SSD1306 O-LED, 128x32",
                   'DISPLAY_TYPE_LCD1302': "LCD 1302"}

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    ch = self.addBoolChoice('DISPLAY_BUS', True, 100, self.onBusChoice,
                            size = (160, -1))
    sz.Add(ch, pos = (1, 1))
    sz.AddSpacer((100, 10), pos = (1, 2))

    ch = self.addBoolChoice('DISPLAY_TYPE', False, 100, self.onChoice,
                            size = (200, -1))
    sz.Add(ch, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def onBusChoice(self, evt):
    choice = self.boolChoices['DISPLAY_BUS']
    if choice.GetClientData(choice.GetSelection()):
      self.boolChoices['DISPLAY_TYPE'].Enable(True)
    else:
      self.boolChoices['DISPLAY_TYPE'].Enable(False)

    Page.onChoice(self, evt)
