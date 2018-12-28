# coding=utf-8

import wx
from configtool.page import Page


class DisplayPage(wx.Panel, Page):
    def __init__(self, parent, nb, idPg, font):
        wx.Panel.__init__(self, nb, wx.ID_ANY)
        Page.__init__(self, font)
        self.parent = parent
        self.id = idPg

        self.labels = {
            "DISPLAY_BUS": "Display Bus:",
            "DISPLAY_TYPE": "Display Type:",
            "DISPLAY_BUS_4BIT": "Direct with 4 pins",
            "DISPLAY_BUS_8BIT": "Direct with 8 pins",
            "DISPLAY_BUS_I2C": "I²C ( = TWI)",
            "DISPLAY_BUS_SPI": "SPI",
            "DISPLAY_TYPE_SSD1306": "SSD1306 O-LED, 128x32 pixels",
            "DISPLAY_TYPE_HD44780": "HD44780 or 1602A, 16x2 characters",
            "DISPLAY_RS_PIN": "RS pin",
            "DISPLAY_RW_PIN": "R/W pin",
            "DISPLAY_E_PIN": "E pin",
            "DISPLAY_D4_PIN": "D4 pin",
            "DISPLAY_D5_PIN": "D5 pin",
            "DISPLAY_D6_PIN": "D6 pin",
            "DISPLAY_D7_PIN": "D7 pin",
        }

        sz = wx.GridBagSizer()
        sz.Add((20, 40), pos=(0, 0))

        ch = self.addBoolChoice(
            "DISPLAY_BUS", True, 100, self.onBusChoice, size=(160, -1)
        )
        sz.Add(ch, pos=(1, 1))
        sz.Add((100, 10), pos=(1, 2))

        ch = self.addBoolChoice(
            "DISPLAY_TYPE", False, 100, self.onChoice, size=(240, -1)
        )
        sz.Add(ch, pos=(1, 3))

        b = wx.StaticBox(self, wx.ID_ANY, "Direct 4-bit Bus Pins:")
        b.SetFont(font)
        self.pinbox = wx.StaticBoxSizer(b, wx.VERTICAL)
        self.pinbox.Add((5, 5))
        for k in (
            "DISPLAY_RS_PIN",
            "DISPLAY_RW_PIN",
            "DISPLAY_E_PIN",
            "DISPLAY_D4_PIN",
            "DISPLAY_D5_PIN",
            "DISPLAY_D6_PIN",
            "DISPLAY_D7_PIN",
        ):
            tc = self.addPinChoice(k, 200)
            self.pinbox.Add(tc)
            self.pinbox.Add((5, 5))
        sz.Add(self.pinbox, pos=(3, 1))

        self.SetSizer(sz)
        self.enableAll(False)

    def onBusChoice(self, evt):
        choice = self.boolChoices["DISPLAY_BUS"]
        if choice.GetClientData(choice.GetSelection()):
            self.boolChoices["DISPLAY_TYPE"].Enable(True)
        else:
            self.boolChoices["DISPLAY_TYPE"].Enable(False)
        self.adjustPinVisibility()

        Page.onChoice(self, evt)

    def adjustPinVisibility(self):
        visible = False

        choice = self.boolChoices["DISPLAY_BUS"]
        if choice.GetSelection() >= 0:
            selection = choice.GetClientData(choice.GetSelection())
            if selection == "DISPLAY_BUS_4BIT":
                visible = True

        self.pinbox.ShowItems(visible)

    def insertValues(self, cfgValues):
        Page.insertValues(self, cfgValues)
        self.adjustPinVisibility()
