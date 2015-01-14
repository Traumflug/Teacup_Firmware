
import wx
from configtool.page import Page


class CommunicationsPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self)
    self.parent = parent
    self.id = idPg
    self.defaultBaud = '115200'

    self.bauds = ['19200', '38400', '57600', self.defaultBaud]

    self.labels = {'BAUD': "Baud Rate:", 'USB_SERIAL': "USB Serial"}

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    ch = self.addChoice('BAUD', self.bauds, self.bauds.index(self.defaultBaud),
                        60, self.onChoice)
    sz.Add(ch, pos = (1, 1))

    sz.AddSpacer((100, 10), pos = (1, 2))

    k = 'USB_SERIAL'
    cb = self.addCheckBox(k, self.onCheckBox)
    sz.Add(cb, pos = (1, 3))

    self.SetSizer(sz)
    self.enableAll(False)

  def insertValues(self, cfgValues):
    self.assertValid(True)
    self.enableAll(True)
    for k in self.fieldValid.keys():
      self.fieldValid[k] = True

    for k in self.checkBoxes.keys():
      if k in cfgValues.keys() and cfgValues[k]:
        self.checkBoxes[k].SetValue(True)
      else:
        self.checkBoxes[k].SetValue(False)

    self.setChoice('BAUD', cfgValues, self.defaultBaud)

    self.assertModified(False)
