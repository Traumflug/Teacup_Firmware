
import wx
from configtool.page import Page


class CommunicationsPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.id = idPg
    self.defaultBaud = '115200'

    self.bauds = ['19200', '38400', '57600', self.defaultBaud]

    self.labels = {'XONXOFF': "XON/XOFF Flow Control", 'BAUD': "Baud Rate:",
                   'USB_SERIAL': "USB Serial"}

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    k = 'USB_SERIAL'
    cb = self.addCheckBox(k, self.onUSBCheckBox)
    sz.Add(cb, pos = (1, 1))

    ch = self.addChoice('BAUD', self.bauds, self.bauds.index(self.defaultBaud),
                        80, self.onChoice)
    sz.Add(ch, pos = (1, 3))

    cb = self.addCheckBox('XONXOFF', self.onCheckBox)
    sz.Add(cb, pos = (3, 3))

    sz.AddSpacer((100, 10), pos = (2, 2))

    self.SetSizer(sz)
    self.enableAll(False)

  def onUSBCheckBox(self, evt):
    self.assertModified(True)
    f = not self.checkBoxes['USB_SERIAL'].IsChecked()
    self.checkBoxes['XONXOFF'].Enable(f)
    self.choices['BAUD'].Enable(f)
    evt.Skip()

  def insertValues(self, cfgValues):
    Page.insertValues(self, cfgValues)

    k = 'BAUD'
    self.setChoice(k, cfgValues, self.defaultBaud)
    if k in cfgValues.keys():
      self.choicesOriginal[k] = cfgValues[k]

    if self.checkBoxes['USB_SERIAL'].IsChecked():
      self.checkBoxes['XONXOFF'].Enable(False)
      self.choices['BAUD'].Enable(False)

  def getValues(self):
    result = Page.getValues(self)

    if result['USB_SERIAL']:
      result['BAUD'][1] = False
      result['XONXOFF'] = False

    return result
