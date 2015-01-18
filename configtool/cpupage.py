
import wx
from configtool.page import Page
from configtool.data import supportedCPUs


class CpuPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.parent = parent
    self.id = idPg

    self.labels = {'F_CPU': "CPU Clock Rate:"}
    self.defaultClock = '16000000UL'
    self.clocks = ['8000000UL', self.defaultClock, '20000000UL']
    self.processors = []

    sz = wx.GridBagSizer()
    sz.AddSpacer((20, 40), pos = (0, 0))

    k = 'F_CPU'
    ch = self.addChoice(k, self.clocks, self.clocks.index(self.defaultClock),
                        100, self.onChoice)
    sz.Add(ch, pos = (1, 1))
    sz.AddSpacer((100, 10), pos = (1, 2))

    b = wx.StaticBox(self, wx.ID_ANY, "Processor Type(s)")
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)

    ht = "Choose the processor(s) this configuration will work with."
    for k in supportedCPUs:
      cb = self.addCheckBox(k, self.onCheckBox)
      cb.SetToolTipString(ht)
      sbox.Add(cb)
      sbox.AddSpacer((120, 5))

    sbox.AddSpacer((5, 5))
    sz.Add(sbox, pos = (1, 3), span = (3, 1))

    self.SetSizer(sz)
    self.enableAll(False)

  def setProcessors(self, plist):
    self.processors = plist
    for p in supportedCPUs:
      if p in self.processors:
        self.checkBoxes[p].SetValue(True)
      else:
        self.checkBoxes[p].SetValue(False)

  def getProcessors(self):
    plist = []
    for p in supportedCPUs:
      if self.checkBoxes[p].IsChecked():
        plist.append(p)

    return plist

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

    self.setChoice('F_CPU', cfgValues, self.defaultClock)

    self.assertModified(False)
