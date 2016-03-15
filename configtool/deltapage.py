
import wx
from configtool.data import BSIZE
from configtool.page import Page

class DeltaPage(wx.Panel, Page):
  def __init__(self, parent, nb, idPg, font):
    wx.Panel.__init__(self, nb, wx.ID_ANY)
    Page.__init__(self, font)
    self.id = idPg
    self.parent = parent
    self.font = font

    self.geoKeys = ['DEFAULT_DELTA_DIAGONAL_ROD','DEFAULT_DELTA_RADIUS',
					'TOWER_X_ANGLE_DEG','TOWER_Y_ANGLE_DEG','TOWER_Z_ANGLE_DEG']

    self.labels = {'DEFAULT_DELTA_DIAGONAL_ROD':'Rod length:',
					'DEFAULT_DELTA_RADIUS':'Delta radius:',
					'TOWER_X_ANGLE_DEG':'X tower angle:',
					'TOWER_Y_ANGLE_DEG':'Y tower angle:',
					'TOWER_Z_ANGLE_DEG':'Z tower angle:'}

    labelWidth = 100;

    sz = wx.GridBagSizer()
    sz.AddSpacer((10, 10), pos = (0, 0))
    sz.AddSpacer((90, 10), pos = (0, 4))

    b = wx.StaticBox(self, wx.ID_ANY, "Delta Geometry")
    b.SetFont(font)
    sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
    sbox.AddSpacer((5, 5))
    for k in self.geoKeys:
      tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlFloat)
      sbox.Add(tc)
      sbox.AddSpacer((5, 5))

    sz.Add(sbox, pos = (1, 1))

    self.enableAll(False)
    self.SetSizer(sz)

  def enableAll(self, flag = True):
    Page.enableAll(self, flag)

  def setHelpText(self, ht):
    Page.setHelpText(self, ht)

  def insertValues(self, cfgValues):
    self.assertValid(True)
    for k in self.fieldValid.keys():
      self.fieldValid[k] = True
    for k in self.textControls.keys():
      if k in cfgValues.keys():
        self.textControls[k].SetValue(cfgValues[k])
      else:
        self.textControls[k].SetValue("")

    self.assertModified(False)
    self.enableAll(True)

  def getValues(self):
    result = Page.getValues(self)
    return result
