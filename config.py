#!/usr/bin/env python

import wx
import os.path
import inspect

cmd_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile(
                              inspect.currentframe()))[0]))

from configtool.printerpanel import PrinterPanel
from configtool.boardpanel import BoardPanel
from configtool.data import VERSION

ID_LOAD_PRINTER = 1000
ID_SAVE_PRINTER = 1001
ID_SAVE_PRINTER_AS = 1002
ID_LOAD_BOARD = 1010
ID_SAVE_BOARD = 1011
ID_SAVE_BOARD_AS = 1012


class ConfigFrame(wx.Frame):
  def __init__(self):
    wx.Frame.__init__(self, None, -1,
                      "Teacup Firmware Configurator - " + VERSION,
                      size = (880, 500))
    self.Bind(wx.EVT_CLOSE, self.onClose)

    panel = wx.Panel(self, -1)

    self.heaters = []

    sz = wx.BoxSizer(wx.HORIZONTAL)

    self.nb = wx.Notebook(panel, wx.ID_ANY, size = (880, 500),
                          style = wx.BK_DEFAULT)

    self.pgPrinter = PrinterPanel(self, self.nb, cmd_folder)
    self.nb.AddPage(self.pgPrinter, "Printer")

    self.pgBoard = BoardPanel(self, self.nb, cmd_folder)
    self.nb.AddPage(self.pgBoard, "Board")

    panel.Fit()

    sz.Add(self.nb, 1, wx.EXPAND + wx.ALL, 5)
    self.SetSizer(sz)
    self.makeMenu()

  def onClose(self, evt):
    if not self.pgPrinter.confirmLoseChanges("exit"):
      return

    if not self.pgBoard.confirmLoseChanges("exit"):
      return

    self.Destroy()

  def setPrinterTabText(self, txt):
    self.nb.SetPageText(0, txt)

  def setBoardTabText(self, txt):
    self.nb.SetPageText(1, txt)

  def setHeaters(self, ht):
    self.heaters = ht
    self.pgPrinter.setHeaters(ht)

  def makeMenu(self):
    file_menu = wx.Menu()

    file_menu.Append(ID_LOAD_PRINTER, "Load printer",
                     "Load a printer configuration file.")
    self.Bind(wx.EVT_MENU, self.pgPrinter.onLoadConfig, id = ID_LOAD_PRINTER)

    file_menu.Append(ID_SAVE_PRINTER, "Save printer",
                     "Save printer configuration.")
    self.Bind(wx.EVT_MENU, self.pgPrinter.onSaveConfig, id = ID_SAVE_PRINTER)
    file_menu.Enable(ID_SAVE_PRINTER, False)

    file_menu.Append(ID_SAVE_PRINTER_AS, "Save printer as...",
                     "Save printer configuration to a new file.")
    self.Bind(wx.EVT_MENU, self.pgPrinter.onSaveConfigAs,
              id = ID_SAVE_PRINTER_AS)
    file_menu.Enable(ID_SAVE_PRINTER_AS, False)

    file_menu.AppendSeparator()

    file_menu.Append(ID_LOAD_BOARD, "Load board",
                     "Load a board configuration file.")
    self.Bind(wx.EVT_MENU, self.pgBoard.onLoadConfig, id = ID_LOAD_BOARD)

    file_menu.Append(ID_SAVE_BOARD, "Save board", "Save board configuration.")
    self.Bind(wx.EVT_MENU, self.pgBoard.onSaveConfig, id = ID_SAVE_BOARD)
    file_menu.Enable(ID_SAVE_BOARD, False)

    file_menu.Append(ID_SAVE_BOARD_AS, "Save board as...",
                     "Save board configuration to a new file.")
    self.Bind(wx.EVT_MENU, self.pgBoard.onSaveConfigAs, id = ID_SAVE_BOARD_AS)
    file_menu.Enable(ID_SAVE_BOARD_AS, False)

    file_menu.AppendSeparator()

    file_menu.Append(wx.ID_EXIT, "E&xit", "Exit the application.")
    self.Bind(wx.EVT_MENU, self.onClose, id = wx.ID_EXIT)

    self.fileMenu = file_menu

    menu_bar = wx.MenuBar()

    menu_bar.Append(file_menu, "&File")

    self.SetMenuBar(menu_bar)

  def enableSavePrinter(self, flag):
    self.fileMenu.Enable(ID_SAVE_PRINTER, flag)
    self.fileMenu.Enable(ID_SAVE_PRINTER_AS, flag)

  def enableSaveBoard(self, flag):
    self.fileMenu.Enable(ID_SAVE_BOARD, flag)
    self.fileMenu.Enable(ID_SAVE_BOARD_AS, flag)


if __name__ == '__main__':
  app = wx.PySimpleApp()
  frame = ConfigFrame()
  frame.Show(True)
  app.MainLoop()
