#!/usr/bin/env python

import wx
import os.path
import inspect

cmd_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile(
                              inspect.currentframe()))[0]))

from configtool.printerpanel import PrinterPanel
from configtool.boardpanel import BoardPanel
from configtool.data import VERSION, reInclude

ID_LOAD_PRINTER = 1000
ID_SAVE_PRINTER = 1001
ID_SAVE_PRINTER_AS = 1002
ID_LOAD_BOARD = 1010
ID_SAVE_BOARD = 1011
ID_SAVE_BOARD_AS = 1012
ID_LOAD_CONFIG = 1020
ID_LOAD_DEFAULT = 1021
ID_SAVE_CONFIG = 1022


class ConfigFrame(wx.Frame):
  def __init__(self):
    wx.Frame.__init__(self, None, -1,
                      "Teacup Firmware Configurator - " + VERSION,
                      size = (880, 550))
    self.Bind(wx.EVT_CLOSE, self.onClose)

    self.font = wx.Font(8, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL,
                        wx.FONTWEIGHT_BOLD)

    panel = wx.Panel(self, -1)

    self.heaters = []
    self.savePrtEna = False
    self.saveBrdEna = False

    sz = wx.BoxSizer(wx.HORIZONTAL)

    self.nb = wx.Notebook(panel, wx.ID_ANY, size = (880, 550),
                          style = wx.BK_DEFAULT)
    self.nb.SetFont(self.font)

    self.pgPrinter = PrinterPanel(self, self.nb, self.font, cmd_folder)
    self.nb.AddPage(self.pgPrinter, "Printer")

    self.pgBoard = BoardPanel(self, self.nb, self.font, cmd_folder)
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

    file_menu.Append(ID_LOAD_CONFIG, "Load config.h",
                     "Load config.h and its named printer and board files.")
    self.Bind(wx.EVT_MENU, self.onLoadConfig, id = ID_LOAD_CONFIG)

    file_menu.Append(ID_LOAD_DEFAULT, "Load default",
                "Load default config.h and its named printer and board files.")
    self.Bind(wx.EVT_MENU, self.onLoadDefault, id = ID_LOAD_DEFAULT)

    file_menu.Append(ID_SAVE_CONFIG, "Save config.h", "Save config.h file.")
    self.Bind(wx.EVT_MENU, self.onSaveConfig, id = ID_SAVE_CONFIG)
    file_menu.Enable(ID_SAVE_CONFIG, False)

    file_menu.AppendSeparator()

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
    self.savePrtEna = flag
    if self.savePrtEna and self.saveBrdEna:
      self.enableSaveConfig(True)
    else:
      self.enableSaveConfig(False)

  def enableSaveBoard(self, flag):
    self.fileMenu.Enable(ID_SAVE_BOARD, flag)
    self.fileMenu.Enable(ID_SAVE_BOARD_AS, flag)
    self.saveBrdEna = flag
    if self.savePrtEna and self.saveBrdEna:
      self.enableSaveConfig(True)
    else:
      self.enableSaveConfig(False)

  def enableSaveConfig(self, flag):
    self.fileMenu.Enable(ID_SAVE_CONFIG, flag)

  def onLoadConfig(self, evt):
    self.loadConfigFile("config.h")

  def onLoadDefault(self, evt):
    self.loadConfigFile("config.default.h")

  def loadConfigFile(self, fn):
    if not self.pgPrinter.confirmLoseChanges("load config"):
      return
    if not self.pgBoard.confirmLoseChanges("load config"):
      return

    pfile = None
    bfile = None
    path = os.path.join(cmd_folder, fn)
    try:
      cfgBuffer = list(open(path))
    except:
      self.message("Unable to process config file %s." % fn, "File error")
      return

    for ln in cfgBuffer:
      if not ln.lstrip().startswith("#include"):
        continue

      m = reInclude.search(ln)
      if m:
        t = m.groups()
        if len(t) == 1:
          if "printer." in t[0]:
            if pfile:
              self.message("Multiple printer file include statements.\n"
                           "Ignoring %s." % ln, "Config error",
                           wx.OK + wx.ICON_WARNING)
            else:
              pfile = os.path.join(cmd_folder, t[0])
          elif "board." in t[0]:
            if bfile:
              self.message("Multiple board file include statements.\n"
                           "Ignoring %s." % ln, "Config error",
                           wx.OK + wx.ICON_WARNING)
            else:
              bfile = os.path.join(cmd_folder, t[0])
          else:
            self.message("Unable to parse include statement:\n%s" % ln,
                         "Config error")

    if not pfile:
      self.message("Config file did not contain a printer file include "
                   "statement.", "Config error")
      return

    if not bfile:
      self.message("Config file did not contain a board file include "
                   "statement.", "Config error")
      return

    self.pgPrinter.loadConfigFile(pfile)

    self.pgBoard.loadConfigFile(bfile)

  def onSaveConfig(self, evt):
    fn = os.path.join(cmd_folder, "config.h")
    try:
      fp = open(fn, 'w')
    except:
      self.message("Unable to open config.h for output.", "File error")
      return

    bfn = self.pgBoard.getFileName()
    if not self.pgBoard.saveConfigFile(bfn):
      self.message("Unable to save board configuration %s." %
                   os.path.basename(bfn), "File error")
      return

    pfn = self.pgPrinter.getFileName()
    if not self.pgPrinter.saveConfigFile(pfn):
      self.message("Unable to save printer configuration %s." %
                   os.path.basename(pfn), "File error")
      return

    prefix = cmd_folder + os.path.sep
    lpfx = len(prefix)

    if bfn.startswith(prefix):
      rbfn = bfn[lpfx:]
    else:
      rbfn = bfn

    if pfn.startswith(prefix):
      rpfn = pfn[lpfx:]
    else:
      rpfn = pfn

    fp.write("\n")
    fp.write("// Configuration for controller board.\n")
    fp.write("#include \"%s\"\n" % rbfn)
    fp.write("\n")
    fp.write("// Configuration for printer board.\n")
    fp.write("#include \"%s\"\n" % rpfn)

    fp.close()

    m = ("%s successfully saved.\n"
         "%s successfully saved.\nconfig.h successfully saved.") % (rbfn, rpfn)
    self.message(m, "Save configuration success", wx.OK + wx.ICON_INFORMATION)

  def message(self, text, title, style = wx.OK + wx.ICON_ERROR):
    dlg = wx.MessageDialog(self, text, title, style)
    dlg.ShowModal()
    dlg.Destroy()


if __name__ == '__main__':
  app = wx.PySimpleApp()
  frame = ConfigFrame()
  frame.Show(True)
  app.MainLoop()
