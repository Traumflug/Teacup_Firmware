#!/usr/bin/env python

import wx
import os.path
import inspect

cmd_folder = os.path.realpath(os.path.abspath(os.path.split(inspect.getfile(
                              inspect.currentframe()))[0]))

from configtool.settings import Settings, SettingsDlg
from configtool.printerpanel import PrinterPanel
from configtool.boardpanel import BoardPanel
from configtool.build import Build, Upload
from configtool.data import reInclude

ID_LOAD_PRINTER = 1000
ID_SAVE_PRINTER = 1001
ID_SAVE_PRINTER_AS = 1002
ID_LOAD_BOARD = 1010
ID_SAVE_BOARD = 1011
ID_SAVE_BOARD_AS = 1012
ID_LOAD_CONFIG = 1020
ID_SAVE_CONFIG = 1021
ID_BUILD = 1030
ID_UPLOAD = 1031
ID_SETTINGS = 1040


class ConfigFrame(wx.Frame):
  def __init__(self):
    wx.Frame.__init__(self, None, -1, "Teacup Configtool", size = (880, 550))
    self.Bind(wx.EVT_CLOSE, self.onClose)

    panel = wx.Panel(self, -1)

    self.settings = Settings(self, cmd_folder)
    self.settings.font = wx.Font(8, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL,
                                 wx.FONTWEIGHT_BOLD)
    self.settings.folder = cmd_folder

    self.heaters = []
    self.savePrtEna = False
    self.saveBrdEna = False

    sz = wx.BoxSizer(wx.HORIZONTAL)

    self.nb = wx.Notebook(panel, wx.ID_ANY, size = (880, 550),
                          style = wx.BK_DEFAULT)
    self.nb.SetFont(self.settings.font)

    self.printerFileName = None
    self.printerTabDecor = ""
    self.printerBaseText = "Printer"
    self.pgPrinter = PrinterPanel(self, self.nb, self.settings)
    self.nb.AddPage(self.pgPrinter, self.printerBaseText)

    self.boardFileName = None
    self.boardTabDecor = ""
    self.boardBaseText = "Board"
    self.pgBoard = BoardPanel(self, self.nb, self.settings)
    self.nb.AddPage(self.pgBoard, self.boardBaseText)

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

  def setPrinterTabFile(self, fn):
    self.printerFileName = fn
    self.updatePrinterTab()

  def setPrinterTabDecor(self, prefix):
    self.printerTabDecor = prefix
    self.updatePrinterTab()

  def updatePrinterTab(self):
    txt = self.printerTabDecor + self.printerBaseText
    if self.printerFileName:
      txt += " <%s>" % self.printerFileName
    self.nb.SetPageText(0, txt)

  def setBoardTabFile(self, fn):
    self.boardFileName = fn
    self.updateBoardTab()

  def setBoardTabDecor(self, prefix):
    self.boardTabDecor = prefix
    self.updateBoardTab()

  def updateBoardTab(self):
    txt = self.boardTabDecor + self.boardBaseText
    if self.boardFileName:
      txt += " <%s>" % self.boardFileName
    self.nb.SetPageText(1, txt)

  def setHeaters(self, ht):
    self.heaters = ht
    self.pgPrinter.setHeaters(ht)

  def makeMenu(self):
    file_menu = wx.Menu()

    file_menu.Append(ID_LOAD_CONFIG, "Load config.h",
                     "Load config.h and its named printer and board files.")
    self.Bind(wx.EVT_MENU, self.onLoadConfig, id = ID_LOAD_CONFIG)
    file_menu.Enable(ID_LOAD_CONFIG, False)

    file_menu.Append(ID_SAVE_CONFIG, "Save config.h", "Save config.h file.")
    self.Bind(wx.EVT_MENU, self.onSaveConfig, id = ID_SAVE_CONFIG)
    file_menu.Enable(ID_SAVE_CONFIG, False)

    file_menu.AppendSeparator()

    file_menu.Append(ID_LOAD_PRINTER, "Load printer",
                     "Load a printer configuration file.")
    self.Bind(wx.EVT_MENU, self.pgPrinter.onLoadConfig, id = ID_LOAD_PRINTER)

    file_menu.Append(ID_SAVE_PRINTER, "Save printer",
                     "Save printer configuration.")
    self.Bind(wx.EVT_MENU, self.onSavePrinterConfig, id = ID_SAVE_PRINTER)
    file_menu.Enable(ID_SAVE_PRINTER, False)

    file_menu.Append(ID_SAVE_PRINTER_AS, "Save printer as...",
                     "Save printer configuration to a new file.")
    self.Bind(wx.EVT_MENU, self.onSavePrinterConfigAs, id = ID_SAVE_PRINTER_AS)
    file_menu.Enable(ID_SAVE_PRINTER_AS, False)

    file_menu.AppendSeparator()

    file_menu.Append(ID_LOAD_BOARD, "Load board",
                     "Load a board configuration file.")
    self.Bind(wx.EVT_MENU, self.pgBoard.onLoadConfig, id = ID_LOAD_BOARD)

    file_menu.Append(ID_SAVE_BOARD, "Save board", "Save board configuration.")
    self.Bind(wx.EVT_MENU, self.onSaveBoardConfig, id = ID_SAVE_BOARD)
    file_menu.Enable(ID_SAVE_BOARD, False)

    file_menu.Append(ID_SAVE_BOARD_AS, "Save board as...",
                     "Save board configuration to a new file.")
    self.Bind(wx.EVT_MENU, self.onSaveBoardConfigAs, id = ID_SAVE_BOARD_AS)
    file_menu.Enable(ID_SAVE_BOARD_AS, False)

    file_menu.AppendSeparator()

    file_menu.Append(wx.ID_EXIT, "E&xit", "Exit the application.")
    self.Bind(wx.EVT_MENU, self.onClose, id = wx.ID_EXIT)

    self.fileMenu = file_menu

    menu_bar = wx.MenuBar()

    menu_bar.Append(file_menu, "&File")

    edit_menu = wx.Menu()

    edit_menu.Append(ID_SETTINGS, "Settings", "Change settings.")
    self.Bind(wx.EVT_MENU, self.onEditSettings, id = ID_SETTINGS)

    self.editMenu = edit_menu

    menu_bar.Append(edit_menu, "&Edit")

    build_menu = wx.Menu()

    build_menu.Append(ID_BUILD, "Build", "Build the executable.")
    self.Bind(wx.EVT_MENU, self.onBuild, id = ID_BUILD)

    build_menu.Append(ID_UPLOAD, "Upload", "Upload the executable.")
    self.Bind(wx.EVT_MENU, self.onUpload, id = ID_UPLOAD)

    self.buildMenu = build_menu

    menu_bar.Append(build_menu, "&Build")

    self.SetMenuBar(menu_bar)
    self.checkEnableLoadConfig()
    self.checkEnableUpload()

  def onSaveBoardConfig(self, evt):
    self.pgBoard.onSaveConfig(evt)
    self.checkEnableLoadConfig()

  def onSaveBoardConfigAs(self, evt):
    self.pgBoard.onSaveConfigAs(evt)
    self.checkEnableLoadConfig()

  def onSavePrinterConfig(self, evt):
    self.pgPrinter.onSaveConfig(evt)
    self.checkEnableLoadConfig()

  def onSavePrinterConfigAs(self, evt):
    self.pgPrinter.onSaveConfigAs(evt)
    self.checkEnableLoadConfig()

  def checkEnableLoadConfig(self):
    fn = os.path.join(cmd_folder, "config.h")
    if os.path.isfile(fn):
      self.fileMenu.Enable(ID_LOAD_CONFIG, True)
      self.buildMenu.Enable(ID_BUILD, True)
    else:
      self.fileMenu.Enable(ID_LOAD_CONFIG, False)
      self.buildMenu.Enable(ID_BUILD, False)

  def checkEnableUpload(self):
    fn = os.path.join(cmd_folder, "teacup.hex")
    if os.path.isfile(fn):
      self.buildMenu.Enable(ID_UPLOAD, True)
    else:
      self.buildMenu.Enable(ID_UPLOAD, False)

  def enableSavePrinter(self, saveFlag, saveAsFlag):
    self.fileMenu.Enable(ID_SAVE_PRINTER, saveFlag)
    self.fileMenu.Enable(ID_SAVE_PRINTER_AS, saveAsFlag)
    self.savePrtEna = saveAsFlag
    if self.savePrtEna and self.saveBrdEna:
      self.enableSaveConfig(True)
    else:
      self.enableSaveConfig(False)

  def enableSaveBoard(self, saveFlag, saveAsFlag):
    self.fileMenu.Enable(ID_SAVE_BOARD, saveFlag)
    self.fileMenu.Enable(ID_SAVE_BOARD_AS, saveAsFlag)
    self.saveBrdEna = saveAsFlag
    if self.savePrtEna and self.saveBrdEna:
      self.enableSaveConfig(True)
    else:
      self.enableSaveConfig(False)

  def enableSaveConfig(self, flag):
    self.fileMenu.Enable(ID_SAVE_CONFIG, flag)

  def onLoadConfig(self, evt):
    self.loadConfigFile("config.h")

  def loadConfigFile(self, fn):
    if not self.pgPrinter.confirmLoseChanges("load config"):
      return

    if not self.pgBoard.confirmLoseChanges("load config"):
      return

    pfile, bfile = self.getConfigFileNames(fn)

    if not pfile:
      self.message("Config file did not contain a printer file "
                   "include statement.", "Config error")
    else:
      if not self.pgPrinter.loadConfigFile(pfile):
        self.message("There was a problem loading the printer config file:\n%s"
                     % pfile, "Config error")

    if not bfile:
      self.message("Config file did not contain a board file "
                   "include statement.", "Config error")
    else:
      if not self.pgBoard.loadConfigFile(bfile):
        self.message("There was a problem loading the board config file:\n%s"
                     % bfile, "Config error")

  def getConfigFileNames(self, fn):
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

    return pfile, bfile

  def onSaveConfig(self, evt):
    fn = os.path.join(cmd_folder, "config.h")
    try:
      fp = open(fn, 'w')
    except:
      self.message("Unable to open config.h for output.", "File error")
      return

    bfn = self.pgBoard.getFileName()
    if self.pgBoard.isModified() and self.pgBoard.isValid():
      if not self.pgBoard.saveConfigFile(bfn):
        return

    pfn = self.pgPrinter.getFileName()
    if self.pgPrinter.isModified() and self.pgPrinter.isValid():
      if not self.pgPrinter.saveConfigFile(pfn):
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

    self.message("config.h successfully saved.", "Save configuration success",
                 wx.OK + wx.ICON_INFORMATION)

    self.checkEnableLoadConfig()

  def onBuild(self, evt):
    self.onBuildorUpload(True)

  def onUpload(self, evt):
    self.onBuildorUpload(False)

  def onBuildorUpload(self, buildFlag):
    if not (self.pgPrinter.hasData() or self.pgBoard.hasData()):
      dlg = wx.MessageDialog(self, "Data needs to be loaded. "
                                   "Click Yes to load config.h.",
                             "Data missing",
                             wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
      rc = dlg.ShowModal()
      dlg.Destroy()
      if rc != wx.ID_YES:
        return

      self.loadConfigFile("config.h")
    else:
      if self.pgPrinter.isModified():
        dlg = wx.MessageDialog(self, "Printer data needs to be saved. Click "
                                     "Yes to save printer configuration.",
                               "Changes pending",
                               wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
        rc = dlg.ShowModal()
        dlg.Destroy()
        if rc != wx.ID_YES:
          return

        self.onSavePrinterConfig(None)

      if self.pgBoard.isModified():
        dlg = wx.MessageDialog(self, "Board data needs to be saved. Click "
                                     "Yes to save board configuration.",
                               "Changes pending",
                               wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
        rc = dlg.ShowModal()
        dlg.Destroy()
        if rc != wx.ID_YES:
          return

        self.onSaveBoardConfig(None)

    if not self.verifyConfigLoaded():
      dlg = wx.MessageDialog(self, "Loaded configuration does not match the "
                                   "config.h file. Click Yes to save config.h.",
                             "Configuration changed",
                             wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
      rc = dlg.ShowModal()
      dlg.Destroy()
      if rc != wx.ID_YES:
        return

      self.onSaveConfig(None)

    f_cpu, cpu, baud = self.pgBoard.getCPUInfo()
    if not cpu:
      dlg = wx.MessageDialog(self, "Unable to determine CPU type.",
                             "CPU type error", wx.OK | wx.ICON_ERROR)
      dlg.ShowModal()
      dlg.Destroy()
      return

    if not f_cpu:
      dlg = wx.MessageDialog(self, "Unable to determine CPU clock rate.",
                             "CPU clock rate error", wx.OK | wx.ICON_ERROR)
      dlg.ShowModal()
      dlg.Destroy()
      return

    if not baud:
      # TODO: It looks like serial port baud rate is confused with bootloader
      #       baud rate here. These two can be the same, but don't have to.
      #       Bootloader baud rate isn't user selectable, it's a property of
      #       the bootloader and can be changed only by overwriting the
      #       bootloader.
      dlg = wx.MessageDialog(self, "Unable to determine CPU baud rate.",
                             "CPU baud rate error", wx.OK | wx.ICON_ERROR)
      dlg.ShowModal()
      dlg.Destroy()
      return

    if buildFlag:
      # TODO: building the executable needs no baud rate.
      dlg = Build(self, self.settings, f_cpu, cpu, baud)
      dlg.ShowModal()
      dlg.Destroy()
      self.checkEnableUpload()
    else:
      dlg = Upload(self, self.settings, f_cpu, cpu, baud)
      dlg.ShowModal()
      dlg.Destroy()

  def verifyConfigLoaded(self):
    pfile, bfile = self.getConfigFileNames("config.h")
    lpfile = self.pgPrinter.getFileName()
    lbfile = self.pgBoard.getFileName()

    return ((pfile == lpfile) and (bfile == lbfile))

  def onEditSettings(self, evt):
    dlg = SettingsDlg(self, self.settings)
    rc = dlg.ShowModal()
    dlg.Destroy()

  def message(self, text, title, style = wx.OK + wx.ICON_ERROR):
    dlg = wx.MessageDialog(self, text, title, style)
    dlg.ShowModal()
    dlg.Destroy()


if __name__ == '__main__':
  app = wx.PySimpleApp()
  frame = ConfigFrame()
  frame.Show(True)
  app.MainLoop()
