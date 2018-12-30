from __future__ import print_function

import sys
import time

try:
    import wx
    if int(wx.__version__[0]) < 4:
        print("Configtool needs wxPython 4.x. Please check https://wxpython.org/pages/downloads/.")
        time.sleep(10)
        sys.exit(-1)
except:
    print(
        "ImportError: No module named wx\n\n"
        "wxPython is not installed. This program requires wxPython to run.\n"
        "See your package manager and/or https://wxpython.org/pages/downloads/."
    )
    time.sleep(10)
    sys.exit(-1)

import os.path

from configtool.data import reHelpText
from configtool.decoration import Decoration
from configtool.settings import Settings
from configtool.settingsdlg import SettingsDlg
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
ID_HELP = 1050
ID_REPORT = 1051
ID_ABOUT = 1052


class ConfigFrame(wx.Frame):
    def __init__(self, settings):
        wx.Frame.__init__(self, None, -1, "Teacup Configtool", size=(880, 550))
        self.Bind(wx.EVT_CLOSE, self.onClose)
        self.Bind(wx.EVT_SIZE, self.onResize)

        self.deco = Decoration()

        panel = wx.Panel(self, -1)
        panel.SetBackgroundColour(self.deco.getBackgroundColour())
        panel.Bind(wx.EVT_PAINT, self.deco.onPaintBackground)

        self.settings = settings
        self.settings.app = self
        self.settings.font = wx.Font(
            8, wx.FONTFAMILY_SWISS, wx.FONTSTYLE_NORMAL, wx.FONTWEIGHT_BOLD
        )

        self.heaters = []
        self.savePrtEna = False
        self.saveBrdEna = False
        self.protPrtFile = False
        self.protBrdFile = False

        sz = wx.BoxSizer(wx.HORIZONTAL)

        self.nb = wx.Notebook(panel, wx.ID_ANY, size=(880, 550), style=wx.BK_DEFAULT)
        self.nb.SetBackgroundColour(self.deco.getBackgroundColour())
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
        self.panel = panel

        sz.Add(self.nb, 1, wx.EXPAND + wx.ALL, 5)
        self.SetSizer(sz)
        self.makeMenu()

    def onClose(self, evt):
        if not self.pgPrinter.confirmLoseChanges("exit"):
            return

        if not self.pgBoard.confirmLoseChanges("exit"):
            return

        self.Destroy()

    def onResize(self, evt):
        self.panel.SetSize(self.GetClientSize())
        self.Refresh()
        evt.Skip()

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

        file_menu.Append(
            ID_LOAD_CONFIG,
            "Load config.h",
            "Load config.h and its named printer and board files.",
        )
        self.Bind(wx.EVT_MENU, self.onLoadConfig, id=ID_LOAD_CONFIG)
        file_menu.Enable(ID_LOAD_CONFIG, False)

        file_menu.Append(ID_SAVE_CONFIG, "Save config.h", "Save config.h file.")
        self.Bind(wx.EVT_MENU, self.onSaveConfig, id=ID_SAVE_CONFIG)
        file_menu.Enable(ID_SAVE_CONFIG, False)

        file_menu.AppendSeparator()

        file_menu.Append(
            ID_LOAD_PRINTER, "Load printer", "Load a printer configuration file."
        )
        self.Bind(wx.EVT_MENU, self.pgPrinter.onLoadConfig, id=ID_LOAD_PRINTER)

        file_menu.Append(ID_SAVE_PRINTER, "Save printer", "Save printer configuration.")
        self.Bind(wx.EVT_MENU, self.onSavePrinterConfig, id=ID_SAVE_PRINTER)
        file_menu.Enable(ID_SAVE_PRINTER, False)

        file_menu.Append(
            ID_SAVE_PRINTER_AS,
            "Save printer as...",
            "Save printer configuration to a new file.",
        )
        self.Bind(wx.EVT_MENU, self.onSavePrinterConfigAs, id=ID_SAVE_PRINTER_AS)
        file_menu.Enable(ID_SAVE_PRINTER_AS, False)

        file_menu.AppendSeparator()

        file_menu.Append(
            ID_LOAD_BOARD, "Load board", "Load a board configuration file."
        )
        self.Bind(wx.EVT_MENU, self.pgBoard.onLoadConfig, id=ID_LOAD_BOARD)

        file_menu.Append(ID_SAVE_BOARD, "Save board", "Save board configuration.")
        self.Bind(wx.EVT_MENU, self.onSaveBoardConfig, id=ID_SAVE_BOARD)
        file_menu.Enable(ID_SAVE_BOARD, False)

        file_menu.Append(
            ID_SAVE_BOARD_AS,
            "Save board as...",
            "Save board configuration to a new file.",
        )
        self.Bind(wx.EVT_MENU, self.onSaveBoardConfigAs, id=ID_SAVE_BOARD_AS)
        file_menu.Enable(ID_SAVE_BOARD_AS, False)

        file_menu.AppendSeparator()

        file_menu.Append(wx.ID_EXIT, "E&xit", "Exit the application.")
        self.Bind(wx.EVT_MENU, self.onClose, id=wx.ID_EXIT)

        self.fileMenu = file_menu

        menu_bar = wx.MenuBar()

        menu_bar.Append(file_menu, "&File")

        edit_menu = wx.Menu()

        edit_menu.Append(ID_SETTINGS, "Settings", "Change settings.")
        self.Bind(wx.EVT_MENU, self.onEditSettings, id=ID_SETTINGS)

        self.editMenu = edit_menu

        menu_bar.Append(edit_menu, "&Edit")

        build_menu = wx.Menu()

        build_menu.Append(ID_BUILD, "Build", "Build the executable.")
        self.Bind(wx.EVT_MENU, self.onBuild, id=ID_BUILD)

        build_menu.Append(ID_UPLOAD, "Upload", "Upload the executable.")
        self.Bind(wx.EVT_MENU, self.onUpload, id=ID_UPLOAD)

        self.buildMenu = build_menu

        menu_bar.Append(build_menu, "&Build")

        help_menu = wx.Menu()

        help_menu.Append(ID_HELP, "Help", "Find help.")
        self.Bind(wx.EVT_MENU, self.onHelp, id=ID_HELP)

        help_menu.Append(
            ID_REPORT, "Report problem", "Report a problem to Teacup maintainers."
        )
        self.Bind(wx.EVT_MENU, self.onReportProblem, id=ID_REPORT)

        help_menu.AppendSeparator()

        help_menu.Append(ID_ABOUT, "About Teacup")
        self.Bind(wx.EVT_MENU, self.onAbout, id=ID_ABOUT)

        self.helpMenu = help_menu

        menu_bar.Append(help_menu, "&Help")

        self.SetMenuBar(menu_bar)
        loadFlag = self.checkEnableLoadConfig()
        self.checkEnableUpload()
        if loadFlag:
            self.loadConfigFile("config.h")

    def onSaveBoardConfig(self, evt):
        rc = self.pgBoard.onSaveConfig(evt)
        if rc:
            self.checkEnableLoadConfig()
        return rc

    def onSaveBoardConfigAs(self, evt):
        rc = self.pgBoard.onSaveConfigAs(evt)
        if rc:
            self.checkEnableLoadConfig()
        return rc

    def onSavePrinterConfig(self, evt):
        rc = self.pgPrinter.onSaveConfig(evt)
        if rc:
            self.checkEnableLoadConfig()
        return rc

    def onSavePrinterConfigAs(self, evt):
        rc = self.pgPrinter.onSaveConfigAs(evt)
        if rc:
            self.checkEnableLoadConfig()
        return rc

    def checkEnableLoadConfig(self):
        fn = os.path.join(self.settings.folder, "config.h")
        if os.path.isfile(fn):
            self.fileMenu.Enable(ID_LOAD_CONFIG, True)
            self.buildMenu.Enable(ID_BUILD, True)
            return True
        else:
            self.fileMenu.Enable(ID_LOAD_CONFIG, False)
            self.buildMenu.Enable(ID_BUILD, False)
            return False

    def checkEnableUpload(self):
        fn = os.path.join(self.settings.folder, "teacup.hex")
        if os.path.isfile(fn):
            self.buildMenu.Enable(ID_UPLOAD, True)
        else:
            self.buildMenu.Enable(ID_UPLOAD, False)

    def enableSavePrinter(self, saveFlag, saveAsFlag):
        self.fileMenu.Enable(ID_SAVE_PRINTER, saveFlag)
        self.fileMenu.Enable(ID_SAVE_PRINTER_AS, saveAsFlag)
        self.savePrtEna = saveAsFlag
        self.protPrtFile = not saveFlag
        if self.savePrtEna and self.saveBrdEna:
            self.enableSaveConfig(True)
        else:
            self.enableSaveConfig(False)

    def enableSaveBoard(self, saveFlag, saveAsFlag):
        self.fileMenu.Enable(ID_SAVE_BOARD, saveFlag)
        self.fileMenu.Enable(ID_SAVE_BOARD_AS, saveAsFlag)
        self.saveBrdEna = saveAsFlag
        self.protBrdFile = not saveFlag
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
            return False

        if not self.pgBoard.confirmLoseChanges("load config"):
            return False

        pfile, bfile = self.getConfigFileNames(fn)

        if not pfile:
            self.message(
                "Config file did not contain a printer file " "include statement.",
                "Config error",
            )
            return False
        else:
            if not self.pgPrinter.loadConfigFile(pfile):
                self.message(
                    "There was a problem loading the printer config file:\n%s" % pfile,
                    "Config error",
                )
                return False

        if not bfile:
            self.message(
                "Config file did not contain a board file " "include statement.",
                "Config error",
            )
            return False
        else:
            if not self.pgBoard.loadConfigFile(bfile):
                self.message(
                    "There was a problem loading the board config file:\n%s" % bfile,
                    "Config error",
                )
                return False

        return True

    def getConfigFileNames(self, fn):
        pfile = None
        bfile = None
        path = os.path.join(self.settings.folder, fn)
        try:
            cfgBuffer = list(open(path))
        except:
            self.message("Unable to process config file %s." % fn, "File error")
            return None, None

        for ln in cfgBuffer:
            if not ln.lstrip().startswith("#include"):
                continue

            m = reInclude.search(ln)
            if m:
                t = m.groups()
                if len(t) == 1:
                    if "printer." in t[0]:
                        if pfile:
                            self.message(
                                "Multiple printer file include statements.\n"
                                "Ignoring %s." % ln,
                                "Config error",
                                wx.OK + wx.ICON_WARNING,
                            )
                        else:
                            pfile = os.path.join(self.settings.folder, t[0])
                    elif "board." in t[0]:
                        if bfile:
                            self.message(
                                "Multiple board file include statements.\n"
                                "Ignoring %s." % ln,
                                "Config error",
                                wx.OK + wx.ICON_WARNING,
                            )
                        else:
                            bfile = os.path.join(self.settings.folder, t[0])
                    else:
                        self.message(
                            "Unable to parse include statement:\n%s" % ln,
                            "Config error",
                        )

        return pfile, bfile

    def onSaveConfig(self, evt):
        fn = os.path.join(self.settings.folder, "config.h")
        try:
            fp = open(fn, "w")
        except:
            self.message("Unable to open config.h for output.", "File error")
            return False

        bfn = self.pgBoard.getFileName()
        if self.pgBoard.isModified() and self.pgBoard.isValid():
            if not self.pgBoard.saveConfigFile(bfn):
                return False
        else:
            self.pgBoard.generateTempTables()

        pfn = self.pgPrinter.getFileName()
        if self.pgPrinter.isModified() and self.pgPrinter.isValid():
            if not self.pgPrinter.saveConfigFile(pfn):
                return False

        prefix = self.settings.folder + os.path.sep
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
        fp.write('#include "%s"\n' % rbfn)
        fp.write("\n")
        fp.write("// Configuration for printer board.\n")
        fp.write('#include "%s"\n' % rpfn)

        fp.close()

        self.checkEnableLoadConfig()
        return True

    def onBuild(self, evt):
        self.onBuildorUpload(True)

    def onUpload(self, evt):
        self.onBuildorUpload(False)

    def onBuildorUpload(self, buildFlag):
        if not (self.pgPrinter.hasData() or self.pgBoard.hasData()):
            dlg = wx.MessageDialog(
                self,
                "Data needs to be loaded. " "Click Yes to load config.h.",
                "Data missing",
                wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION,
            )
            rc = dlg.ShowModal()
            dlg.Destroy()
            if rc != wx.ID_YES:
                return

            self.loadConfigFile("config.h")
        else:
            if self.pgPrinter.isModified():
                dlg = wx.MessageDialog(
                    self,
                    "Printer data needs to be saved. Click "
                    "Yes to save printer configuration.",
                    "Changes pending",
                    wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION,
                )
                rc = dlg.ShowModal()
                dlg.Destroy()
                if rc != wx.ID_YES:
                    return

                if self.protPrtFile:
                    rc = self.onSavePrinterConfigAs(None)
                else:
                    rc = self.onSavePrinterConfig(None)
                if not rc:
                    return

            if self.pgBoard.isModified():
                dlg = wx.MessageDialog(
                    self,
                    "Board data needs to be saved. Click "
                    "Yes to save board configuration.",
                    "Changes pending",
                    wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION,
                )
                rc = dlg.ShowModal()
                dlg.Destroy()
                if rc != wx.ID_YES:
                    return

                if self.protBrdFile:
                    rc = self.onSaveBoardConfigAs(None)
                else:
                    rc = self.onSaveBoardConfig(None)
                if not rc:
                    return

        if not self.verifyConfigLoaded():
            dlg = wx.MessageDialog(
                self,
                "Loaded configuration does not match the "
                "config.h file. Click Yes to save config.h.",
                "Configuration changed",
                wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION,
            )
            rc = dlg.ShowModal()
            dlg.Destroy()
            if rc != wx.ID_YES:
                return

            if not self.onSaveConfig(None):
                return

        f_cpu, cpu = self.pgBoard.getCPUInfo()
        if not cpu:
            dlg = wx.MessageDialog(
                self,
                "Unable to determine CPU type.",
                "CPU type error",
                wx.OK | wx.ICON_ERROR,
            )
            dlg.ShowModal()
            dlg.Destroy()
            return

        if not f_cpu:
            dlg = wx.MessageDialog(
                self,
                "Unable to determine CPU clock rate.",
                "CPU clock rate error",
                wx.OK | wx.ICON_ERROR,
            )
            dlg.ShowModal()
            dlg.Destroy()
            return

        if buildFlag:
            dlg = Build(self, self.settings, f_cpu, cpu)
            dlg.ShowModal()
            dlg.Destroy()
            self.checkEnableUpload()
        else:
            dlg = Upload(self, self.settings, f_cpu, cpu)
            dlg.ShowModal()
            dlg.Destroy()

    def verifyConfigLoaded(self):
        pfile, bfile = self.getConfigFileNames("config.h")
        lpfile = self.pgPrinter.getFileName()
        lbfile = self.pgBoard.getFileName()

        return (pfile == lpfile) and (bfile == lbfile)

    def onEditSettings(self, evt):
        dlg = SettingsDlg(self, self.settings)
        rc = dlg.ShowModal()
        dlg.Destroy()

    def onHelp(self, evt):
        self.message(
            "Find help by hovering slowly over the buttons and text "
            "fields. Tooltip should appear, explaining things.",
            "Find help",
            style=wx.OK,
        )

    def onReportProblem(self, evt):
        import urllib
        import webbrowser
        import subprocess
        from sys import platform

        # Testing allowed URLs up to 32 kB in size. Longer URLs are simply chopped.
        mailRecipients = (
            "reply+0004dc756da9f0641af0a3834c580ad5be469f4f6b"
            "5d4cfc92cf00000001118c958a92a169ce051faa8c@"
            "reply.github.com,mah@jump-ing.de"
        )
        mailSubject = "Teacup problem report"
        mailBody = (
            'Please answer these questions before hitting "send":\n\n'
            "What did you try to do?\n\n\n"
            "What did you expect to happen?\n\n\n"
            "What happened instead?\n\n\n\n"
            "To allow developers to help, configuration files are "
            "attached, with help comments stripped:\n"
        )

        for f in self.pgBoard.getFileName(), self.pgPrinter.getFileName():
            if not f:
                mailBody += "\n(no file loaded)\n"
                continue

            mailBody += "\n" + os.path.basename(f) + ":\n"
            mailBody += "----------------------------------------------\n"
            try:
                fc = open(f).read()
                fc = reHelpText.sub("", fc)
                mailBody += fc
            except:
                mailBody += "(could not read this file)\n"
            mailBody += "----------------------------------------------\n"

            url = (
                "mailto:"
                + urllib.quote(mailRecipients)
                + "?subject="
                + urllib.quote(mailSubject)
                + "&body="
                + urllib.quote(mailBody)
            )

        # This is a work around a bug in gvfs-open coming with (at least) Ubuntu
        # 15.04. gvfs-open would open mailto:///user@example.com instead of
        # the requested mailto:user@example.com.
        if platform.startswith("linux"):
            try:
                subprocess.check_output(["gvfs-open", "--help"])

                # Broken gvfs-open exists, so it might be used.
                # Try to open the URL directly.
                for urlOpener in (
                    "thunderbird",
                    "evolution",
                    "firefox",
                    "mozilla",
                    "epiphany",
                    "konqueror",
                    "chromium-browser",
                    "google-chrome",
                ):
                    try:
                        subprocess.check_output(
                            [urlOpener, url], stderr=subprocess.STDOUT
                        )
                        return
                    except:
                        pass
            except:
                pass

        webbrowser.open_new(url)

    def onAbout(self, evt):
        # Get the contributors' top 10 with something like this:
        #   git shortlog experimental -sne | perl -ne \
        #      '/([0-9]+)\s*(.*)\s*(\<.*)/g && \
        #       printf  "\n%12s\"    %s (%d commits)\\n\"", "", $2, $1;
        #
        # Most recent commiters are in the .mailmap
        self.message(
            "Teacup Firmware is a 3D Printer and CNC machine controlling "
            "firmware with emphasis on performance, efficiency and "
            "outstanding quality. What Teacup does, shall it do very well."
            "\n\n\n"
            "Lots of people hard at work! Top 10 contributors:\n\n"
            "    Markus Hitter  (870 commits)\n"
            "    Michael Moon  (325 commits)\n"
            "    Nico Tonnhofer  (164 commits)\n"
            "    Phil Hord  (117 commits)\n"
            "    Jeff Bernardis  (55 commits)\n"
            "    Markus Amsler  (48 commits)\n"
            "    David Forrest  (27 commits)\n"
            "    Jim McGee  (15 commits)\n"
            "    Ben Jackson  (12 commits)\n"
            "    Robert Konklewski  (12 commits)\n"
            "About Teacup",
            style=wx.OK,
        )

    def message(self, text, title, style=wx.OK + wx.ICON_ERROR):
        dlg = wx.MessageDialog(self, text, title, style)
        dlg.ShowModal()
        dlg.Destroy()


def StartGui(settings):
    app = wx.App(False)
    frame = ConfigFrame(settings)
    frame.Show(True)
    app.MainLoop()
