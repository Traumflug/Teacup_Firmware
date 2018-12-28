import os
import wx
import re

from sys import platform
from configtool.decoration import Decoration
from configtool.data import (
    defineValueFormat,
    defineBoolFormat,
    defineHeaterFormat,
    reHelpTextStart,
    reHelpTextEnd,
    reStartSensors,
    reEndSensors,
    reStartHeaters,
    reEndHeaters,
    reCandHeatPins,
    reCandThermPins,
    reCandProcessors,
    reCandCPUClocks,
    reFloatAttr,
    reDefine,
    reDefineBL,
    reDefQS,
    reDefQSm,
    reDefQSm2,
    reDefBool,
    reDefBoolBL,
    reDefHT,
    reDefTS,
    reDefTT,
    reSensor,
    reHeater3,
    reHeater4,
    reTempTable4,
    reTempTable7,
)
from configtool.pinoutspage import PinoutsPage
from configtool.displaypage import DisplayPage
from configtool.sensorpage import SensorsPage
from configtool.heaterspage import HeatersPage
from configtool.communicationspage import CommunicationsPage
from configtool.cpupage import CpuPage
from configtool.protectedfiles import protectedFiles
from configtool.thermistortablefile import generateTempTables

from configtool.board import Board


class BoardPanel(wx.Panel):
    def __init__(self, parent, nb, settings):
        wx.Panel.__init__(self, nb, wx.ID_ANY)
        self.parent = parent

        self.deco = Decoration()
        self.protFileLoaded = False

        self.settings = settings

        self.board = Board(self.settings)

        self.dir = os.path.join(self.settings.folder, "config")

        self.SetBackgroundColour(self.deco.getBackgroundColour())
        self.Bind(wx.EVT_PAINT, self.deco.onPaintBackground)
        sz = wx.BoxSizer(wx.HORIZONTAL)

        self.nb = wx.Notebook(self, wx.ID_ANY, size=(21, 21), style=wx.BK_DEFAULT)
        self.nb.SetBackgroundColour(self.deco.getBackgroundColour())
        self.nb.SetFont(self.settings.font)

        self.pages = []
        self.titles = []
        self.pageModified = []
        self.pageValid = []

        self.pgCpu = self.registerPage(CpuPage, "CPU")
        self.pgPins = self.registerPage(PinoutsPage, "Pinouts")
        self.pgDisplay = self.registerPage(DisplayPage, "Display")
        self.pgHeaters = self.registerPage(HeatersPage, "Heaters")
        self.pgSensors = self.registerPage(
            SensorsPage, "Temperature Sensors", heatersPage=self.pgHeaters
        )
        self.pgCommunications = self.registerPage(CommunicationsPage, "Communications")

        sz.Add(self.nb, 1, wx.EXPAND + wx.ALL, 5)

        self.SetSizer(sz)
        self.Fit()

    def registerPage(self, klass, label, *args, **kwargs):
        page = klass(
            self, self.nb, len(self.pages), *args, font=self.settings.font, **kwargs
        )
        self.nb.AddPage(page, label)
        self.pages.append(page)
        self.titles.append(label)
        self.pageModified.append(False)
        self.pageValid.append(True)
        return page

    def getCPUInfo(self):
        return self.board.getCPUInfo()

    def assertModified(self, pg, flag=True):
        self.pageModified[pg] = flag
        self.modifyTab(pg)

    def isModified(self):
        return True in self.pageModified

    def isValid(self):
        return not (False in self.pageValid)

    def hasData(self):
        return self.board.hasData()

    def getFileName(self):
        return self.board.getFileName()

    def assertValid(self, pg, flag=True):
        self.pageValid[pg] = flag
        self.modifyTab(pg)

        if False in self.pageValid:
            self.parent.enableSaveBoard(False, False)
        else:
            self.parent.enableSaveBoard(not self.protFileLoaded, True)

    def modifyTab(self, pg):
        if self.pageModified[pg] and not self.pageValid[pg]:
            pfx = "?* "
        elif self.pageModified[pg]:
            pfx = "* "
        elif not self.pageValid[pg]:
            pfx = "? "
        else:
            pfx = ""

        self.nb.SetPageText(pg, pfx + self.titles[pg])
        if True in self.pageModified and False in self.pageValid:
            pfx = "?* "
        elif True in self.pageModified:
            pfx = "* "
        elif False in self.pageValid:
            pfx = "? "
        else:
            pfx = ""
        self.parent.setBoardTabDecor(pfx)

    def setHeaters(self, ht):
        self.parent.setHeaters(ht)

    def onClose(self, evt):
        if not self.confirmLoseChanges("exit"):
            return

        self.Destroy()

    def confirmLoseChanges(self, msg):
        if True not in self.pageModified:
            return True

        dlg = wx.MessageDialog(
            self,
            "Are you sure you want to " + msg + "?\n"
            "There are changes to your board "
            "configuration that will be lost.",
            "Changes pending",
            wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION,
        )
        rc = dlg.ShowModal()
        dlg.Destroy()

        if rc != wx.ID_YES:
            return False

        return True

    def onLoadConfig(self, evt):
        if not self.confirmLoseChanges("load a new board configuration"):
            return

        if platform.startswith("darwin"):
            # Mac OS X appears to be a bit limited on wildcards.
            wildcard = "Board configuration (board.*.h)|*.h"
        else:
            wildcard = "Board configuration (board.*.h)|board.*.h"

        dlg = wx.FileDialog(
            self,
            message="Choose a board config file",
            defaultDir=self.dir,
            defaultFile="",
            wildcard=wildcard,
            style=wx.FD_OPEN | wx.FD_CHANGE_DIR,
        )

        path = None
        if dlg.ShowModal() == wx.ID_OK:
            path = dlg.GetPath()

        dlg.Destroy()
        if path is None:
            return

        self.dir = os.path.dirname(path)
        rc, efn = self.loadConfigFile(path)

        if not rc:
            dlg = wx.MessageDialog(
                self,
                "Unable to process file %s." % efn,
                "File error",
                wx.OK + wx.ICON_ERROR,
            )
            dlg.ShowModal()
            dlg.Destroy()
            return

    def loadConfigFile(self, fn):
        ok, file = self.board.loadConfigFile(fn)
        if not ok:
            return ok, file

        if os.path.basename(fn) in protectedFiles:
            self.parent.enableSaveBoard(False, True)
            self.protFileLoaded = True
        else:
            self.protFileLoaded = False
            self.parent.enableSaveBoard(True, True)

        self.parent.setBoardTabFile(os.path.basename(fn))
        self.pgHeaters.setCandidatePins(self.board.candHeatPins)
        self.pgSensors.setCandidatePins(self.board.candThermPins)
        self.pgCpu.setCandidateProcessors(self.board.candProcessors)
        self.pgCpu.setCandidateClocks(self.board.candClocks)

        for pg in self.pages:
            pg.insertValues(self.board.cfgValues)
            pg.setHelpText(self.board.helpText)

        self.pgSensors.setSensors(self.board.sensors)
        self.pgHeaters.setHeaters(self.board.heaters)

        return True, None

    def onSaveConfig(self, evt):
        path = self.getFileName()
        return self.saveConfigFile(path)

    def onSaveConfigAs(self, evt):
        if platform.startswith("darwin"):
            # Mac OS X appears to be a bit limited on wildcards.
            wildcard = "Board configuration (board.*.h)|*.h"
        else:
            wildcard = "Board configuration (board.*.h)|board.*.h"

        dlg = wx.FileDialog(
            self,
            message="Save as ...",
            defaultDir=self.dir,
            defaultFile="",
            wildcard=wildcard,
            style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT,
        )

        val = dlg.ShowModal()

        if val != wx.ID_OK:
            dlg.Destroy()
            return

        path = dlg.GetPath()
        dlg.Destroy()

        rc = self.saveConfigFile(path)
        if rc:
            self.parent.setBoardTabFile(os.path.basename(path))
            self.protFileLoaded = False
            self.parent.enableSaveBoard(True, True)
        return rc

    def saveConfigFile(self, path):
        if os.path.basename(path) in protectedFiles:
            dlg = wx.MessageDialog(
                self,
                "It's not allowed to overwrite files "
                "distributed by Teacup. Choose another name.",
                "Protected file error",
                wx.OK + wx.ICON_ERROR,
            )
            dlg.ShowModal()
            dlg.Destroy()
            return False

        if not os.path.basename(path).startswith("board."):
            dlg = wx.MessageDialog(
                self,
                "Illegal file name: %s.\n" 'File name must begin with "board."' % path,
                "Illegal file name",
                wx.OK + wx.ICON_ERROR,
            )
            dlg.ShowModal()
            dlg.Destroy()
            return False

        values = {}
        for pg in self.pages:
            v1 = pg.getValues()
            for k in v1.keys():
                values[k] = v1[k]

        ext = os.path.splitext(os.path.basename(path))[1]
        self.dir = os.path.dirname(path)

        if ext == "":
            path += ".h"

        try:
            self.board.saveConfigFile(path, values)
        except:
            dlg = wx.MessageDialog(
                self,
                "Unable to write to file %s." % path,
                "File error",
                wx.OK + wx.ICON_ERROR,
            )
            dlg.ShowModal()
            dlg.Destroy()
            return False

        return self.generateTempTables()

    def generateTempTables(self):
        if not generateTempTables(self.board.sensors, self.settings):
            dlg = wx.MessageDialog(
                self,
                "Error writing to file thermistortable.h.",
                "File error",
                wx.OK + wx.ICON_ERROR,
            )
            dlg.ShowModal()
            dlg.Destroy()
            return False

        return True
