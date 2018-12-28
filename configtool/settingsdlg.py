import wx
from configtool.data import BSIZESMALL, offsetTcLabel

ARDUINODIR = 0
CFLAGS = 1
LDFLAGS = 2
OBJCOPYFLAGS = 3
PROGRAMMER = 4
PROGRAMFLAGS = 5
PORT = 6
UPLOADSPEED = 7
NUMTEMPS = 8
MINADC = 9
MAXADC = 10
T0 = 11
R1 = 12


class SettingsDlg(wx.Dialog):
    def __init__(self, parent, settings):
        wx.Dialog.__init__(self, parent, wx.ID_ANY, "Modify settings", size=(500, 300))
        self.SetFont(settings.font)
        self.settings = settings

        self.modified = False

        self.Bind(wx.EVT_CLOSE, self.onExit)

        htArdDir = (
            "Path to the Arduino IDE folder. Configtool will figure the "
            "details on where to find avr-gcc and avrdude inside there."
            "\n\nIf empty, the system wide installed tools will be used."
        )
        htCFlags = (
            "Flags passed into the avr-gcc compiler. These flags can "
            "have 3 different variables embedded within them:"
            "\n\n  %F_CPU%   will be replaced by the value of the CPU "
            "Clock Rate."
            "\n\n  %CPU%     will be replaced by the value of the CPU. "
            "\n\n  %ALNAME%  is the name of the source file being "
            "compiled with the .c extension replaced by .al.\n\n"
            "Note: the flag -save-temps=obj does not appear to be a "
            'valid flag for some compiler versions. Omit the "=obj", '
            "omit the flag entirely, or simply ignore the related warnings."
        )
        htLDFlags = "Flags passed to avr-gcc to be passed on to the linker."
        htObjCopy = "Flags passed to avr-objcopy."
        htProgrammer = "The programmer type - passed to avrdude."
        htProgramFlags = "Extra flags passed to avrdude."
        htPort = (
            "The port to which the controller is connected. Typically a "
            "path starting with /dev/... on Linux or Mac OS X, or some "
            "COM... on Windows."
        )
        htSpeed = "The baud rate with which to communicate with the bootloader."
        htNumTemps = (
            "The number of entries generated for the thermistor tables. "
            "Higher numbers slightly increase temperature reading "
            "accuracy, but also cost binary size. Default is 25."
        )
        htMinAdc = "The minimum ADC value returned by the thermistor. Typically 0."
        htMaxAdc = (
            "The maximum ADC value returned by the thermistor. "
            "Typically 1023 (maximum of 10-bit ADCs)."
        )
        htT0 = "The T0 value used for thermistor table calculation. Typically 25."
        htR1 = "The R1 value used for thermistor table calculation. Typically 0."

        # This table MUST be in the same order as the constants defined at
        # the top of this file.
        self.fields = [
            ["Arduino Directory", settings.arduinodir, htArdDir],
            ["C Compiler Flags", settings.cflags, htCFlags],
            ["LD Flags", settings.ldflags, htLDFlags],
            ["Object Copy Flags", settings.objcopyflags, htObjCopy],
            ["AVR Programmer", settings.programmer, htProgrammer],
            ["AVR Upload Flags", settings.programflags, htProgramFlags],
            ["Port", settings.port, htPort],
            ["Upload Speed", settings.uploadspeed, htSpeed],
            ["Number of Temps", settings.numTemps, htNumTemps],
            ["Minimum ADC value", settings.minAdc, htMinAdc],
            ["Maximum ADC value", settings.maxAdc, htMaxAdc],
            ["T0", settings.t0, htT0],
            ["R1", settings.r1, htR1],
        ]

        self.teList = []

        hsz = wx.BoxSizer(wx.HORIZONTAL)
        hsz.Add((10, 10))

        sz = wx.BoxSizer(wx.VERTICAL)
        sz.Add((10, 10))

        labelWidth = 140
        for f in self.fields:
            lsz = wx.BoxSizer(wx.HORIZONTAL)
            t = wx.StaticText(
                self, wx.ID_ANY, f[0], size=(labelWidth, -1), style=wx.ALIGN_RIGHT
            )
            t.SetFont(settings.font)
            lsz.Add(t, 1, wx.TOP, offsetTcLabel)

            lsz.Add((8, 8))

            te = wx.TextCtrl(self, wx.ID_ANY, f[1], size=(600, -1))
            te.Bind(wx.EVT_TEXT, self.onTextCtrl)
            te.SetToolTip(f[2])
            lsz.Add(te)
            self.teList.append(te)

            sz.Add(lsz)
            sz.Add((10, 10))

        sz.Add((20, 20))

        bsz = wx.BoxSizer(wx.HORIZONTAL)
        b = wx.Button(self, wx.ID_ANY, "Save", size=BSIZESMALL)
        b.SetFont(settings.font)
        self.Bind(wx.EVT_BUTTON, self.onSave, b)
        bsz.Add(b)
        self.bSave = b
        bsz.Add((5, 5))

        b = wx.Button(self, wx.ID_ANY, "Exit", size=BSIZESMALL)
        b.SetFont(settings.font)
        self.Bind(wx.EVT_BUTTON, self.onExit, b)
        bsz.Add(b)
        self.bExit = b

        sz.Add(bsz, 1, wx.ALIGN_CENTER_HORIZONTAL)
        sz.Add((10, 10))

        hsz.Add(sz)
        hsz.Add((10, 10))

        self.SetSizer(hsz)
        self.setModified(False)

        self.Fit()

    def setModified(self, flag):
        self.modified = flag
        if flag:
            self.bSave.Enable(True)
            self.bExit.SetLabel("Cancel")
        else:
            self.bSave.Enable(False)
            self.bExit.SetLabel("Exit")

    def onTextCtrl(self, evt):
        self.setModified(True)
        evt.Skip()

    def onSave(self, evt):
        self.saveValues()
        self.EndModal(wx.ID_OK)

    def saveValues(self):
        self.settings.arduinodir = self.teList[ARDUINODIR].GetValue()
        self.settings.cflags = self.teList[CFLAGS].GetValue()
        self.settings.ldflags = self.teList[LDFLAGS].GetValue()
        self.settings.objcopyflags = self.teList[OBJCOPYFLAGS].GetValue()
        self.settings.programmer = self.teList[PROGRAMMER].GetValue()
        self.settings.programflags = self.teList[PROGRAMFLAGS].GetValue()
        self.settings.port = self.teList[PORT].GetValue()
        self.settings.uploadspeed = self.teList[UPLOADSPEED].GetValue()
        self.settings.numTemps = self.teList[NUMTEMPS].GetValue()
        self.settings.minAdc = self.teList[MINADC].GetValue()
        self.settings.maxAdc = self.teList[MAXADC].GetValue()
        self.settings.t0 = self.teList[T0].GetValue()
        self.settings.r1 = self.teList[R1].GetValue()

        self.settings.saveSettings()

    def onExit(self, evt):
        if not self.confirmLoseChanges("exit"):
            return
        self.EndModal(wx.ID_EXIT)

    def confirmLoseChanges(self, msg):
        if not self.modified:
            return True

        dlg = wx.MessageDialog(
            self,
            "Are you sure you want to " + msg + "?\n"
            "There are changes to your settings that "
            "will be lost.",
            "Changes pending",
            wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION,
        )
        rc = dlg.ShowModal()
        dlg.Destroy()

        if rc != wx.ID_YES:
            return False

        return True
