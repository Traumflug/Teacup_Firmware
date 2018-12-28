import wx
from configtool.data import BSIZESMALL, reFloat, offsetChLabel, offsetTcLabel


class CalcScrew(wx.Dialog):
    def __init__(self, parent, font, cbUse):
        wx.Dialog.__init__(
            self,
            parent,
            wx.ID_ANY,
            "Steps calculator for screw driven axes",
            size=(400, 204),
        )
        self.SetFont(font)
        self.Bind(wx.EVT_CLOSE, self.onExit)

        self.use = cbUse
        labelWidth = 150

        hsz = wx.BoxSizer(wx.HORIZONTAL)
        hsz.Add((10, 10))

        sz = wx.BoxSizer(wx.VERTICAL)
        sz.Add((10, 10))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(
            self, wx.ID_ANY, "Step Angle:", size=(labelWidth, -1), style=wx.ALIGN_RIGHT
        )
        st.SetFont(font)
        lsz.Add(st, 1, wx.TOP, offsetChLabel)
        lsz.Add((5, 5))

        stepAngles = [
            "1.8 (200 per revolution)",
            "0.9 (400 per revolution)",
            "7.5 (48 per revolution)",
        ]
        self.stepAngleValues = [200, 400, 48]
        tc = wx.Choice(self, wx.ID_ANY, choices=stepAngles)
        tc.SetFont(font)
        tc.SetSelection(0)
        tc.Bind(wx.EVT_CHOICE, self.onChoice)
        lsz.Add(tc)
        tc.SetToolTip("Step angle. Depends on your type of stepper motor.")
        self.tcStep = tc

        sz.Add(lsz)
        sz.Add((10, 10))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(
            self,
            wx.ID_ANY,
            "Microstepping:",
            size=(labelWidth, -1),
            style=wx.ALIGN_RIGHT,
        )
        st.SetFont(font)
        lsz.Add(st, 1, wx.TOP, offsetChLabel)
        lsz.Add((5, 5))

        microStepping = [
            "1 - full step",
            "1/2 - half step",
            "1/4 - quarter step",
            "1/8",
            "1/16",
            "1/32",
            "1/64",
            "1/128",
        ]
        self.microSteppingValues = [1, 2, 4, 8, 16, 32, 64, 128]
        tc = wx.Choice(self, wx.ID_ANY, choices=microStepping)
        tc.SetFont(font)
        tc.Bind(wx.EVT_CHOICE, self.onChoice)
        tc.SetSelection(4)
        lsz.Add(tc)
        tc.SetToolTip(
            "Microstepping. Most boards allow to change this by "
            "setting jumpers. The value here must match the "
            "setting on the board in conjunction with the type "
            "of stepper driver chip."
        )
        self.tcMicroStepping = tc

        sz.Add(lsz)
        sz.Add((10, 10))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(
            self,
            wx.ID_ANY,
            "Screw Pitch (mm/rev):",
            size=(labelWidth, -1),
            style=wx.ALIGN_RIGHT,
        )
        st.SetFont(font)
        lsz.Add(st, 1, wx.TOP, offsetTcLabel)
        lsz.Add((5, 5))

        tc = wx.TextCtrl(self, wx.ID_ANY, "2", style=wx.TE_RIGHT)
        tc.SetFont(font)
        tc.Bind(wx.EVT_TEXT, self.onTextCtrlFloat)
        lsz.Add(tc)
        tc.SetToolTip("Screw pitch. Defined by the pitch of the screw.")
        self.tcScrewPitch = tc

        lsz.Add((5, 5))

        screwPresets = [
            "-",
            "M8 - metric (1.25 mm/rev)",
            "M6 - metric (1 mm/rev)",
            "M5 - metric (0.8 mm/rev)",
            "12 (12 mm/rev)",
            "16 (16 mm/rev)",
            "25 (25 mm/rev)",
            '5/15"-18 imperial coarse (1.41111 mm/rev)',
            '3/16"-20 imperial (1.270 mm/rev)',
            '1/4"-16 ACME (1.5875 mm/rev)',
        ]
        self.screwPresetValues = [
            -1,
            1.25,
            1.00,
            0.8,
            12.0,
            16.0,
            25.0,
            1.41111,
            1.270,
            1.5875,
        ]
        tc = wx.Choice(self, wx.ID_ANY, choices=screwPresets)
        tc.SetFont(font)
        tc.SetSelection(0)
        tc.Bind(wx.EVT_CHOICE, self.onPresetChoice)
        lsz.Add(tc)
        tc.SetToolTip("Screw pitch presets.")
        self.tcPresets = tc

        sz.Add(lsz)
        sz.Add((10, 10))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(
            self, wx.ID_ANY, "Gear Ratio:", size=(labelWidth, -1), style=wx.ALIGN_RIGHT
        )
        st.SetFont(font)
        lsz.Add(st, 1, wx.TOP, offsetTcLabel)
        lsz.Add((5, 5))

        tc = wx.TextCtrl(self, wx.ID_ANY, "1", size=(40, -1), style=wx.TE_RIGHT)
        tc.SetFont(font)
        tc.Bind(wx.EVT_TEXT, self.onTextCtrlFloat)
        lsz.Add(tc)
        tc.SetToolTip("Gear ratio. 1:1 if there is no gear.")
        self.tcRatioTop = tc

        lsz.Add((5, 5))
        st = wx.StaticText(self, wx.ID_ANY, ":")
        st.SetFont(font)
        lsz.Add(st)
        lsz.Add((5, 5))

        tc = wx.TextCtrl(self, wx.ID_ANY, "1", size=(40, -1), style=wx.TE_RIGHT)
        tc.SetFont(font)
        tc.Bind(wx.EVT_TEXT, self.onTextCtrlFloat)
        lsz.Add(tc)
        tc.SetToolTip("Gear ratio. 1:1 if there is no gear.")
        self.tcRatioBottom = tc

        sz.Add(lsz)
        sz.Add((30, 30))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(
            self, wx.ID_ANY, "Result:", size=(labelWidth, -1), style=wx.ALIGN_RIGHT
        )
        st.SetFont(font)
        lsz.Add(st)
        lsz.Add((5, 5))

        tc = wx.StaticText(self, wx.ID_ANY, "", size=(300, -1), style=wx.ALIGN_LEFT)
        tc.SetFont(font)
        lsz.Add(tc)
        self.tcResult = tc

        sz.Add(lsz)
        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(
            self, wx.ID_ANY, "Resolution:", size=(labelWidth, -1), style=wx.ALIGN_RIGHT
        )
        st.SetFont(font)
        lsz.Add(st)
        lsz.Add((5, 5))

        tc = wx.StaticText(self, wx.ID_ANY, "", size=(300, -1), style=wx.ALIGN_LEFT)
        tc.SetFont(font)
        lsz.Add(tc)
        self.tcResolution = tc

        sz.Add(lsz)

        sz.Add((20, 20))

        bsz = wx.BoxSizer(wx.HORIZONTAL)
        b = wx.Button(self, wx.ID_ANY, "Use for X", size=BSIZESMALL)
        b.SetFont(font)
        self.Bind(wx.EVT_BUTTON, self.onUseForX, b)
        bsz.Add(b)
        self.bUseForX = b
        bsz.Add((5, 5))

        b = wx.Button(self, wx.ID_ANY, "Use for Y", size=BSIZESMALL)
        b.SetFont(font)
        self.Bind(wx.EVT_BUTTON, self.onUseForY, b)
        bsz.Add(b)
        self.bUseForY = b
        bsz.Add((5, 5))

        b = wx.Button(self, wx.ID_ANY, "Use for Z", size=BSIZESMALL)
        b.SetFont(font)
        self.Bind(wx.EVT_BUTTON, self.onUseForZ, b)
        bsz.Add(b)
        self.bUseForZ = b
        bsz.Add((5, 5))

        b = wx.Button(self, wx.ID_ANY, "Use for E", size=BSIZESMALL)
        b.SetFont(font)
        self.Bind(wx.EVT_BUTTON, self.onUseForE, b)
        bsz.Add(b)
        self.bUseForE = b

        sz.Add(bsz, flag=wx.ALIGN_CENTER_HORIZONTAL)
        sz.Add((10, 10))

        hsz.Add(sz)
        hsz.Add((10, 10))

        self.enableUseButtons(False)

        self.SetSizer(hsz)

        self.Fit()

        self.calculate()

    def calculate(self):
        self.tcResult.SetLabel("")
        self.tcResolution.SetLabel("")
        self.enableUseButtons(False)
        s = self.tcStep.GetSelection()
        sv = self.stepAngleValues[s]

        try:
            sp = float(self.tcScrewPitch.GetValue())
        except:
            return

        try:
            ratioA = float(self.tcRatioTop.GetValue())
        except:
            return

        try:
            ratioB = float(self.tcRatioBottom.GetValue())
        except:
            return

        s = self.tcMicroStepping.GetSelection()
        msv = self.microSteppingValues[s]

        ratio = ratioA / ratioB
        steps = sv * msv

        resultmm = steps / sp / ratio
        self.result = int(resultmm * 1000.0)

        self.tcResult.SetLabel("%d steps/m   (%.3f steps/mm)" % (self.result, resultmm))
        self.tcResolution.SetLabel("%.3f micrometers" % (1.0 / resultmm * 1000.0))
        self.enableUseButtons(True)

    def enableUseButtons(self, flag):
        self.bUseForX.Enable(flag)
        self.bUseForY.Enable(flag)
        self.bUseForZ.Enable(flag)
        self.bUseForE.Enable(flag)

    def onUseForX(self, evt):
        self.use("STEPS_PER_M_X", self.result)

    def onUseForY(self, evt):
        self.use("STEPS_PER_M_Y", self.result)

    def onUseForZ(self, evt):
        self.use("STEPS_PER_M_Z", self.result)

    def onUseForE(self, evt):
        self.use("STEPS_PER_M_E", self.result)

    def onPresetChoice(self, evt):
        s = self.tcPresets.GetSelection()
        sv = self.screwPresetValues[s]
        if sv < 0:
            return

        s = "%f" % sv
        s = s.rstrip("0")
        if s[-1] == ".":
            s += "0"
        self.tcScrewPitch.SetValue(s)

    def onChoice(self, evt):
        self.calculate()

    def onTextCtrlFloat(self, evt):
        tc = evt.GetEventObject()
        w = tc.GetValue().strip()
        if w == "":
            valid = False
        else:
            m = reFloat.match(w)
            if m:
                valid = True
            else:
                valid = False

        if valid:
            tc.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOW))
        else:
            tc.SetBackgroundColour("pink")
        tc.Refresh()
        self.calculate()
        evt.Skip()

    def onExit(self, evt):
        self.EndModal(wx.ID_OK)
