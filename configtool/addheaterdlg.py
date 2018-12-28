import wx
from configtool.data import BSIZESMALL, offsetChLabel, offsetTcLabel


class AddHeaterDlg(wx.Dialog):
    def __init__(
        self,
        parent,
        names,
        pins,
        font,
        name="",
        pin="",
        invert="0",
        pwm="1",
        max_pwm="100",
    ):
        wx.Dialog.__init__(self, parent, wx.ID_ANY, "Add heater", size=(400, 204))
        self.SetFont(font)
        self.Bind(wx.EVT_CLOSE, self.onCancel)

        self.names = names
        self.choices = pins

        self.nameValid = name != ""
        self.maxPWMValid = max_pwm != ""
        self.pwmValid = pwm != ""

        sz = wx.BoxSizer(wx.VERTICAL)
        gsz = wx.GridBagSizer()
        gsz.Add((20, 20), pos=(0, 0))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(
            self, wx.ID_ANY, "Heater Name:", size=(80, -1), style=wx.ALIGN_RIGHT
        )
        st.SetFont(font)
        lsz.Add(st, 1, wx.TOP, offsetTcLabel)

        self.tcName = wx.TextCtrl(self, wx.ID_ANY, name)
        self.tcName.SetFont(font)
        if not name:
            self.tcName.SetBackgroundColour("pink")
        self.tcName.Bind(wx.EVT_TEXT, self.onNameEntry)
        lsz.Add(self.tcName)
        self.tcName.SetToolTip("Enter a unique name for this heater.")

        gsz.Add(lsz, pos=(1, 1))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(self, wx.ID_ANY, "Pin:", size=(80, -1), style=wx.ALIGN_RIGHT)
        st.SetFont(font)
        lsz.Add(st, 1, wx.TOP, offsetChLabel)

        self.chPin = wx.Choice(self, wx.ID_ANY, choices=pins)
        self.chPin.SetFont(font)
        self.chPin.Bind(wx.EVT_CHOICE, self.onChoice)
        i = self.chPin.FindString(pin)
        if i == wx.NOT_FOUND:
            self.chPin.SetSelection(0)
        else:
            self.chPin.SetSelection(i)
        lsz.Add(self.chPin)
        self.chPin.SetToolTip("Choose a pin for this heater.")

        gsz.Add(lsz, pos=(3, 1))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(
            self, wx.ID_ANY, "Max PWM:", size=(80, -1), style=wx.ALIGN_RIGHT
        )
        st.SetFont(font)
        lsz.Add(st, 1, wx.TOP, offsetChLabel)

        self.tcMaxPWM = wx.TextCtrl(self, wx.ID_ANY, max_pwm)
        self.tcMaxPWM.SetFont(font)
        self.tcMaxPWM.Bind(wx.EVT_TEXT, self.onMaxPWM)
        lsz.Add(self.tcMaxPWM)
        self.tcMaxPWM.SetToolTip(
            "Enter max. PWM value in [%]. Typically \n"
            "between 40 and 100. Standard is 100.\n"
            "Valid values 1 to 100."
        )

        gsz.Add(lsz, pos=(5, 1))

        self.cbInv = wx.CheckBox(self, wx.ID_ANY, "Invert")
        self.cbInv.SetFont(font)
        self.cbInv.SetValue(int(invert) != 0)
        self.cbInv.SetToolTip("Invert the pin signal.")

        gsz.Add(self.cbInv, pos=(3, 3))

        lsz = wx.BoxSizer(wx.HORIZONTAL)
        st = wx.StaticText(self, wx.ID_ANY, "PWM:", size=(60, -1), style=wx.ALIGN_RIGHT)
        st.SetFont(font)
        lsz.Add(st, 1, wx.TOP, offsetChLabel)

        self.tcPwm = wx.TextCtrl(self, wx.ID_ANY, pwm, size=(60, -1))
        self.tcPwm.SetFont(font)
        self.tcPwm.Bind(wx.EVT_TEXT, self.onPWM)
        lsz.Add(self.tcPwm)
        self.tcPwm.SetToolTip(
            "Use Pulse Width Modulation. "
            "Hardware PWM if available or "
            "Software PWM. When FORCE_SOFTWARE_PWM "
            "is set, always software PWM for 1 and "
            "hardware PWM for >= 2."
        )

        gsz.Add((50, 15), pos=(1, 2))
        gsz.Add(lsz, pos=(1, 3))
        gsz.Add((20, 20), pos=(4, 4))

        sz.Add(gsz)
        sz.Add((30, 30))

        bsz = wx.BoxSizer(wx.HORIZONTAL)

        self.bSave = wx.Button(self, wx.ID_ANY, "Save", size=BSIZESMALL)
        self.bSave.SetFont(font)
        self.bSave.Bind(wx.EVT_BUTTON, self.onSave)
        bsz.Add(self.bSave)

        bsz.Add(30, 100)

        self.bCancel = wx.Button(self, wx.ID_ANY, "Cancel", size=BSIZESMALL)
        self.bCancel.SetFont(font)
        self.bCancel.Bind(wx.EVT_BUTTON, self.onCancel)
        bsz.Add(self.bCancel)

        sz.Add(bsz, flag=wx.ALIGN_CENTER_HORIZONTAL)
        sz.Add((10, 10))
        self.SetSizer(sz)

        self.checkDlgValidity()
        self.Fit()

    def onNameEntry(self, evt):
        tc = evt.GetEventObject()
        w = tc.GetValue().strip()
        if w == "":
            self.nameValid = False
        else:
            if w in self.names:
                self.nameValid = False
            else:
                self.nameValid = True

        if self.nameValid:
            tc.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOW))
        else:
            tc.SetBackgroundColour("pink")
        tc.Refresh()

        self.checkDlgValidity()
        evt.Skip()

    def onChoice(self, evt):
        pass

    def onMaxPWM(self, evt):
        tc = evt.GetEventObject()
        w = tc.GetValue().strip()
        if w == "":
            self.maxPWMValid = False
        else:
            if int(w) > 0 and int(w) <= 100:
                self.maxPWMValid = True
            else:
                self.maxPWMValid = False

        if self.maxPWMValid:
            tc.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOW))
        else:
            tc.SetBackgroundColour("pink")
        tc.Refresh()

        self.checkDlgValidity()
        if evt is not None:
            evt.Skip()

    def onPWM(self, evt):
        tc = evt.GetEventObject()
        w = tc.GetValue().strip()
        if w == "":
            self.pwmValid = False
        else:
            if int(w) >= 0:
                self.pwmValid = True
            else:
                self.pwmValid = False

        if self.pwmValid:
            tc.SetBackgroundColour(wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOW))
        else:
            tc.SetBackgroundColour("pink")
        tc.Refresh()

        self.checkDlgValidity()
        if evt is not None:
            evt.Skip()

    def checkDlgValidity(self):
        if self.nameValid and self.maxPWMValid and self.pwmValid:
            self.bSave.Enable(True)
        else:
            self.bSave.Enable(False)

    def getValues(self):
        nm = self.tcName.GetValue()
        pin = self.choices[self.chPin.GetSelection()]

        if self.cbInv.IsChecked():
            invert = "1"
        else:
            invert = "0"

        pwm = self.tcPwm.GetValue()

        max_pwm = self.tcMaxPWM.GetValue()

        return (nm, pin, invert, pwm, max_pwm)

    def onSave(self, evt):
        self.EndModal(wx.ID_OK)

    def onCancel(self, evt):
        self.EndModal(wx.ID_CANCEL)
