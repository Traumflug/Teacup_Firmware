import wx
from configtool.page import Page


class PinoutsPage(wx.Panel, Page):
    def __init__(self, parent, nb, idPg, font):
        wx.Panel.__init__(self, nb, wx.ID_ANY)
        Page.__init__(self, font)
        self.parent = parent
        self.id = idPg

        pinXkeys = [
            ("X_STEP_PIN", 2),
            ("X_DIR_PIN", 2),
            ("X_MIN_PIN", 2),
            ("X_MAX_PIN", 2),
            ("X_ENABLE_PIN", 2),
            ("X_INVERT_DIR", 1),
            ("X_INVERT_MIN", 1),
            ("X_INVERT_MAX", 1),
            ("X_INVERT_ENABLE", 1),
        ]
        pinYkeys = [
            ("Y_STEP_PIN", 2),
            ("Y_DIR_PIN", 2),
            ("Y_MIN_PIN", 2),
            ("Y_MAX_PIN", 2),
            ("Y_ENABLE_PIN", 2),
            ("Y_INVERT_DIR", 1),
            ("Y_INVERT_MIN", 1),
            ("Y_INVERT_MAX", 1),
            ("Y_INVERT_ENABLE", 1),
        ]
        pinZkeys = [
            ("Z_STEP_PIN", 2),
            ("Z_DIR_PIN", 2),
            ("Z_MIN_PIN", 2),
            ("Z_MAX_PIN", 2),
            ("Z_ENABLE_PIN", 2),
            ("Z_INVERT_DIR", 1),
            ("Z_INVERT_MIN", 1),
            ("Z_INVERT_MAX", 1),
            ("Z_INVERT_ENABLE", 1),
        ]
        pinEkeys = [
            ("E_STEP_PIN", 2),
            ("E_DIR_PIN", 2),
            ("E_ENABLE_PIN", 2),
            ("E_INVERT_DIR", 1),
            ("E_INVERT_ENABLE", 1),
        ]

        self.labels = {
            "X_STEP_PIN": "Step Pin:",
            "X_DIR_PIN": "Direction Pin:",
            "X_MIN_PIN": "Minimum Pin:",
            "X_MAX_PIN": "Maximum Pin:",
            "X_ENABLE_PIN": "Enable Pin:",
            "X_INVERT_DIR": "Invert Direction",
            "X_INVERT_MIN": "Invert Minimum",
            "X_INVERT_MAX": "Invert Maximum",
            "X_INVERT_ENABLE": "Invert Enable",
            "Y_STEP_PIN": "Step Pin:",
            "Y_DIR_PIN": "Direction Pin:",
            "Y_MIN_PIN": "Minimum Pin:",
            "Y_MAX_PIN": "Maximum Pin:",
            "Y_ENABLE_PIN": "Enable Pin:",
            "Y_INVERT_DIR": "Invert Direction",
            "Y_INVERT_MIN": "Invert Minimum",
            "Y_INVERT_MAX": "Invert Maximum",
            "Y_INVERT_ENABLE": "Invert Enable",
            "Z_STEP_PIN": "Step Pin:",
            "Z_DIR_PIN": "Direction Pin:",
            "Z_MIN_PIN": "Minimum Pin:",
            "Z_MAX_PIN": "Maximum Pin:",
            "Z_ENABLE_PIN": "Enable Pin:",
            "Z_INVERT_DIR": "Invert Direction",
            "Z_INVERT_MIN": "Invert Minimum",
            "Z_INVERT_MAX": "Invert Maximum",
            "Z_INVERT_ENABLE": "Invert Enable",
            "E_STEP_PIN": "Step Pin:",
            "E_DIR_PIN": "Direction Pin:",
            "E_ENABLE_PIN": "Enable Pin:",
            "E_INVERT_DIR": "Invert Direction",
            "E_INVERT_ENABLE": "Invert Enable",
            "PS_ON_PIN": "PSU On Pin:",
            "PS_INVERT_ON": "Invert PSU On Pin",
            "PS_MOSFET_PIN": "PSU MOSFET Pin:",
            "STEPPER_ENABLE_PIN": "Stepper Enable Pin:",
            "STEPPER_INVERT_ENABLE": "Stepper Invert Enable",
            "SD_CARD_SELECT_PIN": "SD Card Pin:",
            "DEBUG_LED_PIN": "Debug LED Pin:",
        }

        labelWidth = 120

        sz = wx.GridBagSizer()
        sz.Add((10, 10), pos=(0, 0))

        b = wx.StaticBox(self, wx.ID_ANY, "X Axis")
        b.SetFont(font)
        sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
        sbox.Add((5, 5))
        for k, ctype in pinXkeys:
            if ctype == 0:
                tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlPin)
                sbox.Add(tc)
            elif ctype == 2:
                tc = self.addPinChoice(k, labelWidth)
                sbox.Add(tc)
            else:
                cb = self.addCheckBox(k, self.onCheckBox)
                sbox.Add(cb, 1, wx.LEFT, 30)

            sbox.Add((5, 5))

        sz.Add(sbox, pos=(1, 1))

        b = wx.StaticBox(self, wx.ID_ANY, "Y Axis")
        b.SetFont(font)
        sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
        sbox.Add((5, 5))
        for k, ctype in pinYkeys:
            if ctype == 0:
                tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlPin)
                sbox.Add(tc)
            elif ctype == 2:
                tc = self.addPinChoice(k, labelWidth)
                sbox.Add(tc)
            else:
                cb = self.addCheckBox(k, self.onCheckBox)
                sbox.Add(cb, 1, wx.LEFT, 30)

            sbox.Add((5, 5))

        sz.Add(sbox, pos=(1, 3))

        b = wx.StaticBox(self, wx.ID_ANY, "Z Axis")
        b.SetFont(font)
        sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
        sbox.Add((5, 5))
        for k, ctype in pinZkeys:
            if ctype == 0:
                tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlPin)
                sbox.Add(tc)
            elif ctype == 2:
                tc = self.addPinChoice(k, labelWidth)
                sbox.Add(tc)
            else:
                cb = self.addCheckBox(k, self.onCheckBox)
                sbox.Add(cb, 1, wx.LEFT, 30)

            sbox.Add((5, 5))

        sz.Add(sbox, pos=(1, 5))

        b = wx.StaticBox(self, wx.ID_ANY, "E Axis")
        b.SetFont(font)
        sbox = wx.StaticBoxSizer(b, wx.VERTICAL)
        sbox.Add((5, 5))
        for k, ctype in pinEkeys:
            if ctype == 0:
                tc = self.addTextCtrl(k, labelWidth, self.onTextCtrlPin)
                sbox.Add(tc)
            elif ctype == 2:
                tc = self.addPinChoice(k, labelWidth)
                sbox.Add(tc)
            else:
                cb = self.addCheckBox(k, self.onCheckBox)
                sbox.Add(cb, 1, wx.LEFT, 30)

            sbox.Add((5, 5))

        sz.Add(sbox, pos=(1, 7))

        k = "STEPPER_ENABLE_PIN"
        tc = self.addPinChoice(k, labelWidth + 20)
        sz.Add(tc, pos=(3, 1))

        sz.Add((10, 10), pos=(4, 1))

        k = "STEPPER_INVERT_ENABLE"
        cb = self.addCheckBox(k, self.onCheckBox)
        sz.Add(cb, pos=(5, 1), flag=wx.ALIGN_CENTER_HORIZONTAL)

        k = "PS_ON_PIN"
        tc = self.addPinChoice(k, labelWidth)
        sz.Add(tc, pos=(3, 3))

        k = "PS_INVERT_ON"
        cb = self.addCheckBox(k, self.onCheckBox)
        sz.Add(cb, pos=(5, 3), flag=wx.ALIGN_CENTER_HORIZONTAL)

        k = "PS_MOSFET_PIN"
        tc = self.addPinChoice(k, labelWidth)
        sz.Add(tc, pos=(7, 3))

        k = "SD_CARD_SELECT_PIN"
        tc = self.addPinChoice(k, labelWidth)
        sz.Add(tc, pos=(3, 7))

        k = "DEBUG_LED_PIN"
        tc = self.addPinChoice(k, labelWidth)
        sz.Add(tc, pos=(5, 7))

        self.SetSizer(sz)
        self.enableAll(False)

    def onChoice(self, evt):
        self.assertModified(True)
        evt.Skip()
