import wx


class SensorList(wx.ListCtrl):
    def __init__(self, parent, font):
        self.parent = parent
        self.currentItem = None
        wx.ListCtrl.__init__(
            self,
            parent,
            wx.ID_ANY,
            size=(105 + 105 + 55 + 280 + 4, 100),
            style=wx.LC_REPORT | wx.LC_VIRTUAL | wx.LC_HRULES | wx.LC_VRULES,
        )

        self.SetFont(font)

        self.valid = []
        self.sensorList = []

        self.InsertColumn(0, "Name")
        self.InsertColumn(1, "Sensor Type")
        self.InsertColumn(2, "Pin")
        self.InsertColumn(3, "Additional")
        self.SetColumnWidth(0, 105)
        self.SetColumnWidth(1, 105)
        self.SetColumnWidth(2, 55)
        self.SetColumnWidth(3, 280)

        self.SetItemCount(0)

        self.Bind(wx.EVT_LIST_ITEM_SELECTED, self.OnItemSelected)
        self.Bind(wx.EVT_LIST_ITEM_DESELECTED, self.OnItemDeselected)

    def updateList(self, sensorList):
        self.sensorList = sensorList
        self.valid = [True] * len(sensorList)
        self.currentItem = None
        self.parent.setItemSelected(None)
        i = self.GetFirstSelected()
        while i != -1:
            self.Select(i, False)
            i = self.GetFirstSelected()

        self.SetItemCount(len(sensorList))

    def setRowValidity(self, i, flag=False):
        if i < 0 or i >= len(self.sensorList):
            return

        self.valid[i] = flag
        self.Refresh()

    def setTableValidity(self, flag=False):
        for i in range(len(self.sensorList)):
            self.setRowValidity(i, flag)

    def OnItemSelected(self, event):
        self.currentItem = event.GetIndex()
        self.parent.setItemSelected(self.currentItem)

    def OnItemDeselected(self, event):
        self.currentItem = None
        self.parent.setItemSelected(None)

    def getColumnText(self, index, col):
        item = self.GetItem(index, col)
        return item.GetText()

    def OnGetItemText(self, item, col):
        if item < 0 or item >= len(self.sensorList):
            return "Error - no sensors"

        s = self.sensorList[item]

        if col == 0:
            return s[0]
        elif col == 1:
            return s[1]
        elif col == 2:
            return s[2]
        elif len(s) == 3:
            return ""
        else:
            if s[3] is None:
                return ""
            else:
                return "[%s]" % (", ".join(s[3]))
