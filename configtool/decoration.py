from __future__ import print_function

import wx
import os.path


class Decoration(object):
    def __new__(type, *args):
        # Make it a Singleton.
        if not "_the_instance" in type.__dict__:
            type._the_instance = object.__new__(type)

        return type._the_instance

    def __init__(self):
        if not "_ready" in dir(self):
            self._ready = True
            # It's a Singleton. Initialisations go in here.
            self.backPic = None
            self.backPicOffset = (0, -25)

        if not self.backPic:
            backPicPath = os.path.join("configtool", "background.png")
            if os.path.exists(backPicPath):
                backPic = wx.Bitmap(backPicPath)
                if backPic.IsOk():
                    self.backPic = backPic
                else:
                    print("Background picture %s damaged." % backPicPath)
            else:
                print("Background picture %s doesn't exist." % backPicPath)

    def getBackgroundColour(self):
        return wx.Colour(237, 237, 237)

    # On wxFrames, bind this to wx.EVT_ERASE_BACKGROUND
    # On wxPanels, bind this to wx.EVT_PAINT
    def onPaintBackground(self, evt):
        client = evt.GetEventObject()
        topLevel = client.GetTopLevelParent()

        try:
            dc = evt.GetDC()
        except:
            dc = wx.PaintDC(client)

        if dc:
            # Now draw the background picture with pseudo-transparency. This is,
            # each background is drawn with the same picture, without transparency,
            # and offsetted just right to have all backgrounds in the same position
            # relative to the *toplevel* window, not relative to the current
            # subwindow as usual.

            # Align bottom right.
            offX, offY = (
                topLevel.GetClientSize() - self.backPic.GetSize() + self.backPicOffset
            )

            if client != topLevel:
                # Note: trying to figure this additional offset via various
                #       .GetScreenPosition() or .GetPosition() or whatever is hopeless.
                #       Of many many tries only this worked on Linux.
                offX, offY = client.ScreenToClient(
                    topLevel.ClientToScreen((offX, offY))
                )

            if self.backPic:
                dc.DrawBitmap(self.backPic, offX, offY)

        evt.Skip()
