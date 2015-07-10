
import wx.lib.newevent
import thread, shlex, subprocess
import os, re
from os.path import isfile, join
from sys import platform

if platform.startswith("win"):
  from _subprocess import STARTF_USESHOWWINDOW

(scriptEvent, EVT_SCRIPT_UPDATE) = wx.lib.newevent.NewEvent()
SCRIPT_RUNNING = 1
SCRIPT_FINISHED = 2
SCRIPT_CANCELLED = 3

TOOLPATHS_INSIDE_ARDUINO = [
  "hardware/tools/avr/bin/",
  "hardware/tools/" # avrdude in Arduino 1.0.x
]
if platform.startswith("darwin"):
  # That's an OS property, the Applicaton Bundle hierarchy.
  pathsCopy = TOOLPATHS_INSIDE_ARDUINO
  TOOLPATHS_INSIDE_ARDUINO = []
  for path in pathsCopy:
    TOOLPATHS_INSIDE_ARDUINO.append("Contents/Resources/Java/" + path)
    TOOLPATHS_INSIDE_ARDUINO.append("Contents/Java/" + path)


class ScriptTools:
  def __init__(self, settings):
    self.settings = settings

  def figureCommandPath(self, baseCommand):
    findConf = False
    if baseCommand == "avrdude":
      findConf = True

    if platform.startswith("win"):
      baseCommand += ".exe"

    if self.settings.arduinodir:
      cmdpath = self.settings.arduinodir

      for pathOption in TOOLPATHS_INSIDE_ARDUINO:
        cmdpathTry = cmdpath
        for dir in pathOption.split("/"):
          cmdpathTry = os.path.join(cmdpathTry, dir)
        cmdpathTry = os.path.join(cmdpathTry, baseCommand)
        if os.path.exists(cmdpathTry):
          cmdpath = "\"" + cmdpathTry + "\""
          break

      if findConf:
        confpath = cmdpath.strip("\"")
        exepos = confpath.rfind(".exe")
        if exepos >= 0:
          confpath = confpath[0:exepos]
        confpath += ".conf"
        if not os.path.exists(confpath):
          confpath = os.path.split(confpath)[0]
          confpath = os.path.split(confpath)[0]
          confpath = os.path.join(confpath, "etc")
          confpath = os.path.join(confpath, "avrdude.conf")
        if os.path.exists(confpath):
          cmdpath += " -C \"" + confpath + "\""

    else:
      cmdpath = baseCommand
      # No need to search avrdude.conf in this case.

    return cmdpath


class ScriptThread:
  def __init__(self, win, script):
    self.win = win
    self.running = False
    self.cancelled = False
    self.script = script

  def Start(self):
    self.running = True
    self.cancelled = False
    thread.start_new_thread(self.Run, ())

  def Stop(self):
    self.cancelled = True

  def IsRunning(self):
    return self.running

  def Run(self):
    if platform.startswith("win"):
      startupinfo = subprocess.STARTUPINFO()
      startupinfo.dwFlags |= STARTF_USESHOWWINDOW

    for cmd in self.script:
      evt = scriptEvent(msg = cmd, state = SCRIPT_RUNNING)
      wx.PostEvent(self.win, evt)
      args = shlex.split(str(cmd))
      try:
        if platform.startswith("win"):
          p = subprocess.Popen(args, stderr = subprocess.STDOUT,
                               stdout = subprocess.PIPE,
                               startupinfo = startupinfo)
        else:
          p = subprocess.Popen(args, stderr = subprocess.STDOUT,
                               stdout = subprocess.PIPE)
      except:
        evt = scriptEvent(msg = "Exception occurred trying to run\n\n%s" % cmd,
                          state = SCRIPT_CANCELLED)
        wx.PostEvent(self.win, evt)
        self.running = False
        return
      obuf = ''
      while not self.cancelled:
        o = p.stdout.read(1)
        if o == '': break
        if o == '\r' or o == '\n':
          if obuf.strip() != "":
            evt = scriptEvent(msg = obuf, state = SCRIPT_RUNNING)
            wx.PostEvent(self.win, evt)
          obuf = ''
        elif ord(o) < 32:
          pass
        else:
          obuf += o

      if self.cancelled:
        evt = scriptEvent(msg = None, state = SCRIPT_CANCELLED)
        wx.PostEvent(self.win, evt)
        p.kill()
        self.running = False
        p.wait()
        return

      rc = p.wait()
      if rc != 0:
        msg = "RC = " + str(rc) + " - Build terminated"
        evt = scriptEvent(msg = msg, state = SCRIPT_CANCELLED)
        wx.PostEvent(self.win, evt)
        self.running = False
        return

      evt = scriptEvent(msg = "", state = SCRIPT_RUNNING)
      wx.PostEvent(self.win, evt)

    evt = scriptEvent(msg = None, state = SCRIPT_FINISHED)
    wx.PostEvent(self.win, evt)

    self.running = False


class Build(wx.Dialog):
  def __init__(self, parent, settings, f_cpu, cpu):
    wx.Dialog.__init__(self, parent, wx.ID_ANY, "Build teacup",
                       style = wx.RESIZE_BORDER + wx.DEFAULT_DIALOG_STYLE)
    self.settings = settings
    self.SetFont(self.settings.font)
    self.root = self.settings.folder
    self.f_cpu = f_cpu
    self.cpu = cpu
    self.Bind(wx.EVT_CLOSE, self.onExit)
    self.cancelPending = False

    hsz = wx.BoxSizer(wx.HORIZONTAL)
    hsz.AddSpacer((10, 10))

    sz = wx.BoxSizer(wx.VERTICAL)
    sz.AddSpacer((10, 10))

    tc = wx.TextCtrl(self, wx.ID_ANY, size = (900, 300),
                     style = wx.TE_READONLY + wx.TE_MULTILINE)
    sz.Add(tc, 1, wx.EXPAND)
    f = wx.Font(8, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL,
                wx.FONTWEIGHT_BOLD)
    tc.SetFont(f)
    self.log = tc

    sz.AddSpacer((10, 10))
    hsz.Add(sz, 1, wx.EXPAND)
    hsz.AddSpacer((10, 10))

    self.SetSizer(hsz)

    self.Fit()
    builddir = join(self.root, "build")
    if not os.path.exists(builddir):
      os.makedirs(builddir)
      self.log.AppendText("Directory %s created.\n\n" % builddir)

    self.compile()

  def compile(self):
    self.generateCompileScript()
    if len(self.script) == 0:
      self.log.AppendText("Nothing to compile!\n")
      self.active = False
    else:
      self.Bind(EVT_SCRIPT_UPDATE, self.compileUpdate)
      self.t = ScriptThread(self, self.script)
      self.active = True
      self.t.Start()

  def link(self):
    self.generateLinkScript()
    if len(self.script) == 0:
      self.log.AppendText("Nothing to link!\n")
      self.active = False
    else:
      self.Bind(EVT_SCRIPT_UPDATE, self.linkUpdate)
      t = ScriptThread(self, self.script)
      self.active = True
      t.Start()

  def report(self):
    self.script = []
    self.reportLines = []
    cmdpath = ScriptTools(self.settings).figureCommandPath("avr-objdump")
    elfpath = "\"" + join(self.root, "build", "teacup.elf") + "\""
    cmd = cmdpath + " -h " + elfpath
    self.script.append(cmd)
    self.Bind(EVT_SCRIPT_UPDATE, self.reportUpdate)
    t = ScriptThread(self, self.script)
    self.active = True
    t.Start()

  def generateCompileScript(self):
    self.script = []
    cmdpath = ScriptTools(self.settings).figureCommandPath("avr-gcc")

    cfiles = [f for f in os.listdir(self.root)
                if isfile(join(self.root,f)) and f.endswith(".c")]
    for f in cfiles:
      basename = f[:-2]
      ofile = basename + ".o"
      alfile = basename + ".al"
      opath = "\"" + join(self.root, "build", ofile) + "\""
      alpath = "\"" + join(self.root, "build", alfile) + "\""
      cpath = "\"" + join(self.root, f) + "\""

      opts = self.settings.cflags
      opts = opts.replace("%ALNAME%", alpath)
      opts = opts.replace("%F_CPU%", self.f_cpu)
      opts = opts.replace("%CPU%", self.cpu)

      cmd = cmdpath + " -c " + opts + " -o " + opath + " " + cpath
      self.script.append(cmd)

  def generateLinkScript(self):
    self.script = []
    cmdpath = ScriptTools(self.settings).figureCommandPath("avr-gcc")

    # This is ugly:
    # Work around a problem of avr-ld.exe coming with Arduino 1.6.4 for
    # Windows. Without this it always drops this error message:
    #   collect2.exe: error: ld returned 5 exit status 255
    # Just enabling verbose messages allows ld.exe to complete without failure.
    if platform == "win32":
      cmdpath += " -Wl,-V"

    ofiles = ["\"" + join(self.root, "build", f) + "\""
              for f in os.listdir(join(self.root, "build"))
                if isfile(join(self.root, "build", f)) and f.endswith(".o")]
    opath = " ".join(ofiles)
    elfpath = "\"" + join(self.root, "build", "teacup.elf") + "\""
    hexpath = "\"" + join(self.root, "teacup.hex") + "\""
    opts = self.settings.cflags
    opts = opts.replace("%ALNAME%", "teacup.elf")
    opts = opts.replace("%F_CPU%", self.f_cpu)
    opts = opts.replace("%CPU%", self.cpu)
    cmd = cmdpath + " " + self.settings.ldflags + " " + opts + " -o " + \
          elfpath + " " + opath + " -lm"
    self.script.append(cmd)

    cmdpath = ScriptTools(self.settings).figureCommandPath("avr-objcopy")
    cmd = cmdpath + " " + self.settings.objcopyflags + " " +  elfpath + \
          " " + hexpath
    self.script.append(cmd)

  def compileUpdate(self, evt):
    if evt.msg is not None:
      self.log.AppendText(evt.msg + "\n")

    if evt.state == SCRIPT_RUNNING:
      pass

    if evt.state == SCRIPT_CANCELLED:
      self.active = False

      if self.cancelPending:
        self.EndModal(wx.ID_OK)

      self.log.AppendText("Build terminated abnormally.\n")

    if evt.state == SCRIPT_FINISHED:
      self.log.AppendText("Compile completed normally.\n\n")
      self.link()

  def linkUpdate(self, evt):
    if evt.msg is not None:
      self.log.AppendText(evt.msg + "\n")

    if evt.state == SCRIPT_RUNNING:
      pass
    if evt.state == SCRIPT_CANCELLED:
      self.log.AppendText("Link terminated abnormally.\n")
      self.active = False
    if evt.state == SCRIPT_FINISHED:
      self.log.AppendText("Link completed normally.\n")
      self.report()

  def reportUpdate(self, evt):
    if evt.state == SCRIPT_RUNNING:
      if evt.msg is not None:
        self.reportLines.append(evt.msg)
    if evt.state == SCRIPT_CANCELLED:
      self.log.AppendText(evt.msg + "\n")
      self.log.AppendText("Report terminated abnormally.\n")
      self.active = False
    if evt.state == SCRIPT_FINISHED:
      self.formatReport()
      self.log.AppendText("\nBuild completed normally.\n")
      self.active = False

  def formatReportLine(self, m, name, v168, v328, v644, v1280):
    t = m.groups()
    v = int(t[0], 16)
    self.log.AppendText(("%12s:  %6d bytes   %6.2f%%   %6.2f%%"
                         "   %6.2f%%   %6.2f%%\n") %
                        (name, v, v / float(v168 * 1024) * 100.0,
                         v / float(v328 * 1024) * 100.0,
                         v / float(v644 * 1024) * 100.0,
                         v / float(v1280 * 1024) * 100.0))

  def formatReport(self):
    reText = re.compile("\.text\s+([0-9a-f]+)")
    reBss = re.compile("\.bss\s+([0-9a-f]+)")
    reEEProm = re.compile("\.eeprom\s+([0-9a-f]+)")

    self.log.AppendText("\n                   ATmega...     '168   '328(P)"
                        "   '644(P)     '1280\n")
    for l in self.reportLines:
      m = reText.search(l)
      if m:
        self.formatReportLine(m, "FLASH", 14, 30, 62, 126)
      else:
        m = reBss.search(l)
        if m:
          self.formatReportLine(m, "RAM", 1, 2, 4, 8)
        else:
          m = reEEProm.search(l)
          if m:
            self.formatReportLine(m, "EEPROM", 1, 2, 2, 4)

  def onExit(self, evt):
    if self.active:
      dlg = wx.MessageDialog(self, "Are you sure you want to cancel building?",
                             "Build active",
                             wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
      rc = dlg.ShowModal()
      dlg.Destroy()

      if rc == wx.ID_YES:
        self.cancelPending = True
        self.log.AppendText("Cancelling...\n")
        self.t.Stop()
      return

    self.EndModal(wx.ID_OK)


class Upload(wx.Dialog):
  def __init__(self, parent, settings, f_cpu, cpu):
    wx.Dialog.__init__(self, parent, wx.ID_ANY, "Upload teacup",
                       style = wx.RESIZE_BORDER + wx.DEFAULT_DIALOG_STYLE)
    self.settings = settings
    self.SetFont(self.settings.font)
    self.root = self.settings.folder
    self.f_cpu = f_cpu
    self.cpu = cpu
    self.baud = self.settings.uploadspeed
    self.Bind(wx.EVT_CLOSE, self.onExit)
    self.cancelPending = False

    hsz = wx.BoxSizer(wx.HORIZONTAL)
    hsz.AddSpacer((10, 10))

    sz = wx.BoxSizer(wx.VERTICAL)
    sz.AddSpacer((10, 10))

    tc = wx.TextCtrl(self, wx.ID_ANY, size = (900, 300),
                     style = wx.TE_READONLY + wx.TE_MULTILINE)
    sz.Add(tc, 1, wx.EXPAND)
    f = wx.Font(8, wx.FONTFAMILY_MODERN, wx.FONTSTYLE_NORMAL,
                wx.FONTWEIGHT_BOLD)
    tc.SetFont(f)
    self.log = tc

    sz.AddSpacer((10, 10))
    hsz.Add(sz, 1, wx.EXPAND)
    hsz.AddSpacer((10, 10))

    self.SetSizer(hsz)

    self.Fit()
    self.generateUploadScript()
    if len(self.script) == 0:
      self.log.AppendText("Nothing to upload!\n")
      self.active = False
    else:
      self.Bind(EVT_SCRIPT_UPDATE, self.uploadUpdate)
      self.t = ScriptThread(self, self.script)
      self.active = True
      self.t.Start()

  def generateUploadScript(self):
    self.script = []
    cmdpath = ScriptTools(self.settings).figureCommandPath("avrdude")
    hexpath = "\"" + join(self.root, "teacup.hex") + "\""

    cmd = cmdpath + " -c %s -b %s -p %s -P %s -U flash:w:%s:i" % \
          (self.settings.programmer, self.baud, self.cpu, self.settings.port,
           hexpath)
    self.script.append(cmd)

  def uploadUpdate(self, evt):
    if evt.msg is not None:
      self.log.AppendText(evt.msg + "\n")

    if evt.state == SCRIPT_RUNNING:
      pass

    if evt.state == SCRIPT_CANCELLED:
      self.active = False

      if self.cancelPending:
        self.EndModal(wx.ID_OK)

      self.log.AppendText("Upload terminated abnormally.\n")

    if evt.state == SCRIPT_FINISHED:
      self.log.AppendText("Upload completed normally.\n")
      self.active = False

  def onExit(self, evt):
    if self.active:
      dlg = wx.MessageDialog(self, "Are you sure you want to cancel upload?",
                             "Upload active",
                             wx.YES_NO | wx.NO_DEFAULT | wx.ICON_INFORMATION)
      rc = dlg.ShowModal()
      dlg.Destroy()

      if rc == wx.ID_YES:
        self.cancelPending = True
        self.log.AppendText("Cancelling...\n")
        self.t.Stop()
      return

    self.EndModal(wx.ID_OK)
