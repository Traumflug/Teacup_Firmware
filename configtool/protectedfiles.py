import os
import re
from configtool.data import reHelpTextStart, reHelpTextEnd

protectedFiles = [
  "board.3drag.h",
  "board.gen3.h",
  "board.gen6.h",
  "board.gen7-v1.1-v1.3.h",
  "board.gen7-v1.4.h",
  "board.ramps-v1.2.h",
  "board.ramps-v1.3.h",
  "board.rumba.h",
  "board.sanguinololu-v1.1.h",
  "board.sanguinololu-v1.2.h",
  "board.sanguish.h",
  "board.teensy-v2.0.h",
  "board.teensy++-v2.0.h",
  "printer.mendel.h",
  "printer.wolfstrap.h"
]


def getDescription(cfgdir, ftype, key):
  helpText = []
  
  for fn in protectedFiles:
    if not fn.startswith(ftype + '.'):
      continue
 
    ffn = os.path.join(cfgdir, fn)
    try:
      cfgBuffer = list(open(ffn))
    except:
      continue

    gatheringHelpText = False
    for ln in cfgBuffer:
      if gatheringHelpText:
        if reHelpTextEnd.match(ln):
          return helpText
        else:
          helpText.append(ln.strip())
          continue

      m = reHelpTextStart.match(ln)
      if m:
        t = m.groups()
        if len(t) != 1:
          continue

        tl = re.split(' +', t[0])
        if key in tl:
          gatheringHelpText = True

  return [ "Add descriptive text for %s here." % key ]
