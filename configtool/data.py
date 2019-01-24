import re
from sys import platform


supportedCPUs = [
    "ATmega168",
    "ATmega328P",
    "ATmega644",
    "ATmega644P",
    "ATmega644PA",
    "ATmega1280",
    "ATmega1284",
    "ATmega1284P",
    "ATmega2560",
    "AT90USB1286",
]

# Note: this is a kludge and works for ATmegas, only. Smaller ATmegas have a
#       lot fewer pins, so this list provides a lot of clutter for them. ARMs
#       have entirely different names. The right way would be to fetch pin
#       names from the compiler environment and/or header files.
pinNames = (
    ["AIO%d" % x for x in range(16)]
    + ["DIO%d" % x for x in range(64)]
    + ["P%c%d" % (c, x) for c in range(ord("A"), ord("L") + 1) for x in range(8)]
)

sensorTypes = {
    "MAX6675": "TT_MAX6675",
    "Thermistor": "TT_THERMISTOR",
    "AD595": "TT_AD595",
    "PT100": "TT_PT100",
    "Intercom": "TT_INTERCOM",
    "MCP3008": "TT_MCP3008",
}

BSIZE = (100, 60)
BSIZESMALL = (90, 30)


if platform.startswith("win"):
    offsetTcLabel = 4
    offsetChLabel = 4
else:
    offsetTcLabel = 6
    offsetChLabel = 8

TYPE_GENERAL = 0
TYPE_FLOAT = 1

reDefQSm = re.compile("\s*#define\s+(\S+)\s+(.*)")
reDefQSm2 = re.compile('\s*("[^"]*")')

reInclude = re.compile('^\s*#include\s+"([^"]*)')
reFloatAttr = re.compile("/\*\s*float\s*\*/")
reDefine = re.compile("\s*#define\s+(\w+)\s+(\S+)")
reDefineBL = re.compile("^\s*#define\s+(\w+)\s+(\S+)")
reDefQS = re.compile('\s*#define\s+(\w+)\s+("[^"]*")')
reDefTS = re.compile("\s*(DEFINE_TEMP_SENSOR\\([^)]*\\))")
reDefHT = re.compile("\s*(DEFINE_HEATER\\([^)]*\\))")
reDefTT = re.compile("^\s*//\s*TEMP_TABLE\s+(\S+)\s+(\\(.*\\))")
reDefBool = re.compile("\s*#define\s+(\w+)\s+")
reDefBoolBL = re.compile("^\s*#define\s+(\w+)\s+")
reStartSensors = re.compile("^\s*//\s*DEFINE_TEMP_SENSORS_START")
reEndSensors = re.compile("^\s*//\s*DEFINE_TEMP_SENSORS_END")
reStartHeaters = re.compile("^\s*//\s*DEFINE_HEATERS_START")
reEndHeaters = re.compile("^\s*//\s*DEFINE_HEATERS_END")
reCandHeatPins = re.compile("^\s*//\s*#define\s+HEATER_PIN\s+(\w+)")
reCandThermPins = re.compile("^\s*//\s*#define\s+TEMP_SENSOR_PIN\s+(\w+)")
reCandProcessors = re.compile("^\s*//\s*#define\s+CPU_TYPE\s+(\w+)")
reCandCPUClocks = re.compile("^\s*//\s*#define\s+F_CPU_OPT\s+(\w+)")

reHelpTextStart = re.compile("^\s*/\*\*\s+\\\\def\s+(.*)")
reHelpTextEnd = re.compile("^\s*\*/")
reHelpText = re.compile("/\*\*.*?\*/\r?\n", re.DOTALL)

reSensor = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")
# reHeater3 and reHeater4 deprecated, for compatibility with old config files only.
reHeater3 = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")
reHeater4 = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")
reHeater5 = re.compile(
    ".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)"
)
reTempTable4 = re.compile(
    ".*\\(\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d+)\s*,\s*(\d*.?\d*)\s*\\)"
)
reTempTable7 = re.compile(
    ".*\\(\s*(\d+)\s*,\s*(\d*.?\d*)\s*,\s*(\d+)\s*,\s*(\d*.?\d*)\s*,\s*(\d+)\s*,\s*(\d*.?\d*)\s*,\s*(\d+)\s*\\)"
)

reInteger = re.compile("^\d+U?L?$")
reFloat = re.compile("^\d+(\.\d*)?$")

defineValueFormat = "#define %-24s %s\n"
defineBoolFormat = "#define %s\n"
defineHeaterFormat = "#define HEATER_%s HEATER_%s\n"
defineDCExtruderFormat = "#define %-24s HEATER_%s\n"

reCandHomingOptions = re.compile("^\s*//\s*#define\s+HOMING_OPT\s+(\w+)")
reDefHoming = re.compile(r"\s*(DEFINE_HOMING\([^)]*\))")
reHoming = re.compile(r".*?\W(\w+)")
