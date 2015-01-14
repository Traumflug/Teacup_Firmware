
import re


VERSION = "0.1"

supportedCPUs = ['ATmega168', 'ATmega328P', 'ATmega644P', 'ATmega644PA',
                 'ATmega1280', 'ATmega1284P', 'ATmega2560', 'AT90USB1286']

pinNames = ["AIO%d" % x for x in range(16)] + ["DIO%d" % x for x in range(64)]
pinNamesWithBlank = ["-"] + pinNames

BSIZE = (90, 60)
BSIZESMALL = (90, 30)

reDefQSm = re.compile("\s*#define\s+(\S+)\s+(.*)")
reDefQSm2 = re.compile("\s*(\"[^\"]*\")")

reDefine = re.compile("\s*#define\s+(\w+)\s+(\S+)")
reDefineBL = re.compile("^\s*#define\s+(\w+)\s+(\S+)")
reCommDefBL = re.compile("^\s*//\s*#define\s+(\w+)\s+(\S+)")
reDefQS = re.compile("\s*#define\s+(\w+)\s+(\"[^\"]*\")")
reDefTS = re.compile("\s*(DEFINE_TEMP_SENSOR\\([^)]*\\))")
reDefHT = re.compile("\s*(DEFINE_HEATER\\([^)]*\\))")
reDefBool = re.compile("\s*#define\s+(\w+)\s+")
reDefBoolBL = re.compile("^\s*#define\s+(\w+)\s+")
reCommDefBoolBL = re.compile("^\s*//\s*#define\s+(\S+)\s+")
reStartSensors = re.compile("^\s*//\s*DEFINE_TEMP_SENSORS_START")
reEndSensors = re.compile("^\s*//\s*DEFINE_TEMP_SENSORS_END")
reStartHeaters = re.compile("^\s*//\s*DEFINE_HEATERS_START")
reEndHeaters = re.compile("^\s*//\s*DEFINE_HEATERS_END")
reStartProcessors = re.compile("^\s*//\s*PROCESSORS_START")
reEndProcessors = re.compile("^\s*//\s*PROCESSORS_END")
reCandHeatPins = re.compile("^\s*//\s*#define\s+HEATER_PIN\s+(\w+)")
reCandThermPins = re.compile("^\s*//\s*#define\s+TEMP_SENSOR_PIN\s+(\w+)")

reHelpTextStart = re.compile("^\s*/\*\*\s+\\\\def\s+(.*)")
reHelpTextEnd = re.compile("^\s*\*/")

reSensor3 = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")
reSensor4 = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")
reHeater = re.compile(".*\\(\s*(\w+)\s*,\s*(\w+)\s*,\s*(\w+)\s*\\)")

reInteger = re.compile("^\d+U?L?$")
reFloat = re.compile("^\d+(\.\d*)?$")

reAVR = re.compile("__AVR_(\w+)__")

defineValueFormat =      "#define %-24s %s\n"
defineBoolFormat =       "#define %s\n"
defineHeaterFormat =     "#define HEATER_%s HEATER_%s\n"
defineDCExtruderFormat = "#define %-24s HEATER_%s\n"
