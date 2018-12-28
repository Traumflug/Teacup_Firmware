# Define thermistor presets. These can either be defined to use the
# beta argorithm by specifying 4 parameters:
#    R0, beta, Rp, Vadc
#
# or to use the Steinhart-Hart algorithm by specifying 7 parameters:
#    Rp, T0, R0, T1, R1, T2, R2
#
thermistorPresets = {
    "RS 10K": ["10000", "3480", "1600", "5.0"],
    "RRRF 10K": ["10000", "3964", "1600", "5.0"],
    "ATC Semitec 104GT-2": ["100000", "4267", "4700", "5.0"],
    "EPCOS 100K (B57560G1104F)": ["100000", "4092", "4700", "5.0"],
    "EPCOS 100K (B5754061104)": ["100000", "4066", "4700", "5.0"],
    "EPCOS 100K (B57560G104F)": ["100000", "4036", "4700", "5.0"],
    "Honeywell 100K": ["100000", "3974", "4700", "5.0"],
    "RRRF 100K": ["100000", "3960", "4700", "5.0"],
}
