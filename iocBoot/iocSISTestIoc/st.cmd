
# We are currently in the IOC's boot directory, set relative TOP.
epicsEnvSet("TOP", "../..")

## Basic configuration
# Size (NELM) of waveform records.
epicsEnvSet("WFSIZ", "2048")
# Increase CA buffer sizes, needed for larger waveforms.
epicsEnvSet("EPICS_CA_MAX_ARRAY_BYTES", "65535")

## Read thread configuration
# Read thread priority (EPICS units)
epicsEnvSet("READ_THREAD_PRIO", "60")
# Read thread stack size
epicsEnvSet("READ_THREAD_STACK", "0")

## Interrupt thread configuration
# Interrupt thread priority (EPICS units)
epicsEnvSet("INTR_THREAD_PRIO", "90")

## AreaDetector configuration
# Max buffers for the channel port
epicsEnvSet("AD_MAX_BUFS", "32")
# Max memory for the channels port
epicsEnvSet("AD_MAX_MEM", "0")

## Stdarrays plugins configuration.
# Blocking-callbacks for stdarrays ports.
epicsEnvSet("STDAR_BL_CB", "1")
# Queue size for stdarrays plugins (relevant only if blocking-callbacks is 0).
epicsEnvSet("STDAR_QU_SZ", "3")
# Max-memory for stdarrays plugins.
epicsEnvSet("STDAR_MAX_MEM", "-1")
# Thread priority for stdarrays plugins.
epicsEnvSet("STDARR_PRI", "0")
# Thread stack size for stdarrays plugins.
epicsEnvSet("STDAR_ST_SZ", "16384")

## Various customizations.
# Set to # to disable data snapshot waveforms, empty to enable.
epicsEnvSet("SNAP", "")
# Default factors for voltage offset to DAC conversion.
epicsEnvSet("VOLT_EOFF", "7.20770774459041")
epicsEnvSet("VOLT_ESLO", "12.1271382651112")

## Scan configuration.
# SCAN for refreshing device states
epicsEnvSet("STATES_SCAN", "1 second")
# SCAN for refreshing actual channel values
epicsEnvSet("ACTUAL_SCAN", "1 second")
# SCAN for slow snapshot records
epicsEnvSet("SNAP_SCAN", "1 second")

## Register all support components
dbLoadDatabase "${TOP}/dbd/SISTestIoc.dbd"
SISTestIoc_registerRecordDeviceDriver pdbbase

# Initialize the channel ports (generated using gen_devices_and_channels.py).
< "${TOP}/iocBoot/iocSISTestIoc/SISInitDevices.cmd"

# Uncomment to initialize NDAttrPlugin to test hardware timestamps.
# Set <DEVNAM> to the tested device's name.
#NDAttrConfigure("<DEVNAM>_hw_timestamp_attr", 3, 1, "<DEVNAM>_channels", 0, 10, -1, 0, 16384)

# Prefix of our customized DBs.
epicsEnvSet("DB_PREFIX", "TestIoc_")

# Load records for devices and their channels (generated using gen_devices_and_channels.py).
< "${TOP}/iocBoot/iocSISTestIoc/SISLoadDb.cmd"

# Uncomment to load records for NDAttrPlugin waveforms (the .db name is historical).
# Set <DEVNAM> and <PREFIX> to the tested device's name and record prefix, respectively.
#dbLoadRecords("$(TR_CORE)/db/TRSampleRateAttrTest.db", "PREFIX=<PREFIX>, ATTR_PORT=<DEVNAM>_hw_timestamp_attr")

# Initialize IOC.
iocInit
