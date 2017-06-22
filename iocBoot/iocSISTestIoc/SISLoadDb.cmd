# This file was generated by gen_devices_and_channels.py
# Load EPICS records for each device and its channels.

# Device SIS3302
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRBase.db","PREFIX=SIS,PORT=SIS3302,SIZE=$(WFSIZ),PRESAMPLES=,NOCLK=#,LNK_NEW_BURST=SIS:_on_new_burst")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS.db","PREFIX=SIS,PORT=SIS3302,STATES_SCAN=$(STATES_SCAN),ACTUAL_SCAN=$(ACTUAL_SCAN)")

# Channel 0
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS_Channel.db","PREFIX=SIS:CH0,MPREFIX=SIS,PORT=SIS3302,CHANNEL=0,VOLT_EOFF=$(VOLT_EOFF),VOLT_ESLO=$(VOLT_ESLO),ENBLD=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannel.db","PREFIX=SIS:CH0,CHANNELS_PORT=SIS3302_channels,CHANNEL=0")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannelData.db","PREFIX=SIS:CH0,STDAR_PORT=SIS3302_0,SIZE=$(WFSIZ),FTVL=FLOAT,WF_DTYP=asynFloat32ArrayIn,SNAP_SCAN=$(SNAP_SCAN),SNAPSHOT=$(SNAP)")
# Channel 1
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS_Channel.db","PREFIX=SIS:CH1,MPREFIX=SIS,PORT=SIS3302,CHANNEL=1,VOLT_EOFF=$(VOLT_EOFF),VOLT_ESLO=$(VOLT_ESLO),ENBLD=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannel.db","PREFIX=SIS:CH1,CHANNELS_PORT=SIS3302_channels,CHANNEL=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannelData.db","PREFIX=SIS:CH1,STDAR_PORT=SIS3302_1,SIZE=$(WFSIZ),FTVL=FLOAT,WF_DTYP=asynFloat32ArrayIn,SNAP_SCAN=$(SNAP_SCAN),SNAPSHOT=$(SNAP)")
# Channel 2
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS_Channel.db","PREFIX=SIS:CH2,MPREFIX=SIS,PORT=SIS3302,CHANNEL=2,VOLT_EOFF=$(VOLT_EOFF),VOLT_ESLO=$(VOLT_ESLO),ENBLD=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannel.db","PREFIX=SIS:CH2,CHANNELS_PORT=SIS3302_channels,CHANNEL=2")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannelData.db","PREFIX=SIS:CH2,STDAR_PORT=SIS3302_2,SIZE=$(WFSIZ),FTVL=FLOAT,WF_DTYP=asynFloat32ArrayIn,SNAP_SCAN=$(SNAP_SCAN),SNAPSHOT=$(SNAP)")
# Channel 3
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS_Channel.db","PREFIX=SIS:CH3,MPREFIX=SIS,PORT=SIS3302,CHANNEL=3,VOLT_EOFF=$(VOLT_EOFF),VOLT_ESLO=$(VOLT_ESLO),ENBLD=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannel.db","PREFIX=SIS:CH3,CHANNELS_PORT=SIS3302_channels,CHANNEL=3")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannelData.db","PREFIX=SIS:CH3,STDAR_PORT=SIS3302_3,SIZE=$(WFSIZ),FTVL=FLOAT,WF_DTYP=asynFloat32ArrayIn,SNAP_SCAN=$(SNAP_SCAN),SNAPSHOT=$(SNAP)")
# Channel 4
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS_Channel.db","PREFIX=SIS:CH4,MPREFIX=SIS,PORT=SIS3302,CHANNEL=4,VOLT_EOFF=$(VOLT_EOFF),VOLT_ESLO=$(VOLT_ESLO),ENBLD=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannel.db","PREFIX=SIS:CH4,CHANNELS_PORT=SIS3302_channels,CHANNEL=4")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannelData.db","PREFIX=SIS:CH4,STDAR_PORT=SIS3302_4,SIZE=$(WFSIZ),FTVL=FLOAT,WF_DTYP=asynFloat32ArrayIn,SNAP_SCAN=$(SNAP_SCAN),SNAPSHOT=$(SNAP)")
# Channel 5
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS_Channel.db","PREFIX=SIS:CH5,MPREFIX=SIS,PORT=SIS3302,CHANNEL=5,VOLT_EOFF=$(VOLT_EOFF),VOLT_ESLO=$(VOLT_ESLO),ENBLD=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannel.db","PREFIX=SIS:CH5,CHANNELS_PORT=SIS3302_channels,CHANNEL=5")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannelData.db","PREFIX=SIS:CH5,STDAR_PORT=SIS3302_5,SIZE=$(WFSIZ),FTVL=FLOAT,WF_DTYP=asynFloat32ArrayIn,SNAP_SCAN=$(SNAP_SCAN),SNAPSHOT=$(SNAP)")
# Channel 6
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS_Channel.db","PREFIX=SIS:CH6,MPREFIX=SIS,PORT=SIS3302,CHANNEL=6,VOLT_EOFF=$(VOLT_EOFF),VOLT_ESLO=$(VOLT_ESLO),ENBLD=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannel.db","PREFIX=SIS:CH6,CHANNELS_PORT=SIS3302_channels,CHANNEL=6")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannelData.db","PREFIX=SIS:CH6,STDAR_PORT=SIS3302_6,SIZE=$(WFSIZ),FTVL=FLOAT,WF_DTYP=asynFloat32ArrayIn,SNAP_SCAN=$(SNAP_SCAN),SNAPSHOT=$(SNAP)")
# Channel 7
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRSIS_Channel.db","PREFIX=SIS:CH7,MPREFIX=SIS,PORT=SIS3302,CHANNEL=7,VOLT_EOFF=$(VOLT_EOFF),VOLT_ESLO=$(VOLT_ESLO),ENBLD=1")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannel.db","PREFIX=SIS:CH7,CHANNELS_PORT=SIS3302_channels,CHANNEL=7")
dbLoadRecords("$(TOP)/db/$(DB_PREFIX)TRChannelData.db","PREFIX=SIS:CH7,STDAR_PORT=SIS3302_7,SIZE=$(WFSIZ),FTVL=FLOAT,WF_DTYP=asynFloat32ArrayIn,SNAP_SCAN=$(SNAP_SCAN),SNAPSHOT=$(SNAP)")