TOP=../..
include $(TOP)/configure/CONFIG
include $(TR_CORE)/cfg/RULES.customize_pvs
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE

#----------------------------------------------------
#  Optimization of db files using dbst (DEFAULT: NO)
#DB_OPT = YES

#----------------------------------------------------
# Create and install (or just install) into <top>/db
# databases, templates, substitutions like this

# Prefix for customized DBs.
# NOTE: It must not be empty because we install both original
# and customized TRSIS DBs, otherwise the outputs would collide!
TR_CUSTOM_DB_PREFIX := TestIoc_

# These TRCore DBs are used with customized PV names (see below).
DB += $(addprefix $(TR_CUSTOM_DB_PREFIX),$(TRCORE_CUSTOMIZABLE_DBS))

# Install original TRSIS DBs.
TRSIS_DBS := TRSIS.db TRSIS_Channel.db
DB += $(TRSIS_DBS)

# Install customized TRSIS DBs.
DB += $(addprefix $(TR_CUSTOM_DB_PREFIX),$(TRSIS_DBS))

#----------------------------------------------------
# If <anyname>.db template is not named <anyname>*.template add
# <anyname>_template = <templatename>

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

# PV name customization (applies both to TRBASE and TRSIS DBs).
DB_CUSTOMIZE := \
  -r name :Name \
  -r perSecond :Rate \
  -r arm :Arm \
  -r set_arm :SetArm \
  -r autoRestart :AutoRestart \
  -r numberPTS :NumberPTS \
  -r numberPPS :NumberPPS \
  -r TIME_DATA :Time \
  -r DATA Waveform \
  -r DATA_SNAPSHOT WaveformSlow \
  -r DATA_TIMESTAMP TODFst \
  -r DATA_SNAPSHOT_TIMESTAMP TOD \
  -r GET_BURST_END_TO_READ_END_TIME :InterruptProcessingTime \
  -r GET_READ_END_TO_DATA_PROCESSED_TIME :MainProcessingTime \
  -r trigger :Trigger \
  -r softTrigger :SoftTrigger \
  -r numberPTE :NumberPTE \
  -r clock :Clock

# Customizations based on macro substitution.

# ZNAM/ONAM for autoRestart record.
DB_CUSTOMIZE += -m AUTORESTART_ZNAM no -m AUTORESTART_ONAM yes

# Unit for time array (in fractions of second, e.g. 1e6 for microseconds)
# and corresponding EGU field.
DB_CUSTOMIZE += -m TIME_UNIT_INV 1e6 -m TIME_EGU us

# Timestamp format.
DB_CUSTOMIZE += -m TIMESTAMP_FMT "%m/%d/%Y %H:%M:%S.%01f"

# Additional fields in perSecond (rate) record.
DB_CUSTOMIZE += -m PERSECOND_FIELDS "field(HIHI,12) field(LOLO,8) field(LOW,8) field(HHSV,MAJOR) field(LLSV,MAJOR) field(LSV,MINOR) field(DESC, \"Event Rate\")"

# Default settings for acquisition.
DB_CUSTOMIZE += \
  -m DEFAULT_AUTORESTART 1 \
  -m DEFAULT_NUM_BURSTS 0 \
  -m DEFAULT_NUM_PTS 400 \
  -m DEFAULT_NUM_PPS 1000 \
  -m DEFAULT_TRIGGER 1 \
  -m DEFAULT_PREAVERAGE 0 \
  -m DEFAULT_CLOCK 7 \
  -m DEFAULT_EXTRATE 100e6 \
  -m DEFAULT_NUM_PTE 1 \
  -m DEFAULT_EVENTS_BUF_SIZE 1024 \
  -m DEFAULT_TEST_DATA_MODE 0 \
  -m DEFAULT_DAC_VOLTAGE_OFFSET 0.0

# Rules for generating customized DBs (TRBase and TRSIS DBs).
$(eval $(call TRCORE_CUSTOM_DB_RULES,$(TRCORE_DB_DIR),$(TRCORE_CUSTOMIZABLE_DBS),$(TR_CUSTOM_DB_PREFIX),$(DB_CUSTOMIZE)))
$(eval $(call TRCORE_CUSTOM_DB_RULES,..,$(TRSIS_DBS),$(TR_CUSTOM_DB_PREFIX),$(DB_CUSTOMIZE)))
