TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Determine if it needs to be built: only for RTEMS except RTEMS-uC5282.
BUILD_SIS := NO
ifeq ($(firstword $(subst -, ,$(T_A))),RTEMS)
BUILD_SIS := YES
endif
ifeq ($(T_A),RTEMS-uC5282)
BUILD_SIS := NO
endif
ifeq ($(T_A),RTEMS-mvme2307)
BUILD_SIS := NO
endif

# Conditional build.
ifeq ($(BUILD_SIS),YES)

#==================================================
# build a support library

LIBRARY_IOC += trSIS

DBD += trSIS.dbd

trSIS_SRCS += TR_SIS.cpp

trSIS_LIBS += trCore drvRTEMSDmaSup ADBase asyn $(EPICS_BASE_IOC_LIBS)

#=============================
# Build the IOC application

PROD_IOC = SISTestIoc
# SISTestIoc.dbd will be created and installed
DBD += SISTestIoc.dbd

# SISTestIoc.dbd will be made up from these files:
SISTestIoc_DBD += base.dbd

# Include dbd files from all support applications:
SISTestIoc_DBD += asyn.dbd ADSupport.dbd NDPluginSupport.dbd trSIS.dbd

# Add all the support libraries needed by this IOC
SISTestIoc_LIBS += trSIS drvRTEMSDmaSup trCore NDPlugin ADBase asyn

# SISTestIoc_registerRecordDeviceDriver.cpp derives from SISTestIoc.dbd
SISTestIoc_SRCS += SISTestIoc_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
SISTestIoc_SRCS_DEFAULT += SISTestIocMain.cpp
SISTestIoc_SRCS_vxWorks += -nil-

# Finally link to the EPICS Base libraries
SISTestIoc_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

# End confitional build.
endif

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

