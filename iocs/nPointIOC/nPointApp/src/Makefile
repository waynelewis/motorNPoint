TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# Build the IOC application

PROD_IOC = nPoint
# nPoint.dbd will be created and installed
DBD += nPoint.dbd

# nPoint.dbd will be made up from these files:
nPoint_DBD += base.dbd

# Include dbd files from all support applications:
#nPoint_DBD += xxx.dbd

# Add all the support libraries needed by this IOC
#nPoint_LIBS += xxx

# nPoint_registerRecordDeviceDriver.cpp derives from nPoint.dbd
nPoint_SRCS += nPoint_registerRecordDeviceDriver.cpp

# Build the main IOC entry point on workstation OSs.
nPoint_SRCS_DEFAULT += nPointMain.cpp
nPoint_SRCS_vxWorks += -nil-

# Add support from base/src/vxWorks if needed
#nPoint_OBJS_vxWorks += $(EPICS_BASE_BIN)/vxComLibrary

# Finally link to the EPICS Base libraries
nPoint_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

