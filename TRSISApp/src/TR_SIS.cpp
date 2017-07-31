/* This file is part of the SIS Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the SIS Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <time.h>

#include <string>
#include <vector>
#include <algorithm>
#include <new>

#ifdef __rtems__
#include <libcpu/io.h>
#endif

#include <epicsExport.h>
#include <epicsGuard.h>
#include <epicsThread.h>
#include <epicsTimer.h>
#include <epicsInterrupt.h>
#include <epicsEndian.h>
#include <errlog.h>
#include <iocsh.h>
#include <devLib.h>

#include "SIS3302.h"
#include "SIS3302_extra.h"

#include "TR_SIS.h"
#include "TR_SIS_InterruptLock.h"
#include "TR_SIS_DropThreadPriority.h"
#include "TR_SIS_HwConstants.h"

// Define to enable lots of debug messages
//#define SIS_DEBUG 1
//#define SIS_DEBUG_EVENT_INFO 1

// Define to enable sanity checks in interrupt handler
#define DEBUG_INTERRUPT 1

// If set to 1 then use epicsTimeGetCurrentInt to get the time in
// the interrupt handler, the default is to use clock_gettime. 
//#define USE_EPICS_TIME_GET_CURRENT_INT 1

// Constants affecting operation/configuration

// Maximum times to poll when waiting for DAC operation to complete
static const int MaxDacAndResetVerifyRetries = 500;

// How much additional stop delay we use in page wrap mode to ensure no
// samples are lost at the end. Since up to 2 samples may be discared
// at the end, delaying the stop by 2 samples compensates for this.
static const int ExtraStopDelaySamples = 2;

// How many more samples we reserve space for in page wrap mode to avoid
// overwriting pre-trigger data with data at the end of an event. We need
// space for the ExtraStopDelaySamples plus one sample since the device
// may write up to one additional sample after the stop. Note that with
// soft trigger (and pre-trigger samples), the ExtraStopDelaySamples are
// not needed since stop delay will anyway be zero but we still reserve
// them for simplicity.
static const int ExtraWrapPageSpace = ExtraStopDelaySamples + 1;

// Maximum allowed events buffer size.
static const int MaxEventsBufferSize = 262144;

// Thread priority that we drop to while busy polling registers.
static const unsigned int PollPriorityDrop = epicsThreadPriorityLow;

// Constant definitions which must be there to avoid link errors.
const int TRSISDriver::MaxChannelNum;
const uint32_t TRSISDriver::FullBitMask;
const int TRSISDriver::NumConfigParams;

// Check which byte order needs to be configured in the HW
// (same as host byte order).
#if EPICS_BYTE_ORDER == EPICS_ENDIAN_LITTLE
static const bool UseBigEndian = false;
#elif EPICS_BYTE_ORDER == EPICS_ENDIAN_BIG
static const bool UseBigEndian = true;
#else
#error "Cannot determine byte order"
#endif

// Small utility functions / classes

template<typename T>
static void allocateVectorNoCatch(std::vector<T> &target, size_t size)
{
    std::vector<T> tempVector;
    tempVector.resize(size);
    std::swap(target, tempVector);
}

template<typename T>
static bool allocateVector(std::vector<T> &target, size_t size)
{
    try {
        allocateVectorNoCatch(target, size);
    } catch (std::bad_alloc const &) {
        return false;
    }
    return true;
}

// Return the ADC channel data offset in the VME address space
inline uint32_t getChannelVmeOffset(int channelNum)
{
    return SIS3302_ADC1_OFFSET + channelNum * SIS3302_NEXT_ADC_OFFSET;
}

// This class is used as the submit callback, it sets the hardware timestamp.
struct SubmitCallback : public TRArrayCompletionCallback {
    // Set explicitly after construction.
    double hw_time;
    
    // Add the hardware timestamp to the NDArray.
    bool completeArray(NDArray *array)
    {
        array->pAttributeList->add("HW_TIMESTAMP", "HW_TIMESTAMP", NDAttrFloat64, (void *)&hw_time);
        return true;
    }
};

// Functions of the main class

TRSISDriver::TRSISDriver(char const *port_name,
    uint32_t a32offset, int intVec, int intLev, int read_thread_prio,
    int read_thread_stack_size, int max_ad_buffers, size_t max_ad_memory,
    int interrupt_thread_prio_epics)
:
    TRBaseDriver(TRBaseConfig()
        .set(&TRBaseConfig::port_name, std::string(port_name))
        .set(&TRBaseConfig::num_channels, MaxChannelNum)
        .set(&TRBaseConfig::num_config_params, NumConfigParams)
        .set(&TRBaseConfig::num_asyn_params, (int) NUM_ASYN_PARAMS)
        .set(&TRBaseConfig::interface_mask, asynInt32Mask|asynFloat64Mask)
        .set(&TRBaseConfig::interrupt_mask, asynInt32Mask|asynFloat64Mask)
        .set(&TRBaseConfig::read_thread_prio, read_thread_prio)
        .set(&TRBaseConfig::read_thread_stack_size, read_thread_stack_size)
        .set(&TRBaseConfig::max_ad_buffers, max_ad_buffers)
        .set(&TRBaseConfig::max_ad_memory, max_ad_memory)
        .set(&TRBaseConfig::supports_pre_samples, true)
        .set(&TRBaseConfig::update_arrays, false)
    ),
    m_vme_a32_base_address(a32offset),
    m_pci_base_address(NULL),
    m_interrupt_level(intLev),
    m_interrupt_vector(intVec),
    m_interrupt_thread_runnable(this),
    m_interrupt_thread(m_interrupt_thread_runnable,
        (std::string("SISint:") + port_name).c_str(),
        epicsThreadGetStackSize(epicsThreadStackSmall),
        interrupt_thread_prio_epics),
    m_opened(false),
    m_armed(false),
    m_stop_interrupt(false),
    m_prev_hw_timestamp(0),
    m_event_group_id(0),
    m_interrupt_time_broken(false)
{
    char param_name[64];
    
    if (interrupt_thread_prio_epics <= read_thread_prio) {
        errlogSevPrintf(errlogMajor, "WARNING: Interrupt thread priority (%d) "
            "is not greater than read thead priority (%d).",
            interrupt_thread_prio_epics, read_thread_prio);
    }
    
    // Configuration params.
    // The invalid-values of parameters with a discrete set of possible values
    // are set to -1, which represents a "not-available" value. EPICS records
    // exposing armed values are encouraged to follow this convention. For integer
    // parameters which represent actual numeric value, double is used as the
    // effective-value type and the invalid-values are NAN.
    initConfigParam(m_config_triggering_mode,       "TRIGGERING_MODE",       -1);
    initConfigParam(m_config_averaged_samples_log,  "AVERAGED_SAMPLES_LOG",  -1);
    initConfigParam(m_config_clock_source,          "CLOCK_SOURCE",          -1);
    initConfigParam(m_config_external_clock_rate,   "EXTERNAL_CLOCK_RATE",   (double) NAN);
    initConfigParam(m_config_events_per_group,      "EVENTS_PER_GROUP",      (double) NAN);
    initConfigParam(m_config_events_buffer_size,    "EVENTS_BUFFER_SIZE",    (double) NAN);
    initConfigParam(m_config_test_data_mode,        "TEST_DATA_MODE",        -1);
    initConfigParam(m_config_test_data_start,       "TEST_DATA_START",       (double) NAN);
    
    // Internal parameters (defined by this driver).
    initInternalParam(m_config_timestamp_period,    "TIMESTAMP_PERIOD",      (double) NAN);

    // NOTE: All initConfigParam/initInternalParam must be before all createParam
    // so that the parameter index comparison in writeInt32/readInt32 works.

    // Requests
    createParam("SEND_SW_TRIGGER",          asynParamInt32,    &m_reg_params[SEND_SW_TRIGGER]);
    createParam("UPDATE_ACTUAL_VALUES",     asynParamInt32,    &m_reg_params[UPDATE_ACTUAL_VALUES]);
    createParam("UPDATE_STATES",            asynParamInt32,    &m_reg_params[UPDATE_STATES]);
    createParam("GENERAL_RESET",            asynParamInt32,    &m_reg_params[GENERAL_RESET]);
    createParam("CLEAR_TIMESTAMP",          asynParamInt32,    &m_reg_params[CLEAR_TIMESTAMP]);

    // Static information
    createParam("FIRMWARE_MAJOR_REVISION",  asynParamInt32,    &m_reg_params[FIRMWARE_MAJOR_REVISION]);
    createParam("FIRMWARE_MINOR_REVISION",  asynParamInt32,    &m_reg_params[FIRMWARE_MINOR_REVISION]);

    // Dynamic information (states)
    createParam("ACTUAL_EVENTS",            asynParamInt32,    &m_reg_params[ACTUAL_EVENTS]);
    createParam("CLOCK_SOURCE_READBACK",    asynParamInt32,    &m_reg_params[CLOCK_SOURCE_READBACK]);
    createParam("SAMPLING_BUSY",            asynParamInt32,    &m_reg_params[SAMPLING_BUSY]);
    createParam("SAMPLING_LOGIC_ARMED",     asynParamInt32,    &m_reg_params[SAMPLING_LOGIC_ARMED]);
    createParam("STOP_DELAY_READBACK",      asynParamInt32,    &m_reg_params[STOP_DELAY_READBACK]);
    createParam("NEXT_SAMPLE_ADDRESS_CH0",  asynParamInt32,    &m_reg_params[NEXT_SAMPLE_ADDRESS_CH0]);
    createParam("DELAYED_REARMS_COUNT",     asynParamInt32,    &m_reg_params[DELAYED_REARMS_COUNT]);
    
    // Last hardware timestamps
    createParam("LAST_HW_TIME",             asynParamFloat64,  &m_reg_params[LAST_HW_TIME]);
    createParam("LAST_REL_TIME",            asynParamFloat64,  &m_reg_params[LAST_REL_TIME]);
    
    // Channel enabled
    for (int channel = 0; channel < MaxChannelNum; ++channel) {
        ::sprintf(param_name, "CHANNEL_%d_ENABLED", channel);
        createParam(param_name, asynParamInt32, &m_reg_params[CHANNEL_ENABLED_FIRST + channel]);
    }
    
    // Channel actual value
    for (int channel = 0; channel < MaxChannelNum; ++channel) {
        ::sprintf(param_name, "CHANNEL_%d_ACTUAL_VALUE", channel);
        createParam(param_name, asynParamInt32, &m_reg_params[CHANNEL_ACTUAL_VALUE_FIRST + channel]);
    }
    
    // Channel DAC value
    for (int channel = 0; channel < MaxChannelNum; ++channel) {
        ::sprintf(param_name, "CHANNEL_%d_DAC", channel);
        createParam(param_name, asynParamInt32, &m_reg_params[CHANNEL_DAC_FIRST + channel]);
    }
}

bool TRSISDriver::Open()
{
    assert(!m_opened);
    
    do {
        long devStatus;

        // Set up the VME address mapping.
        // We use the asyn port name as the owner name.
        volatile void *pci_base_address;
        devStatus = devRegisterAddress(portName, atVMEA32, m_vme_a32_base_address,
                SIS3302AddressSpaceSize, &pci_base_address);
        if (devStatus != 0) {
            errlogSevPrintf(errlogMajor, "Open device: devRegisterAddress failed.");
            goto cleanup0;
        }

        // Store the base of the address mapping.
        m_pci_base_address = (volatile char *)pci_base_address;

        // Try to read the Module ID register using devReadProbe. This is both
        // to get the module info and verify that the device is accessible.
        uint32_t moduleRegValue;
        devStatus = devReadProbe(4, (volatile char *)m_pci_base_address + SIS3302_MODID, &moduleRegValue);
        if (devStatus != 0) {
            errlogSevPrintf(errlogMajor,
                "Open device: no card at %#.8x (local address %p), devReadProbe failed with error %ld.",
                (unsigned int)m_vme_a32_base_address, (void *)m_pci_base_address, devStatus);
            goto cleanup1;
        }
        
        // Sanity check the Module ID field (expecting 0x3302 as in SIS3302).
        uint32_t moduleId = moduleRegValue >> 16;
        if (moduleId != 0x3302) {
            errlogSevPrintf(errlogMajor, "Open device: Illegal Module ID (%#.4x), aborting.",
                    (unsigned int)moduleId);
            goto cleanup1;
        }
        
        // Reset the digitizer with verification.
        // Do this before setting up interrupts to avoid any spurious interrupts
        // from the digitizer before it was reset.
        if (!resetAndVerify()) {
            errlogSevPrintf(errlogMajor, "Open device: Failed to reset.");
            goto cleanup1;
        }
        
        // Setup the VME interrupt handler.
        devStatus = devConnectInterruptVME(m_interrupt_vector, TRSISDriver::interruptHandlerTrampoline, this);
        if (devStatus != 0) {
            errlogSevPrintf(errlogMajor, "Open device: devConnectInterruptVME failed with error %ld.",
                devStatus);
            goto cleanup1;
        }
        
        // Enable the interrupt handler for the configured level.
        devStatus = devEnableInterruptLevelVME(m_interrupt_level);
        if (devStatus != 0) {
            errlogSevPrintf(errlogMajor, "Open device: devEnableInterruptLevelVME failed with error %ld.",
                devStatus);
            goto cleanup2;
        }

        // Initialize DMA handle, associate interrupt handler
        m_dma_id = rtemsVmeDmaCreate(TRSISDriver::dmaInterruptHandlerTrampoline, this);
        if (m_dma_id == NULL) {
            errlogSevPrintf(errlogMajor, "Open device: rtemsVmeDmaCreate failed.");
            goto cleanup2;
        }
        
        // Update device info registers, set opened to true.
        {
            epicsGuard<asynPortDriver> guard(*this);
            finalizeOpen(moduleRegValue);
        }
        
        // Start interrupt thread.
        m_interrupt_thread.start();
    } while (false);
    
    return true;
    
cleanup2:
    devDisconnectInterruptVME(m_interrupt_vector, TRSISDriver::interruptHandlerTrampoline);
cleanup1:
    devUnregisterAddress(atVMEA32, m_vme_a32_base_address, portName);
cleanup0:
    m_pci_base_address = NULL;
    return false;
}

void TRSISDriver::finalizeOpen(uint32_t moduleRegValue)
{
    // Set device information parameters
    setIntegerParam(m_reg_params[FIRMWARE_MINOR_REVISION], moduleRegValue & 0xFF);
    setIntegerParam(m_reg_params[FIRMWARE_MAJOR_REVISION], (moduleRegValue >> 8) & 0xFF);
    callParamCallbacks();
    
    // Set dititizer name based on module ID.
    char digitizer_name[16];
    ::sprintf(digitizer_name, "SIS%04x", (unsigned int)(moduleRegValue >> 16));
    setDigitizerName(digitizer_name);
    
    // Set opened to allow reading/writing various parameters and arming.
    m_opened = true;
}

asynStatus TRSISDriver::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
    int reason = pasynUser->reason;

    // Apply changes to desired clock source immediately while disarmed
    // because it potentially affects the actual values. If changed while
    // armed, the change will be applied later in onDisarmed.
    if (reason == m_config_clock_source.desiredParamIndex()) {
        // First call the base class to update the desired value.
        asynStatus status = TRBaseDriver::writeInt32(pasynUser, value);
        // Only then call setDesiredClockSource if needed (which reads this value).
        if (status == asynSuccess && !isArmed() && m_opened) {
            if (!setDesiredClockSource()) {
                status = asynError;
            }
        }
        return status;
    }
    
    // If the parameter is not ours or a config parameter, delegate to base class
    if (reason < m_reg_params[SEND_SW_TRIGGER]) {
        return TRBaseDriver::writeInt32(pasynUser, value);
    }
    
    // Check that the device has been opened.
    if (!m_opened) {
        return asynError;
    }
    
    // Handle based on which parameter it is
    if (reason == m_reg_params[SEND_SW_TRIGGER]) {
        return handleSendSoftwareTrigger();
    }
    else if (reason == m_reg_params[UPDATE_ACTUAL_VALUES]) {
        // Update actual sample values for all channels, results can be
        // read from CHANNEL_i_SAMPLE_VALUE parameters.
        return updateActualValues();
    }
    else if (reason == m_reg_params[UPDATE_STATES]) {
        // Update statuses, results can be read from associated parameters.
        return updateStates();
    }
    else if (reason == m_reg_params[GENERAL_RESET]) {
        return handleReset();
    }
    else if (reason == m_reg_params[CLEAR_TIMESTAMP]) {
        return handleClearTimestamp();
    }
    else if (reason >= m_reg_params[CHANNEL_DAC_FIRST] && reason <= m_reg_params[CHANNEL_DAC_LAST]) {
        int channel = reason - m_reg_params[CHANNEL_DAC_FIRST];
        return handleSetChannelDac(channel, value);
    }
    
    // Simple parameter, delegate to asynPortDriver.
    return asynPortDriver::writeInt32(pasynUser, value);
}

asynStatus TRSISDriver::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int reason = pasynUser->reason;

    // If the parameter is not ours or a config parameter, delegate to base class
    if (reason < m_reg_params[SEND_SW_TRIGGER]) {
        return TRBaseDriver::readInt32(pasynUser, value);
    }

    // Check that the device has been opened.
    if (!m_opened) {
        return asynError;
    }
    
    // Simple parameter, delegate to asynPortDriver.
    return asynPortDriver::readInt32(pasynUser, value);
}

asynStatus TRSISDriver::updateActualValues()
{
    assert(m_opened);
    
    for (int channelGroup = 0; channelGroup < MaxChannelNum / 2; ++channelGroup) {
        uint32_t actualValueReg = readRegister(SIS3302_ACTUAL_SAMPLE_VALUE_ADC12 +
            channelGroup * SIS3302ChannelActualValueRegSpacing);
        
        uint16_t loVal = actualValueReg & 0xFFFF;
        uint16_t hiVal = (actualValueReg >> 16) & 0xFFFF;
        
        int paramOffset = CHANNEL_ACTUAL_VALUE_FIRST + channelGroup * 2;
        setIntegerParam(m_reg_params[paramOffset], hiVal);
        setIntegerParam(m_reg_params[paramOffset + 1], loVal);
    }
    
    callParamCallbacks();
    
    return asynSuccess;
}

asynStatus TRSISDriver::updateStates()
{
    assert(m_opened);
    
    uint32_t eventCounter = readRegister(SIS3302_ACTUAL_EVENT_COUNTER);
    setIntegerParam(m_reg_params[ACTUAL_EVENTS], eventCounter);
    
    uint32_t acqControl = readRegister(SIS3302_ACQUISTION_CONTROL);
    setIntegerParam(m_reg_params[CLOCK_SOURCE_READBACK], (acqControl >> AcqControlClockShift) & 0x7);
    setIntegerParam(m_reg_params[SAMPLING_LOGIC_ARMED],  (acqControl >> AcqControlArmedShift) & 0x1);
    setIntegerParam(m_reg_params[SAMPLING_BUSY],         (acqControl >> AcqControlBusyShift)  & 0x1);

    uint32_t stopDelay = readRegister(SIS3302_STOP_DELAY);
    setIntegerParam(m_reg_params[STOP_DELAY_READBACK], stopDelay);
    
    uint32_t nextSampleAddrCh0 = readRegister(SIS3302_ACTUAL_SAMPLE_ADDRESS_ADC1);
    setIntegerParam(m_reg_params[NEXT_SAMPLE_ADDRESS_CH0], nextSampleAddrCh0);
    
    callParamCallbacks();
    
    return asynSuccess;
}

// This does not reset the DAC configuration.
asynStatus TRSISDriver::handleReset()
{
    assert(m_opened);
    
    // Check with TRBaseDriver if we are armed (or arming).
    if (isArmed()) {
        errlogSevPrintf(errlogMajor, "handleReset: Cannot reset while armed!");
        return asynError;
    }

    // Check for successful reset by setting the (arbitrarily chosen)
    // acquisition control register to a non-power-up-default value, and
    // verifying that it returns to the default (0x0) after reset.
    if (!resetAndVerify()) {
        return asynError;
    }
    
    // Restore the desired clock source.
    if (!setDesiredClockSource()) {
        return asynError;
    }
    
    return asynSuccess;
}

asynStatus TRSISDriver::handleClearTimestamp()
{
    assert(m_opened);
    
    // Clear the timestamp counter by writing to the key address.
    writeRegister(SIS3302_KEY_TIMESTAMP_CLR, 1);
    
    return asynSuccess;
}

asynStatus TRSISDriver::handleSetChannelDac(int channel, int value)
{
    assert(m_opened);
    assert(isChannelNumber(channel));
    
    // Check value range.
    if (value < 0 || value > 65535) {
        errlogSevPrintf(errlogMajor, "handleSetChannelDac: Illegal DAC value for channel %d.",
            channel);
        return asynError;
    }
    
    // Remember desired value.
    setIntegerParam(m_reg_params[CHANNEL_DAC_FIRST + channel], value);
    callParamCallbacks();
    
    // Set the DAC.
    if (!setChannelDac(channel, value)) {
        return asynError;
    }
    
    return asynSuccess;
}

bool TRSISDriver::resetAndVerify()
{
    // Set clock to 1MHz.
    uint32_t acq_control_register = (uint32_t)ClockSource1Mhz << AcqControlClockShift;
    if (!commitSetClearRegister(SIS3302_ACQUISTION_CONTROL, acq_control_register, AcqControlClockMask)) {
        errlogSevPrintf(errlogMajor, "resetAndVerify: Error setting pre-reset non-default value of "
                "acquisition control register.");
        return false;
    }
    
    // Issue the reset command.
    writeRegister(SIS3302_KEY_RESET, 1);

    // Check that the Acquisition Control register becomes all zero.
    if (!verifyRegister(SIS3302_ACQUISTION_CONTROL, 0, FullBitMask, MaxDacAndResetVerifyRetries)) {
        uint32_t acq_control_reg = readRegister(SIS3302_ACQUISTION_CONTROL);
        errlogSevPrintf(errlogMajor, "resetAndVerify: Error resetting: acquisition control "
            "readback = %#.8x, should be 0.", (unsigned int)acq_control_reg);
        return false;
    }
    
    return true;
}

bool TRSISDriver::setChannelDac(int channel, uint16_t value)
{
    assert(m_opened);
    assert(isChannelNumber(channel));
    
    // Write desired value
    writeRegister(SIS3302_DAC_DATA, value);
    
    // Bit for this channel in the DAC control/status register.
    uint32_t channel_bit = (uint32_t)channel << DacControlChannelNumShift;
    
    // Load DAC shift register
    if (!commitRegister(SIS3302_DAC_CONTROL_STATUS, DacControlLoadShiftReg|channel_bit,
                        0, DacControlBusy, MaxDacAndResetVerifyRetries)) {
        errlogSevPrintf(errlogMajor, "setChannelDac: Error loading shift register of "
                "channel %d DAC: command timeout.", channel);
        return false;
    }

    // Load DAC
    if (!commitRegister(SIS3302_DAC_CONTROL_STATUS, DacControlLoadDAC|channel_bit,
                        0, DacControlBusy, MaxDacAndResetVerifyRetries))
    {
        errlogSevPrintf(errlogMajor, "setChannelDac: Error loading "
                "channel %d DAC: command timeout.", channel);
        return false;
    }
    
    return true;
}

bool TRSISDriver::setDesiredClockSource()
{
    assert(m_opened);
    
    // Get the desired clock source from the desired-parameter.
    int clock_source = m_config_clock_source.getDesired();
    
    // Sanity check value.
    if (clock_source < ClockSource100Mhz || clock_source > ClockSource2nd100Mhz) {
        errlogSevPrintf(errlogMajor, "setDesiredClockSource: Invalid clock source!");
        return false;
    }
    
    // Change only the clock source in the Acquisition Control regiter.
    uint32_t acq_control_register = (uint32_t)clock_source << AcqControlClockShift;
    if (!commitSetClearRegister(SIS3302_ACQUISTION_CONTROL, acq_control_register, AcqControlClockMask)) {
        errlogSevPrintf(errlogMajor, "setDesiredClockSource: Error setting clock source "
                "in acquisition control register.");
        return false;
    }
    
    return true;
}

void TRSISDriver::writeRegister(uint32_t offset, uint32_t value, bool from_interrupt)
{
    (void)from_interrupt;
    
#if SIS_DEBUG
    if (!from_interrupt) {
        ::printf("Writing value %#.8x to register at offset %#.8x ...\n",
            (unsigned int)value, (unsigned int)offset);
    }
#endif

    volatile char *addr = m_pci_base_address + offset;

#ifdef __rtems__
    out_be32((volatile unsigned *)addr, (int)value);
#else
#error "writeRegister not implemented for this platform"
    *((volatile uint32_t *)addr) = value;
#endif
    
#if SIS_DEBUG
    if (!from_interrupt) {
        ::printf("Success.\n");
    }
#endif
}

uint32_t TRSISDriver::readRegister(uint32_t offset, bool from_interrupt)
{
    (void)from_interrupt;
    
#if SIS_DEBUG
    if (!from_interrupt) {
        ::printf("Reading from register at offset %#.8x ...\n", (unsigned int) offset);
    }
#endif

    volatile char *addr = m_pci_base_address + offset;
    uint32_t value;
    
#ifdef __rtems__
    value = (uint32_t)in_be32((volatile unsigned *)addr);
#else
#error "readRegister not implemented for this platform"
    value = *((volatile uint32_t *)addr);
#endif
    
#if SIS_DEBUG
    if (!from_interrupt) {
        ::printf("Read value %#.8x.\n", (unsigned int) value);
    }
#endif

    return value;
}

void TRSISDriver::ioMemoryBarrier()
{
    // We need a compiler level memory barrier (memory clobber) as well
    // as a CPU instruction which ensures that memory and I/O writes by
    // the CPU are not reordered across this point with respect to each
    // other. For PowerPC, the eieio instruction achieves this.
    // Note that RTEMS has a macro for eieio but which is not sufficient
    // because it does not have the memory clobber.
#if defined(__GNUC__) && defined(__PPC__)
    asm volatile ("eieio\n" : : : "memory");
#else
#error "ioMemoryBarrier not implemented for this compiler/CPU"
#endif
}

void TRSISDriver::dmaMemoryBarrier()
{
    // Ensure ordering with respect to CPU and DMA memory accesses.
    // A compiler level memory barrier is needed as well as a CPU instruction.
    // For PowerPC, the sync instruction (but not eieio) is appropriate.
#if defined(__GNUC__) && defined(__PPC__)
    asm volatile ("sync\n" : : : "memory");
#else
#error "dmaMemoryBarrier not implemented for this compiler/CPU"
#endif
}

bool TRSISDriver::checkSettings(TRArmInfo &arm_info)
{
    // Check that the device has been opened.
    if (!m_opened) {
        return false;
    }
    
    return checkClockAndAveragingSettings(arm_info) &&
           checkEventConfiguration(arm_info) &&
           checkTestDataConfiguration();
}

bool TRSISDriver::checkClockAndAveragingSettings(TRArmInfo &arm_info)
{
    int clock_source = m_config_clock_source.getSnapshot();
    
    // This will be set to the "raw" sample rate, before averaging.
    double raw_sample_rate;
    
    // Set display rate from clock source
    switch (clock_source) {
        case ClockSource100Mhz:
        case ClockSource2nd100Mhz:
            raw_sample_rate = MaxInternalFreq; // 100 MHz
            break;
        case ClockSource50Mhz:
            raw_sample_rate = 50e6;
            break;
        case ClockSource25Mhz:
            raw_sample_rate = 25e6;
            break;
        case ClockSource10Mhz:
            raw_sample_rate = 10e6;
            break;
        case ClockSource1Mhz:
            raw_sample_rate = 1e6;
            break;
        case ClockSourceExternalRandom:
        case ClockSourceExternalSymmetric:
            raw_sample_rate = m_config_external_clock_rate.getSnapshot();
            break;
        default:
            errlogSevPrintf(errlogMajor, "checkSettings: Invalid clock source!");
            return false;
    }

    // Timestamp rate is the raw sample rate except with
    // ClockSourceExternalRandom, see below
    double timestamp_rate = raw_sample_rate;

    // Check consistency of clock type and rate
    if (clock_source == ClockSourceExternalRandom ||
        clock_source == ClockSourceExternalSymmetric)
    {
        if (!(raw_sample_rate > 0)) {
            errlogSevPrintf(errlogMajor, "checkSettings: External clock rate must be positive.");
            return false;
        }

        if (clock_source == ClockSourceExternalRandom) {
            if (raw_sample_rate > MaxInternalFreq) {
                errlogSevPrintf(errlogMinor, "checkSettings: Random external clock rate is above "
                        "the internal clock frequency. The displayed rate will not match "
                        "the actual internal sampling rate of 100 MHz, proceeding anyway.");
            }

            // With external random clock the timestamp rate is 100 MHz
            timestamp_rate = MaxInternalFreq;
        } else {
            if (raw_sample_rate < 1e6) {
                errlogSevPrintf(errlogMinor, "checkSettings: Symmetric external clock rate is below "
                        "the minimum supported value of 1 MHz, sampling may be erratic.");
            }
        }
    } else {
        // External clock rate parameter is irrelevent.
        m_config_external_clock_rate.setIrrelevant();
    }
    
    // Save the timestamp period to the internal configuration parameter.
    m_config_timestamp_period.setSnapshot(1.0 / timestamp_rate);

    // Check averaging mode
    int averagedSamplesLog = m_config_averaged_samples_log.getSnapshot();
    if (averagedSamplesLog < 0 || averagedSamplesLog >= AveragedSamplesLogTooBig) {
        errlogSevPrintf(errlogMajor, "checkSettings: Averaged samples number must be 2^N with N in [0, %d]",
                (int)(AveragedSamplesLogTooBig - 1));
        return false;
    }
    
    // Warn about using averaging with external random clock, because it did not
    // work correctly with our device (HW seemed to ignore external clock and use
    // internal 100 MHz clock instead).
    if (averagedSamplesLog > 0 && clock_source == ClockSourceExternalRandom) {
        errlogSevPrintf(errlogMajor, "checkSettings: WARNING: Averaging and external random"
            " clock has been found not to work together. Proceeding anyway.");
    }

    // Communicate the effective sample rate for display to framework,
    // it is the raw sample rate divided by the number of averaged samples.
    arm_info.rate_for_display = raw_sample_rate / (1 << averagedSamplesLog);
    
    return true;
}

bool TRSISDriver::checkEventConfiguration(TRArmInfo &arm_info)
{
    // Get sample count settings.
    int eventLength = getNumPrePostSamplesSnapshot();
    int postTriggerSamples = getNumPostSamplesSnapshot();
    
    // This is guaranteed by TRBaseDriver.
    assert(eventLength > 0);
    assert(postTriggerSamples >= 0);
    assert(eventLength >= postTriggerSamples);
    
    // Subtract to get number of pre-trigger samples.
    int preTriggerSamples = eventLength - postTriggerSamples;
    
    // Check events buffer size.
    if (eventsBufferSize() < 2) {
        errlogSevPrintf(errlogMajor, "checkSettings: Events buffer size must be at least 2.");
        return false;
    }
    if (eventsBufferSize() > MaxEventsBufferSize) {
        errlogSevPrintf(errlogMajor, "checkSettings: Events buffer size must be less than %d.",
            MaxEventsBufferSize);
        return false;
    }

    // Check events per group.
    int eventsPerGroup = m_config_events_per_group.getSnapshot();
    if (eventsPerGroup < 1) {
        errlogSevPrintf(errlogMajor, "checkSettings: numberPTE is less than 1");
        return false;
    }
    if (eventsPerGroup > eventsBufferSize() - 1) {
        errlogSevPrintf(errlogMajor, "checkSettings: numberPTE is greater than event buffer size - 1");
        return false;
    }
    if (eventsPerGroup > EventDirectorySize) {
        errlogSevPrintf(errlogMajor, "checkSettings: numberPTE is greater than event directory size (%d)",
            EventDirectorySize);
        return false;
    }
    
    // Check triggering mode and determine paging settings...
    TriggeringMode triggeringMode = (TriggeringMode)m_config_triggering_mode.getSnapshot();
    
    // This switch checks basic trigger settings and determines
    // if page wrap is needed (for pre-trigger samples).
    switch (triggeringMode) {
        case TriggerSoft:
            // We cannot have both pre-trigger and post-trigger samples. The
            // framework already guarantees numberPTS >= 0, numberPPS > 0,
            // numberPPS >= numberPTS.
            if (preTriggerSamples > 0) {
                if (postTriggerSamples > 0) {
                    errlogSevPrintf(errlogMajor, "checkSettings: Only pre-trigger "
                            "or only post-trigger operation is supported in soft-trigger mode.");
                    return false;
                }
                // Need page wrap since we will use autostart for pre-trigger samples.
                m_use_paged_acquisition = true;
            } else {
                // Don't need page wrap since we have software start trigger
                // with specific number of post-trigger samples.
                m_use_paged_acquisition = false;
            }
            break;
        case TriggerAutostartFPStop:
            // Need page wrap for pre-trigger samples with autostart.
            m_use_paged_acquisition = true;
            break;
        case TriggerFPStartFPStop:
            // Need page wrap since there is no time limit between start and stop.
            m_use_paged_acquisition = true;
            break;
        case TriggerFPStartAutostop:
            // Don't need page wrap since external start trigger is used
            // with specific number of post-trigger samples.
            m_use_paged_acquisition = false;
            break;
        default:
            errlogSevPrintf(errlogMajor, "checkSettings: Invalid triggering mode!");
            return false;
    }

    // Determine if autostart is needed. This is for all cases where we need
    // page wrap except FPStartFPStop which uses an external start trigger.
    m_autostart = m_use_paged_acquisition && triggeringMode != TriggerFPStartFPStop;

    // First a necessary condition estimate to eliminate the possibility of
    // integer overflow in the next step. A proper check is done after the
    // definition of eventGroupSize below.
    if (eventLength > SampleBufferSize) {
        errlogSevPrintf(errlogMajor, "checkSettings: Event length must be less than"
            " sample buffer size (%d).", (int)SampleBufferSize);
        return false;
    }

    // Determine the page length / event length...
    if (m_use_paged_acquisition) {
        // Determine the extra stop delay needed to avoid truncation. Since stop delay does not
        // apply to KEY_STOP, loss of up to 2 pre-trigger samples with soft-trigger is unavoidable.
        int stopDelayCorrection = (triggeringMode == TriggerSoft) ? 0 : ExtraStopDelaySamples;
        
        // Calculate the stop delay. It is composed of the delay for post-trigger samples
        // followed by the correction determined above, and the whole thing is multiplied by
        // the number of averaged samples since the delay is measured in pre-average samples.
        int averagedSamples = 1 << m_config_averaged_samples_log.getSnapshot();
        double stopDelayDouble = (double)(postTriggerSamples + stopDelayCorrection) * averagedSamples;
        
        // Check that the required stop delay is supported by the hardware.
        if (stopDelayDouble > MaxStopDelay) {
            errlogSevPrintf(errlogMajor, "checkSettings: required stop delay (%f) exceeds "
                    "maximum delay supported by hardware (%u).",
                    stopDelayDouble, (unsigned int)MaxStopDelay);
            return false;
        }
        
        // Remember stop delay to be set in startAcquisition.
        m_stop_delay = stopDelayDouble;

        // Determine the minimum required page size (see the comment at ExtraWrapPageSpace).
        uint32_t correctedEventLength = eventLength + ExtraWrapPageSpace;
        
        // Find the smallest suitable page size (or whole memory).
        if (!findPageLength(correctedEventLength)) {
            errlogSevPrintf(errlogMajor, "checkSettings: Total event size (%u samples) would"
                " exceed sample buffer size (%d samples).", (unsigned int)correctedEventLength,
                (int)SampleBufferSize);
            return false;
        }
    } else {
        // No stop delay is used.
        m_stop_delay = 0;
        
        // We will arm for the number of samples which is the nearest multiple
        // of four greater than or equal to the requested event length.
        // The stop will always be at a 4-sample boundary so no samples will be
        // discarded and this will include all requested samples.
        m_event_or_page_length = (eventLength + 3) & 0xfffffffc;
    }
    
    // Check that there is sufficient space in the sample buffer
    // for an entire event group.
    double eventGroupSize = (double)eventsPerGroup * m_event_or_page_length;
    if (eventGroupSize > SampleBufferSize) {
        errlogSevPrintf(errlogMajor, "checkSettings: Maximum sample count per event group (%f)"
            " would exceed sample buffer size (%d samples).",
            eventGroupSize, (int)SampleBufferSize);
        return false;
    }
    
    // If there is more than one event per group, force the time array to start
    // with zero time and continue in positive increments for the entire event
    // group. This should be less misleading than possibly starting with
    // negative time in case of pre-trigger samples. Note that the default time
    // array calculation is based on getNumPostSamplesSnapshot and
    // getNumPrePostSamplesSnapshot which would only cover the first event, so
    // we have to override this for this case anyway.
    if (eventsPerGroup > 1) {
        arm_info.custom_time_array_calc_inputs = true;
        arm_info.custom_time_array_num_pre_samples = 0;
        arm_info.custom_time_array_num_post_samples = eventsPerGroup * eventLength;
    }

    return true;
}

bool TRSISDriver::checkTestDataConfiguration()
{
    int test_data_mode = m_config_test_data_mode.getSnapshot();
    
    // We calculate the ADC Input Mode register to be written in startAcquisition.
    m_adc_input_mode = 0;
    
    switch (test_data_mode) {
        case TestDataDisabled: {
            // Test-data-start parameter is irrelevent.
            m_config_test_data_start.setIrrelevant();
        } break;
        
        case TestData16bit:
        case TestData32bit: {
            // Enable test data, set 32-bit mode if desired.
            m_adc_input_mode |= AdcInputModeTestDataEnable;
            if (test_data_mode == TestData32bit) {
                m_adc_input_mode |= AdcInputModeTestData32bit;
            }
            
            // Check the test-data-start according to HW restrictions.
            int test_data_start = m_config_test_data_start.getSnapshot();
            if (test_data_start > 0xFFFF) {
                errlogSevPrintf(errlogMajor, "checkSettings: Test Data Start has to be "
                        "less than or equal to 0xFFFF");
                return false;
            }
            if ((test_data_start & 0xFF) >= 0xFE) {
                errlogSevPrintf(errlogMajor, "checkSettings: Test Data Start has to be "
                        "different from 0xYYFE and 0xYYFF");
                return false;
            }
            
            // Add test-data-start to register.
            m_adc_input_mode |= test_data_start;
        } break;
        
        default:
            errlogSevPrintf(errlogMajor, "checkSettings: Invalid test data mode!");
            return false;
    }
    
    return true;
}

bool TRSISDriver::findPageLength(uint32_t eventLength)
{
    // Look for the best page. At the end of this i will point one past the
    // smallest page which is large enough (or will stay at 0 if no pages
    // are large enough).
    int i;
    int pageListLength = sizeof(PageLengths) / sizeof(PageLengths[0]);
    for (i = 0; i < pageListLength; ++i) {
        if (eventLength > PageLengths[i]) {
            break;
        }
    }
    
    if (i == 0) {
        // No suitable page size, check whether whole memory is enough.
        if (eventLength > (uint32_t)SampleBufferSize) {
            return false;
        }
        
        // Use whole memory, we treat it as page wrap mode but
        // we will not actually enable page wrap.
        m_page_code = -1;
        m_event_or_page_length = SampleBufferSize;
    } else {
        // Use one of the page sizes, we will enable page wrap.
        m_page_code = i - 1;
        m_event_or_page_length = PageLengths[m_page_code];
    }
    
    return true;
}

bool TRSISDriver::startAcquisition(bool had_overflow)
{
    assert(!m_armed);
    
    // Overflow cannot occur for this device.
    (void) had_overflow;
    
    int clock_source = m_config_clock_source.getSnapshot();
    int triggering_mode = m_config_triggering_mode.getSnapshot();
    
    // Acquisition control register
    // In addition to settings discernible from below, we disable FP Lemo
    // Timestamp In.
    uint32_t acq_control_register =
        ((uint32_t)clock_source << AcqControlClockShift) |
        (UseBigEndian ? AcqControlBigEndiannessMask : 0) |
        AcqControlMultiEventMask |
        ((triggering_mode != TriggerSoft) ? AcqControlFpStartStopMask : 0) |
        (m_autostart ? AcqControlAutostartMask : 0);
    
    if (!commitSetClearRegister(SIS3302_ACQUISTION_CONTROL, acq_control_register, 0xffff)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting acquisition control register.");
        return false;
    }
    
    // Interrupt configuration register
    uint32_t irq_config_register = IrqConfROAKModeMask | IrqConfIrqEnableMask |
                                   (m_interrupt_level << IrqConfIrqLevelShift) |
                                   (m_interrupt_vector << IrqConfIrqVectorShift);
    
    if (!commitRegister(SIS3302_IRQ_CONFIG, irq_config_register, irq_config_register)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting interrupt configuration.");
        return false;
    }

    // Start delay is not used, write 0 for paranoia's sake
    if (!commitRegister(SIS3302_START_DELAY, 0, 0)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting start delay to 0.");
        return false;
    }

    // Stop delay
    if (!commitRegister(SIS3302_STOP_DELAY, m_stop_delay, m_stop_delay)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting stop delay.");
        return false;
    }

    // Number of events per group
    int events_per_group = m_config_events_per_group.getSnapshot();
    if (!commitRegister(SIS3302_MAX_NOF_EVENT, events_per_group, events_per_group)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting length of event groups.");
        return false;
    }

    // Number of event groups
    m_remaining_bursts = getNumBurstsSnapshot();
    if (m_remaining_bursts == 0) {
        // unlimited, avoid countdown in interrupt handler
        m_remaining_bursts = -1;
    }

    // We will first be reading from ADC Memory Page 0
    m_current_adc_page = 0;
    if (!commitRegister(SIS3302_ADC_MEMORY_PAGE_REGISTER, m_current_adc_page,
                m_current_adc_page, AdcMemPageCodeMask)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting initial ADC memory page.");
        return false;
    }
    
    // Calculate the event configuration register.
    uint32_t average_mode = m_config_averaged_samples_log.getSnapshot();
    uint32_t event_length_stop = !m_use_paged_acquisition;
    uint32_t page_wrap = m_use_paged_acquisition && m_page_code >= 0;
    uint32_t wrap_page_size = page_wrap ? m_page_code : 0;
    uint32_t event_conf_reg = (average_mode      << EventConfAverageModeShift) |
                              (event_length_stop << EventConfEventLengthStopShift) |
                              (page_wrap         << EventConfPageWrapShift) |
                              (wrap_page_size    << EventConfWrapPageSizeShift);
    
    // Write to write-only "all" register, which simultaneously commits the
    // configuration to all channel groups
    writeRegister(SIS3302_EVENT_CONFIG_ALL_ADC, event_conf_reg);

    // Verify commit by reading back the corresponding R/W register of the
    // first channel group.
    if (!verifyRegister(SIS3302_EVENT_CONFIG_ADC12, event_conf_reg)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting event configuration register.");
        return false;
    }

    // Sample length register (similar write-verify sequence as above)
    // Note: this has to be set even in paged mode, i.e., when we are NOT using
    // event length stop mode, to enable correct functioning of the event
    // directory wrap bit. The hardware manual notes the value written to this
    // register has to be 4 less than the desired number of samples.
    // Write to "all" register.
    uint32_t event_len_reg = m_event_or_page_length - EventLengthValueSubtract;
    writeRegister(SIS3302_SAMPLE_LENGTH_ALL_ADC, event_len_reg);
    
    // Verify "ADC12" register
    if (!verifyRegister(SIS3302_SAMPLE_LENGTH_ADC12, event_len_reg)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting event length register.");
        return false;
    }

    // Begin storing samples at the beginning of the buffer, verification logic as above
    writeRegister(SIS3302_SAMPLE_START_ADDRESS_ALL_ADC, 0);
    if (!verifyRegister(SIS3302_SAMPLE_START_ADDRESS_ADC12, 0)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting sample start address register.");
        return false;
    }
    
    // Write ADC input mode register (test data configuration).
    writeRegister(SIS3302_ADC_INPUT_MODE_ALL_ADC, m_adc_input_mode);
    if (!verifyRegister(SIS3302_ADC_INPUT_MODE_ADC12, m_adc_input_mode)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting ADC input mode register.");
        return false;
    }

    // Turn on LED
    if (!commitSetClearRegister(SIS3302_CONTROL_STATUS, CntlStatusUserLed, CntlStatusUserLed)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error turning on User LED.");
        return false;
    }

    // Reset delayed rearms count before enabling interrupts
    m_delayed_rearms_count = 0;
    {
        epicsGuard<asynPortDriver> guard(*this);
        setIntegerParam(m_reg_params[DELAYED_REARMS_COUNT], m_delayed_rearms_count);
        callParamCallbacks();
    }
    
    // Clear all IRQ sources. This prevents spurious interrupts due to
    // previously latched interrupt conditions.
    if (!commitSetClearRegister(SIS3302_IRQ_CONTROL, 0, IrqControlSourcesMask)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting interrupt control.");
        return false;
    }
    
    // Enable IRQ source 1 (end of last event in multi event mode).
    // Only the low 8 bits are configuration we care about here.
    if (!commitSetClearRegister(SIS3302_IRQ_CONTROL, IrqControlSourceEndLastEvent, IrqControlSourcesMask)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Error setting interrupt control.");
        return false;
    }
    
    // Reset states.
    m_disarm = false;
    m_need_rearm = false;
    m_buffer_start_pointer = 0;
    m_events_start = 0;
    m_events_end = 0;

    // Allocate the sample buffer
    if (!allocateVector(m_sample_vector, events_per_group * m_event_or_page_length)) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Sample buffer allocation failed.");
        goto cleanup;
    }
    
    // Allocate the event info buffer
    if (!allocateVector(m_event_infos, eventsBufferSize())) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Event info buffer allocation failed.");
        goto cleanup;
    }
    
    // Allocate the processed event info buffer (for page wrap mode only)
    if (m_use_paged_acquisition) {
        if (!allocateVector(m_procd_event_infos, events_per_group)) {
            errlogSevPrintf(errlogMajor,
                    "startAcquisition: Processed event info buffer allocation failed.");
            goto cleanup;
        }
    }
    
    // Set m_armed to true before arming sampling logic to avoid a race condition
    // if an interrupt arrives very soon. This also enables handling SW triggers
    // if applicable.
    setArmed(true);
    
    // Barrier to ensure that relevant memory writes complete before the device
    // receives the arm request below.
    ioMemoryBarrier();
    
    // Arm the sampling logic.
    // NOTE: We don't try to verify this by reading the Acquisition Control register
    // because event(s) might already have been completed before we checked.
    writeRegister(SIS3302_KEY_ARM, 1);

    return true;

cleanup:
    if (!releaseAllVectors()) {
        errlogSevPrintf(errlogMajor, "startAcquisition: Additional error deallocating vectors.");
    }
    return false;
}

bool TRSISDriver::releaseAllVectors()
{
    try {
        allocateVectorNoCatch(m_sample_vector, 0);
        allocateVectorNoCatch(m_event_infos, 0);
        allocateVectorNoCatch(m_procd_event_infos, 0);
    } catch (std::bad_alloc const &) {
        return false;
    }
    return true;
}

void TRSISDriver::interruptReading ()
{
    // Set m_disarm to true and signal the event.
    // This way readBurst will soon be woken up if it was waiting,
    // will find m_disarm to be true and will return.
    m_disarm = true;
    m_interrupt_event.signal();
}

void TRSISDriver::stopAcquisition()
{
    // Disable interrupt sources.
    if (!commitSetClearRegister(SIS3302_IRQ_CONTROL, 0, IrqControlSourcesMask)) {
        errlogSevPrintf(errlogMajor, "stopAcquisition: Error setting interrupt control.");
    }
    
    // Sleep a short time so that any interrupts that are pending in hardware
    // are handled by interruptHandler.
    epicsThreadSleep(0.001);
    
    // Synchronize with interruptThread. We want to ensure that handleSingleInterrupt
    // does not run again past this point, to avoid problems if acquisition is restarted
    // while handleSingleInterrupt is running.
    synchronizeStopWithInterruptThread();
    
    // Clear m_armed to ignore any further interrupts and SW triggers.
    // This is done before disarming the sampling logic to ensure that
    // the interrupt handler doesn't mistake the interrupt for a normal
    // end of an event group.
    setArmed(false);
    
    // Barrier to ensure that relevant memory writes complete before the device
    // receives the disarm request below, so that any interrupt caused by disarming
    // will surely see m_armed to be false.
    ioMemoryBarrier();
    
    // Disarm the sampling logic.
    writeRegister(SIS3302_KEY_DISARM, 1);

    // Clear the Acquisition control register but not the clock
    uint32_t no_clock_mask = 0xffff & ~AcqControlClockMask;
    if (!commitSetClearRegister(SIS3302_ACQUISTION_CONTROL, 0, no_clock_mask)) {
        errlogSevPrintf(errlogMajor, "stopAcquisition: Error setting acquisition control register.");
    }
    
    // Turn off LED
    if (!commitSetClearRegister(SIS3302_CONTROL_STATUS, 0, CntlStatusUserLed)) {
        errlogSevPrintf(errlogMajor, "stopAcquisition: Error turning off User LED.");
    }

    // Release internal buffers
    if (!releaseAllVectors()) {
        errlogSevPrintf(errlogMajor, "stopAcquisition: Error deallocating vectors.");
    }
}

void TRSISDriver::synchronizeStopWithInterruptThread()
{
    // If we use only raw interrupts (currently not implemented),
    // this synchronization is not needed.
    if (interruptLockArg() == NULL) {
        return;
    }
    
    // m_stop_interrupt is only set to true here and is cleared by
    // the time we're done, so it must be false when we get here.
    assert(!m_stop_interrupt);
    
    // Lock to protect access to m_stop_interrupt.
    epicsGuard<epicsMutex> lock(m_interrupt_mutex);
    
    // Issue a stop request to the interruptThread by setting the flag
    // and signaling the event (to make sure the interruptThread finds
    // the flag to be set).
    m_stop_interrupt = true;
    m_interrupt_handler_event.signal();
    
    // NOTE: It is important that we signal the event before unlocking the mutex
    // because we rely on the interruptThread to clear the event as part of the
    // stop request. If we signaled the event later that could be after the stop
    // request was already handled and the goal of preventing additional
    // handleSingleInterrupt calls would not be achieved.
    
    // Wait until the interrupt thread acknowledges the stop request.
    do {
        epicsGuardRelease<epicsMutex> unlock(lock);
        m_interrupt_event.wait();
    } while (m_stop_interrupt);
}

void TRSISDriver::onDisarmed()
{
    // Make sure the desired clock source is applied to hardware, since if it
    // was changed while armed then writeInt32 did not apply it at that time.
    if (m_opened) {
        setDesiredClockSource();
    }
}

void TRSISDriver::setArmed(bool value)
{
    // First port lock then interrupt lock!
    epicsGuard<asynPortDriver> guard(*this);
    TR_SIS_InterruptLock lock(interruptLockArg());
    m_armed = value;
}

void TRSISDriver::interruptHandlerTrampoline(void *arg)
{
    static_cast<TRSISDriver *>(arg)->interruptHandler();
}

void TRSISDriver::dmaInterruptHandlerTrampoline(void *arg)
{
    static_cast<TRSISDriver *>(arg)->dmaInterruptHandler();
}

void TRSISDriver::interruptHandler()
{
    // Just signal the event for interruptThread to pick up.
    // We do not need any acknowledgement here because ROAK is used.
    m_interrupt_handler_event.signal();
}

void TRSISDriver::interruptThread()
{
    while (true) {
        // Wait for an interrupt.
        m_interrupt_handler_event.wait();
        
        // Handle the interrupt.
        handleSingleInterrupt(&m_interrupt_mutex);
    }
}

void TRSISDriver::handleSingleInterrupt(epicsMutex *non_interrupt_mutex)
{
    // NOTE: This function is written to support being called both directly from
    // interrupt context as well as from a thread. The argument non_interrupt_mutex
    // tells us which is the case: if it is null we are in a real interrupt handler,
    // otherwise we are in a thread and we use that mutex for locking.
    
    // Disable and clear the interrupt.
    // This is important because if we didn't do it, it would remain
    // latched and another interrupt couldn't be generated.
    writeRegister(SIS3302_IRQ_CONTROL, IrqControlSourceEndLastEvent << 16, true);
    
    // Read out some variables safely (ones that may be updated
    // from the read thread), clear m_stop_interrupt at the same time.
    bool stop_interrupt;
    bool armed;
    SampleRingPointer buffer_start_pointer;
    EventRingPointer events_start;
    {
        TR_SIS_InterruptSideLock lock(non_interrupt_mutex);
        stop_interrupt = m_stop_interrupt;
        m_stop_interrupt = false;
        armed = m_armed;
        buffer_start_pointer = m_buffer_start_pointer;
        events_start = m_events_start;
    }
    
    if (stop_interrupt) {
        // stopAcquisition (synchronizeStopWithInterruptThread) is trying to
        // synchronize with the interrupt thread. We have already cleared
        // m_stop_interrupt to acknowledge the request.
        
        // Clear m_interrupt_handler_event to prevent additional calls of
        // handleSingleInterrupt until the next acquisition is started. Note that
        // stopAcquisition already disabled interrupts in HW and slept a bit
        // therefore there should be no spurious raw interruptHandler calls.
        m_interrupt_handler_event.tryWait();
        
        // Signal the event to make sure stopAcquisition wakes up to find
        // m_stop_interrupt being false.
        m_interrupt_event.signal();
        
        // Don't need to do anything else since data is not being read anymore.
        return;
    }
    
    if (!armed) {
        // NOTE: This actually can occur if the interrupt was generated
        // shortly before we disabled interrupt sources in stopAcquisition.
        // But it's better to have the remote possibility of spurious messages
        // than to not know when there is a real problem with unexpected
        // interrupts.
        epicsInterruptContextMessage("SIS3302: Interrupt while not armed\n");
        return;
    }
    
#if DEBUG_INTERRUPT
    // Sanity check that the sampling logic is disarmed.
    uint32_t acqControl = readRegister(SIS3302_ACQUISTION_CONTROL, true);
    if ((acqControl & AcqControlArmedMask) != 0) {
        epicsInterruptContextMessage("SIS3302 IH ERROR: Sampling logic armed!\n");
        return;
    }
    
    // Expecting m_need_rearm to be false.
    if (m_need_rearm) {
        epicsInterruptContextMessage("SIS3302 IH ERROR: m_need_rearm is true!\n");
        return;
    }
#endif
    
    // Read these variables which do not need to be protected since they
    // are not updated from the read thread while acquisition is active.
    int eventsPerGroup = m_config_events_per_group.getSnapshotFast();
    int events_size = eventsBufferSize();
    EventRingPointer events_end = m_events_end;
    
#if DEBUG_INTERRUPT
    // Sanity check events buffer size.
    if (events_size < 2 || events_size > MaxEventsBufferSize) {
        epicsInterruptContextMessage("SIS3302 IH ERROR: events_size insane\n");
        return;
    }
    
    // Sanity check eventsPerGroup.
    if (eventsPerGroup < 1 || eventsPerGroup > events_size - 1 ||
        eventsPerGroup > EventDirectorySize)
    {
        epicsInterruptContextMessage("SIS3302 IH ERROR: eventsPerGroup insane\n");
        return;
    }
    
    // Sanity check buffer_start_pointer.
    if (!buffer_start_pointer.isValid(SampleBufferSize)) {
        epicsInterruptContextMessage("SIS3302 IH ERROR: buffer_start_pointer insane\n");
        return;
    }
    
    // Sanity check ring buffer pointers.
    if (!events_start.isValid(events_size)) {
        epicsInterruptContextMessage("SIS3302 IH ERROR: events_start insane\n");
        return;
    }
    if (!events_end.isValid(events_size)) {
        epicsInterruptContextMessage("SIS3302 IH ERROR: events_end insane\n");
        return;
    }
#endif

    // Get the current time, put it into the buffer element for the first event.
    epicsTimeStamp *timeStampPtr = &m_event_infos[events_end].epics_timestamp;
#if USE_EPICS_TIME_GET_CURRENT_INT
    if (epicsTimeGetCurrentInt(timeStampPtr) != epicsTimeOK)
#else
    struct timespec tspec;
    if (::clock_gettime(CLOCK_REALTIME, &tspec) != 0 ||
        epicsTimeFromTimespec(timeStampPtr, &tspec) != epicsTimeOK)
#endif
    {
        // Set timestamp to zero to indicate no information.
        timeStampPtr->secPastEpoch = 0;
        timeStampPtr->nsec = 0;
    }
    
    // Calculate how much space there is in the events buffer. This correctly
    // handles the edge case events_end==events_start where the entire buffer
    // and it also takes into account that the element before ring_start
    // cannot be used.
    int ring_space = events_end.ringSub(events_start, events_size).ringComplement(events_size);
    
#if DEBUG_INTERRUPT
    // Sanity check that there is enough space for this event group.
    // When we armed there was enough space so there must still be.
    if (ring_space < eventsPerGroup) {
        epicsInterruptContextMessage("SIS3302 IH ERROR: ring_space < eventsPerGroup\n");
        return;
    }
#endif
    
    // Read out and remember the event directory entries and timestamps.
    // We cannot do that from the read loop because the next event
    // will store these at the same locations. It is unfortunate that
    // the hardware does not support setting a specific start address
    // for these things like it does for data.
    
    uint32_t last_event_stop;
    
    int rel_event_idx = 0;
    do {
        // Read and store the stop pointer.
        uint32_t eventDirOffset = rel_event_idx * (uint32_t)4;
        last_event_stop = readRegister(SIS3302_EVENT_DIRECTORY_ADC1 + eventDirOffset, true);
        m_event_infos[events_end].event_stop_pointer = last_event_stop;
        
        // Read and store the timestamp. Timestamps are only used for
        // the first event but we still read them for simplicity.
        uint32_t timestampOffset = eventDirOffset * 2;
        m_event_infos[events_end].event_timestamp_high =
            readRegister(SIS3302_TIMESTAMP_DIRECTORY + timestampOffset, true);
        m_event_infos[events_end].event_timestamp_low =
            readRegister(SIS3302_TIMESTAMP_DIRECTORY + timestampOffset + 4, true);
        
        // Increment events_end.
        events_end.ringInc(events_size);
    } while (++rel_event_idx < eventsPerGroup);
    
    // Update ring_space according to events just written.
    ring_space -= eventsPerGroup;

    // Decrement remaining burst number if applicable (negative means unlimited).
    if (m_remaining_bursts > 0) {
        --m_remaining_bursts;
    }
    
    // Consider rearming if this isn't the last requested burst
    bool could_not_rearm = false;
    if (m_remaining_bursts != 0) {
        // If there isn't enough space in the sample buffer, the read thread
        // must rearm after processing some data
        if (!rearmIfSpaceAvailable(ring_space, eventsPerGroup, buffer_start_pointer,
                                   last_event_stop, true))
        {
            could_not_rearm = true;
        }
    }
    
    // NOTE: There is no problem that we might have rearmed before updating
    // variables below, since this function cannot run again before it returns.
    
    // Update variables, under a lock because other threads read them.
    {
        TR_SIS_InterruptSideLock lock(non_interrupt_mutex);
        m_events_end = events_end;
        if (could_not_rearm) {
            m_need_rearm = true;
            m_delayed_rearms_count = (m_delayed_rearms_count == INT_MAX) ? 0 :
                m_delayed_rearms_count + 1;
        }
    }
    
    // Signal the event to let the readBurst wait for events.
    m_interrupt_event.signal();
}

void TRSISDriver::dmaInterruptHandler()
{
    // Set the dma event to let the read thread know DMA is done.
    m_dma_event.signal();
}

bool TRSISDriver::rearmIfSpaceAvailable(
    int ring_space, int eventsPerGroup, SampleRingPointer buffer_start_pointer,
    uint32_t last_event_stop, bool from_interrupt)
{
    // First check if there is space available in the events buffer.
    if (ring_space < eventsPerGroup) {
#if SIS_DEBUG
        if (!from_interrupt) {
            errlogSevPrintf(errlogMajor, "rearmIfSpaceAvailable: ring_space=%d < eventsPerGroup=%d",
                    ring_space, eventsPerGroup);
        }
#endif
        return false;
    }
    
    uint32_t event_or_page_length = m_event_or_page_length;
    
    // Compute the start pointer for the next event.
    // First extract the stop pointer bits from the event directory entry.
    SampleRingPointer buffer_end_pointer = last_event_stop & EventDirStopPointerMask;
    if (m_use_paged_acquisition) {
        // In page wrap mode, also move to the start of the next page.
        buffer_end_pointer = getNextPageStart(buffer_end_pointer, event_or_page_length) % SampleBufferSize;
    }

    // Calculate how many more samples can be written to the sample buffer
    // without overwriting existing samples. We need different handling of
    // the start==end condition:
    // - From the interrupt handler, start==end means that the buffer is full,
    //   since something has just been written (and not yet read out since).
    // - From the read thread, start==end means that the buffer is empty,
    //   since something has just been read out (and nothing has later been
    //   written because the digitizer could not have been rearmed).
    int32_t sample_space_left;
    if (from_interrupt) {
        sample_space_left = buffer_start_pointer.ringSub(buffer_end_pointer, SampleBufferSize);
    } else {
        sample_space_left = SampleBufferSize - buffer_end_pointer.ringSub(buffer_start_pointer, SampleBufferSize);
    }
    
    // Check if there is space for an entire event group.
    if ((uint32_t)sample_space_left < eventsPerGroup * event_or_page_length) {
#if SIS_DEBUG
        if (!from_interrupt) {
            errlogSevPrintf(errlogMajor, "rearmIfSpaceAvailable: ssl = (bsp = %d) - (bep = %d) = %d < "
                    "(epg = %d) * (meopl = %u). paged_acq = %d", (int) buffer_start_pointer,
                    (int) buffer_end_pointer, (int) sample_space_left, eventsPerGroup,
                    (unsigned int)event_or_page_length, (int)m_use_paged_acquisition);
        }
#endif
        return false;
    }

    // If we are rearming from the read thead, set need_rearm back to false.
    // This is essential to prevent needless rearming from the read thread,
    // and also because the interrupt handler has a sanity check.
    if (!from_interrupt) {
        m_need_rearm = false;
    }
    
    // Enable the interrupt, set sample start address.
    writeRegister(SIS3302_IRQ_CONTROL, IrqControlSourceEndLastEvent, from_interrupt);
    writeRegister(SIS3302_SAMPLE_START_ADDRESS_ALL_ADC, buffer_end_pointer, from_interrupt);
    
    // Barrier to ensure that relevant memory writes complete before the device
    // receives the arm request below, so that another interrupt handler will
    // surely see updated variables.
    ioMemoryBarrier();
    
    // Rearm.
    writeRegister(SIS3302_KEY_ARM, 1, from_interrupt);
    
    return true;
}

bool TRSISDriver::verifyRegister(uint32_t offset, uint32_t desiredMaskedReadbackValue,
        uint32_t readbackComparisonMask, int maxRetries)
{
    bool first = true;
    TR_SIS_DropThreadPriority dropThreadPriority;
    
    while (true) {
        uint32_t readback = readRegister(offset);

        if ((readback & readbackComparisonMask) == desiredMaskedReadbackValue) {
            return true;
        }
        
        if (maxRetries <= 0) {
            return false;
        }
        
        // When retrying for the first time, drop the thread priority.
        // We must not call dropPriority more than once hence the flag.
        // The destructor will restore the priority.
        if (first) {
            first = false;
            dropThreadPriority.dropPriority(PollPriorityDrop);
        }

        maxRetries--;
    }
}

bool TRSISDriver::commitRegister(uint32_t offset, uint32_t commitValue, uint32_t desiredMaskedReadbackValue,
        uint32_t readbackComparisonMask, int maxRetries)
{
    writeRegister(offset, commitValue);
    return verifyRegister(offset, desiredMaskedReadbackValue, readbackComparisonMask, maxRetries);
}

bool TRSISDriver::commitSetClearRegister(uint32_t offset, uint32_t value, uint32_t mask)
{
    // Set bits in register by writing ones in correspinding low 16 bits
    // and clear bits by writing ones in corresponding high 16 bits.
    uint32_t writeValue = (value & mask) | ((~value & mask) << 16);
    writeRegister(offset, writeValue);
    return verifyRegister(offset, value & mask, mask);
}

asynStatus TRSISDriver::handleSendSoftwareTrigger()
{
    // Only process when armed and in TriggerSoft mode.
    if (m_armed) {
        int triggering_mode = m_config_triggering_mode.getSnapshot();
        int postTriggerSamples = getNumPostSamplesSnapshot();
        
        // Determine whether to send a STOP or START command
        if (triggering_mode == TriggerSoft) {
            uint32_t key_addr = (postTriggerSamples == 0) ? SIS3302_KEY_STOP : SIS3302_KEY_START;
            writeRegister(key_addr, 1);
        }
    }
    
    return asynSuccess;
}

bool TRSISDriver::readBurst()
{
    assert(m_armed);
    
    // The events start pointer is not written by interruptHandler
    // (only read), so we can read it without concern.
    EventRingPointer events_start = m_events_start;
    assert(events_start.isValid(eventsBufferSize()));
    
    // Wait for events to process (or disarming).
    if (!waitForEvents(events_start)) {
        // We are being disarmed.
        return false;
    }

    // Get the current time for diagnostic info.
    epicsTimeStamp readBurstStartTimeTs;
    epicsTimeGetCurrent(&readBurstStartTimeTs);

    int eventsPerGroup = m_config_events_per_group.getSnapshotFast();
    
    // Calculate channel-independent quantities pertaining to the entire
    // event group.

    // Increment the event group ID.
    m_event_group_id = (m_event_group_id == INT_MAX) ? 0 : (m_event_group_id + 1);

    // Hardware timing information
    uint64_t hw_timestamp = m_event_infos[events_start].event_timestamp_low |
                 ((uint64_t)m_event_infos[events_start].event_timestamp_high << 32);

    // Initialize the submit callback class.
    SubmitCallback submit_callback;
    submit_callback.hw_time = hw_timestamp;
    
    // Get the usual timestamps for the NDArray. If the interrupt timestamp
    // equals zero, it was not obtained successfully, and the current time will
    // be used instead.
    epicsTimeStamp interruptEpicsTs = m_event_infos[events_start].epics_timestamp;
    if (interruptTimeBroken(interruptEpicsTs)) {
        interruptEpicsTs = readBurstStartTimeTs;
    }
    double interruptTime = getTimeDouble(&interruptEpicsTs);

    // Get some settings.
    unsigned int numPPS = getNumPrePostSamplesSnapshot();
    bool softTrigger = (m_config_triggering_mode.getSnapshotFast() == TriggerSoft);

    // Store channel-independent information for every event in current event group
    if (m_use_paged_acquisition) {
        for (int event = 0; event < eventsPerGroup; ++event) {
            EventRingPointer eventIndex = events_start.ringAdd(event, eventsBufferSize());
            ProcdEventInfo &eventInfo = m_procd_event_infos[event];
            eventInfo = calculateEventAlignment(m_event_infos[eventIndex], softTrigger);
#if SIS_DEBUG_EVENT_INFO
            ::printf("Wrap bit = %d, stop_pointer < numPPS = %d, discarded samples = %d\n",
                eventInfo.has_wrapped, eventInfo.stop_pointer < numPPS,
                eventInfo.discarded_samples);
#endif
        }
    }

    // Process one channel at a time
    for (int channel = 0; channel < MaxChannelNum; ++channel) {
        // Only process if the channel is enabled
        if (!channelEnabledForRead(channel)) {
            continue;
        }

        // Read the channel data into a local buffer
        if (!doDmaForChannel(channel, m_buffer_start_pointer, m_sample_vector.data(),
                    m_event_or_page_length * eventsPerGroup)) {
            // Error messages printed in underlying functions
            return false;
        }

        // Allocate an NDArray for the data (via TRChannelDataSubmit).
        TRChannelDataSubmit dataSubmit;
        dataSubmit.allocateArray(*this, channel, NDFloat32, eventsPerGroup * numPPS);
        float *outData = (float *)dataSubmit.data();
        if (outData == NULL) {
            errlogSevPrintf(errlogMajor, "readBurst: Could not allocate NDArray.");
            return false;
        }

        // Copy the data from the local buffer to the NDArray.
        // We will increment outData by numPPS for each event copied.
        
        if (!m_use_paged_acquisition) {
            // No page wrapping is used.
            // Each event in the sample buffer is m_event_or_page_length long
            // and we need to copy them separately because that may be greater
            // than numPPS (due to being rounded up to 4 samples).
            for (int event = 0; event < eventsPerGroup; ++event) {
                copySamplesToFloats(&m_sample_vector[event * m_event_or_page_length],
                                    outData, numPPS);
                outData += numPPS;
            }
        } else {
            // Page wrapping is used.
            
            unsigned int inputPageStart = 0;
            
            for (int event = 0; event < eventsPerGroup; ++event) {
                // Get the precalculated event information.
                const ProcdEventInfo &eventInfo = m_procd_event_infos[event];

                // Check whether the event group wrapped (or is truncated at the beginnning)
                if (eventInfo.stop_pointer >= numPPS) {
                    // All of the samples are present in the page and in the correct order
                    copySamplesToFloats(
                        &m_sample_vector[inputPageStart + eventInfo.stop_pointer - numPPS],
                        outData, numPPS);
                } else {
                    // How many of the numPPS samples are at the end of the page before wrapping.
                    int preWrapSamples = numPPS - eventInfo.stop_pointer;
                    
                    // We have wrapped or truncation has occured.
                    if (eventInfo.has_wrapped) {
                        copySamplesToFloats(
                            &m_sample_vector[inputPageStart + m_event_or_page_length - preWrapSamples],
                            outData, preWrapSamples);
                    } else {
                        // Set the first preWrapSamples of the output array to NaN
                        fillFloatsWithNan(outData, preWrapSamples);
                    }
                    
                    // Copy the remaining samples from the beginning of the page
                    copySamplesToFloats(
                        &m_sample_vector[inputPageStart],
                        outData + preWrapSamples, eventInfo.stop_pointer);
                }

                // Set any discarded samples at the end to NaN.
                int discardedSamples = std::min(eventInfo.discarded_samples, (int)numPPS);

                fillFloatsWithNan(&outData[numPPS  - discardedSamples], discardedSamples);
                
                // Increment input and output pointers.
                inputPageStart += m_event_or_page_length;
                outData += numPPS;
            }
        }

        // Submit the channel data to the framework.
        dataSubmit.submit(*this, channel, m_event_group_id,
                          interruptTime, interruptEpicsTs, &submit_callback);
    }

    uint32_t data_end_pointer = m_buffer_start_pointer + m_event_or_page_length * eventsPerGroup;
    
    // Increment the events and buffer start pointers (locally).
    events_start = events_start.ringAdd(eventsPerGroup, eventsBufferSize());
    SampleRingPointer buffer_start_pointer = data_end_pointer % SampleBufferSize;
    
    // Write and read variables safely.
    bool need_rearm;
    int delayed_rearms_count;
    EventRingPointer events_end;
    {
        TR_SIS_InterruptLock lock(interruptLockArg());
        m_events_start = events_start;
        m_buffer_start_pointer = buffer_start_pointer;
        need_rearm = m_need_rearm;
        delayed_rearms_count = m_delayed_rearms_count;
        events_end = m_events_end;
    }
    
    // If the interrupt handler did not rearm the digitizer because
    // there was not enough space in buffers, rearm it now if possible.
    if (need_rearm) {
        // Retrieve the stop pointer of the last event we processed. There is no
        // risk of this being overwritten by the interrupt handler since there will
        // not be interrupts until after we rearm.
        EventRingPointer last_event_pointer = events_start.ringSub(1, eventsBufferSize());
        uint32_t last_event_stop = m_event_infos[last_event_pointer].event_stop_pointer;
        
        // Calculate the remaining space in the events buffer.
        int ring_space = events_end.ringSub(events_start, eventsBufferSize())
                         .ringComplement(eventsBufferSize());
        
        // Call rearmIfSpaceAvailable with the calculated information.
        rearmIfSpaceAvailable(ring_space, eventsPerGroup, buffer_start_pointer, last_event_stop, false); 
    }

    // Calculate the relative hardware timestamp, update previous.
    uint64_t rel_hw_time = (hw_timestamp - m_prev_hw_timestamp) & 0xffffffffffffull;
    m_prev_hw_timestamp = hw_timestamp;

    // Publish information.
    publishMetaInfo(readBurstStartTimeTs, interruptTime, hw_timestamp, rel_hw_time, delayed_rearms_count);
    
    return true;
}

void TRSISDriver::publishMetaInfo(const epicsTimeStamp &readBurstStartTimeTs, double interruptTime,
                                  uint64_t hw_timestamp, uint64_t rel_hw_time, int delayedRearmsCount)
{
    // Convert relative time to seconds.
    double rel_time = rel_hw_time * m_config_timestamp_period.getSnapshotFast();
    
    // Publish the hardware times.
    {
        epicsGuard<asynPortDriver> lock(*this);
        setDoubleParam(m_reg_params[LAST_HW_TIME], hw_timestamp);
        setDoubleParam(m_reg_params[LAST_REL_TIME], rel_time);
        setIntegerParam(m_reg_params[DELAYED_REARMS_COUNT], delayedRearmsCount);
        callParamCallbacks();
    }

    // Publish information handled by the framework.
    TRBurstMetaInfo info(m_event_group_id);
    double readBurstStartTime = getTimeDouble(&readBurstStartTimeTs);    
    if (!m_interrupt_time_broken) {
        info.time_read = (readBurstStartTime - interruptTime) * 1e6; // microseconds
    }
    info.time_process = (getTimeDouble() - readBurstStartTime) * 1e6; // microseconds
    publishBurstMetaInfo(info);
}

bool TRSISDriver::waitForEvents(EventRingPointer events_start)
{
    EventRingPointer events_end;
    
    // Wait for a notification from the VME interrupt handler or a
    // cancellation.
    while (true) {
        // Check if disarming was requested.
        {
            epicsGuard<asynPortDriver> guard(*this);
            if (m_disarm) {
                return false;
            }
        }
        
        // Read the events end pointer safely.
        {
            TR_SIS_InterruptLock lock(interruptLockArg());
            events_end = m_events_end;
        }
        assert(events_end.isValid(eventsBufferSize()));
        
        // Calculate how many events are ready.
        int ring_used = events_end.ringSub(events_start, eventsBufferSize());
        
        // If there is any event ready, return successfully.
        if (ring_used > 0) {
            // Any event ready implies entire event group ready.
            assert(ring_used >= m_config_events_per_group.getSnapshotFast());
            return true;
        }
        
        // Wait for m_interrupt_event to be signaled from the interrupt,
        // or from interruptReading in case disarming is requested.
        m_interrupt_event.wait();
    }
}

bool TRSISDriver::interruptTimeBroken(const epicsTimeStamp &timeStamp)
{
    if (timeStamp.secPastEpoch == 0 && timeStamp.nsec == 0) {
        if (!m_interrupt_time_broken) {
            m_interrupt_time_broken = true;
            errlogSevPrintf(errlogMajor, "readBurst: Reading time in interrupts does not seem to be "
                "working, using times acquired at beginning of processing as data timestamps.");
        }
        return true;
    }
    return false;
}

bool TRSISDriver::channelEnabledForRead(int channel)
{
    int isEnabled;
    epicsGuard<asynPortDriver> lock(*this);
    getIntegerParam(m_reg_params[CHANNEL_ENABLED_FIRST + channel], &isEnabled);
    return (isEnabled != 0);
}

bool TRSISDriver::doDmaForChannel(int channel, uint32_t srcSampleAddress, uint16_t *dest, uint32_t lengthInSamples)
{
    int samplesLeft = lengthInSamples;
    int adcPage = srcSampleAddress / SIS3302AdcMemoryPageSize;
    while (samplesLeft > 0) {
        uint32_t samplesToTransfer = std::min(samplesLeft,
                (int)(getNextPageStart(srcSampleAddress, SIS3302AdcMemoryPageSize) - srcSampleAddress));
        bool status = doDmaForChannelAndPage(channel,
                adcPage % SIS3302AdcMemoryPageNum,
                srcSampleAddress % SIS3302AdcMemoryPageSize,
                dest,
                samplesToTransfer);
        if (!status) {
            // Error message already printed in doDmaForChannelAndPage
            return false;
        }
        dest += samplesToTransfer;
        srcSampleAddress += samplesToTransfer;
        samplesLeft -= samplesToTransfer;
        adcPage++;
    }

    return true;
}

bool TRSISDriver::doDmaForChannelAndPage(int channel, int pageNumber,
        uint32_t offsetInPage, uint16_t *dest, uint32_t lengthInSamples)
{
    // First check if the ADC page number must be updated
    if (pageNumber != (int)m_current_adc_page) {
        m_current_adc_page = pageNumber;
        if (!commitRegister(SIS3302_ADC_MEMORY_PAGE_REGISTER,
                    m_current_adc_page, m_current_adc_page, AdcMemPageCodeMask)) {
            errlogSevPrintf(errlogMajor, "doDmaForChannelAndPage: Error setting ADC memory page.");
            return false;
        }
    }
    
    // Barrier to ensure any CPU accesses to buffer have been done before DMA
    // could start writing.
    dmaMemoryBarrier();

    // Note: no barrier is needed to ensure the page number is updated when
    // the DMA transfer starts, since setting the page number and starting DMA
    // are both volatile accesses to non-cached (I/O) regions.
    
    // Begin DMA transfer
    uint32_t vmeStartAddress = m_vme_a32_base_address + getChannelVmeOffset(channel) +
        2 * offsetInPage;
    int adrsSpace = VME_AM_2eVME_6U | VME_AM_2eSST_HI; // Use the quickest VME mode supported

    STATUS status = rtemsVmeDmaFromVme(m_dma_id, (void *)dest, vmeStartAddress,
            adrsSpace, lengthInSamples * 2, 4);
    if (status != 0) {
        errlogSevPrintf(errlogMajor, "doDmaForChannelAndPage: rtemsVmeDmaFromVme "
                "for channel %d returned status: %d", channel, status);
        return false;
    }

    // Wait for the transfer to complete
    m_dma_event.wait();

    // Check status
    int dmaStatus = rtemsVmeDmaStatusRaw(m_dma_id);
    if (dmaStatus != 0) {
        errlogSevPrintf(errlogMajor, "doDmaForChannelAndPage: DMA transfer "
                "for channel %d finished with error status: %d",
                channel, dmaStatus);
        return false;
    }
    
    // Barrier to ensure we see the new data in the buffer.
    dmaMemoryBarrier();

    return true;
}

TRSISDriver::ProcdEventInfo TRSISDriver::calculateEventAlignment(
    const EventInfo &eventInfo, bool softTrigger)
{
    // The quantities determined in this routine are only meaningful and
    // calculated in paged mode

    ProcdEventInfo procdEventInfo;
    
    // Remember the page wrap bit (whether the data has wrapped around the page).
    procdEventInfo.has_wrapped = ((eventInfo.event_stop_pointer & EventDirWrapBitMask) != 0);

    // This will point to the exact location in the sample buffer where the
    // stop has occurred (may be adjusted below for special case).
    uint32_t actualStopPointer = eventInfo.event_stop_pointer & EventDirFullStopPointerMask;

    // Determine how many samples before actualStopPointer are not available
    // or if negative how many extra samples are available.
    int actualDiscardedSamples = eventInfo.event_stop_pointer & EventDirDiscardedSamplesMask;
    
    // Check for the case where one extra sample was written.
    if (actualDiscardedSamples == 3) {
        // The raw stop pointer points into the next group of 4 samples.
        // Correct actualStopPointer to point into the previous group where
        // the stop actually occurred. This may result in underflow of
        // actualStopPointer but that is fine since below after calculating
        // the difference we extract just the low bits determining the offset
        // within the page.
        actualStopPointer -= 4;
        
        // Set negative actualDiscardedSamples to indicate we have one extra sample
        // after actualStopPointer.
        actualDiscardedSamples = -1;
    }

    // How many samples before actualStopPointer corresponds to the end
    // of the waveform. This is ExtraStopDelaySamples if these were
    // included in the stop delay, else zero.
    uint32_t stopCorrection = softTrigger ? 0 : ExtraStopDelaySamples;
    
    // Calculate the pointer to the end of the waveform relative to the page.
    // This is done by subtracting the correction and extracting only the low
    // bits that define offset within the page.
    procdEventInfo.stop_pointer =
        (uint32_t)(actualStopPointer - stopCorrection) & (m_event_or_page_length - 1);

    // Remember discarded samples.
    procdEventInfo.discarded_samples = actualDiscardedSamples - stopCorrection;
    
    return procdEventInfo;
}

bool TRSISDriver::processBurstData()
{
    // Nothing, data was already processed and submitted in readBurst.
    return true;
}

double TRSISDriver::getTimeDouble(const epicsTimeStamp *epicsTimestampPtr)
{
    epicsTimeStamp ets;
    if (epicsTimestampPtr == NULL) {
        epicsTimeGetCurrent(&ets);
    } else {
        ets = *epicsTimestampPtr;
    }
    return (double)ets.secPastEpoch + ets.nsec * 1e-9;
}

void TRSISDriver::copySamplesToFloats(const uint16_t *src, float *dest, int count)
{
    for (int i = 0; i < count; ++i) {
        dest[i] = (float)src[i];
    }
}

void TRSISDriver::fillFloatsWithNan(float *dest, int count)
{
    for (int i = 0; i < count; ++i) {
        dest[i] = NAN;
    }
}

uint32_t TRSISDriver::getNextPageStart(uint32_t bufferPointer, uint32_t pageSize)
{
    uint32_t page_mask = ~(pageSize - 1);
    return (bufferPointer & page_mask) + pageSize;
}

static bool prioIsValid(int prio)
{
    return prio >= epicsThreadPriorityMin && prio <= epicsThreadPriorityMax;
}

// Register the driver with epics
extern "C"
int TRSISConfigure(char const *port_name,
    uint32_t a32offset, int intVec, int intLev, int read_thread_prio_epics,
    int read_thread_stack_size, int max_ad_buffers, size_t max_ad_memory,
    int interrupt_thread_prio_epics)
{
    // Basic error checking
    if (port_name == NULL || !prioIsValid(read_thread_prio_epics) ||
        !prioIsValid(interrupt_thread_prio_epics))
    { 
        fprintf(stderr, "SISInitDevice Error: parameters are not valid.\n");
        return 1;
    }
    else if ((a32offset & 0xFFFFFFF) != 0) {
        fprintf(stderr, "SISInitDevice Error: VME base address must be a multiple of 0x10000000 (256 MiB).\n");
        return 1;
    }
    else if (intLev < 1 || intLev > 7 || intVec < 0 || intVec > 255) {
        fprintf(stderr, "SISInitDevice Error: Interrupt level must be in range 0-7"
                "and interrupt vector in range 0-255.\n");
        return 1;
    }
    
    TRSISDriver *driver = new TRSISDriver(port_name,
            a32offset, intVec, intLev,
            read_thread_prio_epics, read_thread_stack_size,
            max_ad_buffers, max_ad_memory, interrupt_thread_prio_epics);
    
    driver->completeInit();

    if (!driver->Open()) {
        fprintf(stderr, "SISInitDevice Error: Failed to connect with driver.\n");
        // Must not delete driver to avoid crash in findAsynPortDriver.
        return 1;
    }
    
    return 0;
}

static const iocshArg initArg0 = {"port name", iocshArgString};
static const iocshArg initArg1 = {"sis3302 starting VME address", iocshArgInt};
static const iocshArg initArg2 = {"sis3302 VME interrupt vector", iocshArgInt};
static const iocshArg initArg3 = {"sis3302 VME interrupt level", iocshArgInt};
static const iocshArg initArg4 = {"read thread priority (EPICS units)", iocshArgInt};
static const iocshArg initArg5 = {"read thread stack size", iocshArgInt};
static const iocshArg initArg6 = {"max AreaDetector buffers", iocshArgInt};
static const iocshArg initArg7 = {"max AreaDetector memory", iocshArgInt};
static const iocshArg initArg8 = {"interrupt thread priority (EPICS units)", iocshArgInt};
static const iocshArg * const initArgs[] = {&initArg0, &initArg1, &initArg2,
            &initArg3, &initArg4, &initArg5, &initArg6, &initArg7, &initArg8};
static const iocshFuncDef initFuncDef = {"SISInitDevice", 9, initArgs};

static void initCallFunc(const iocshArgBuf *args)
{
    TRSISConfigure(args[0].sval, args[1].ival, args[2].ival, args[3].ival,
            args[4].ival, args[5].ival, args[6].ival, args[7].ival, args[8].ival);
}

extern "C" {
    void TRSISRegister(void)
    {
        iocshRegister(&initFuncDef, initCallFunc);
    }
    epicsExportRegistrar(TRSISRegister);
}
