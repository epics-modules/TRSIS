/* This file is part of the SIS Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the SIS Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#ifndef TR_SIS_H
#define TR_SIS_H

#include <stdint.h>
#include <stddef.h>

#include <vector>

#include <epicsEvent.h>
#include <epicsTime.h>
#include <epicsAssert.h>
#include <epicsThread.h>

#include <TRBaseDriver.h>

#include "rtemsVmeDmaSup.h"

#include "TR_SIS_RingPointer.h"

class TRSISDriver : public TRBaseDriver
{
public:
    TRSISDriver(char const *port_name,
        uint32_t a32offset, int intVec, int intLev, int read_thread_prio,
        int read_thread_stack_size, int max_ad_buffers, size_t max_ad_memory,
        int interrupt_thread_prio_epics);

    // NOTE: For the sake of principle, there is nothing private in this class and
    // most functions are virtual. But this class was not designed to be extended
    // like that. Support for similar devices might be easiest to implement by
    // directly modifying this class and possibly by splitting into a base class
    // and device-specific derived classes based on the similarities and differences
    // of devices.
    
protected:
    // Constants (only those we need in the header)
    static const int MaxChannelNum = 8;
    static const uint32_t FullBitMask = 0xffffffff;

    // Ring pointer types
    typedef TR_SIS_RingPointer<int> EventRingPointer;
    typedef TR_SIS_RingPointer<int32_t> SampleRingPointer;

    // STRUCTURE TYPES

    // These structures are populated in the interrupt handler
    struct EventInfo
    {
        uint32_t event_stop_pointer;
        uint32_t event_timestamp_low;
        uint16_t event_timestamp_high;
        // The timestamp is only valid for the first event of an event group
        epicsTimeStamp epics_timestamp;
    };

    // These quantities are calculated on the read thread from the data in the
    // corresponding EventInfo structure
    struct ProcdEventInfo
    {
        uint32_t stop_pointer;
        bool has_wrapped;
        int discarded_samples;
    };
    
    // epicsThreadRunable helper for the interrupt thread
    class InterruptThreadRunnable : public epicsThreadRunable
    {
    public:
        inline InterruptThreadRunnable(TRSISDriver *parent) :
            m_parent(parent)
        {}
        
    private:
        TRSISDriver *m_parent;
        
        void run () // override
        {
            m_parent->interruptThread();
        }
    };

    // ENUMS

    // Enumeration of regular parameters
    // Parameters are R/W if not indicated otherwise
    enum SisAsynParams {
        // Requests
        SEND_SW_TRIGGER,            // W/O
        UPDATE_ACTUAL_VALUES,       // W/O
        UPDATE_STATES,              // W/O
        GENERAL_RESET,              // p. 28, W/O
        CLEAR_TIMESTAMP,            // p. 28, W/O

        // Static information
        FIRMWARE_MAJOR_REVISION,    // p. 15, R/O
        FIRMWARE_MINOR_REVISION,    // p. 15, R/O
        
        // Dynamic information (states)
        ACTUAL_EVENTS,              // p. 22, R/O
        CLOCK_SOURCE_READBACK,      // p. 19, R/O
        SAMPLING_BUSY,              // p. 19, R/O
        SAMPLING_LOGIC_ARMED,       // p. 19, R/O
        STOP_DELAY_READBACK,        // p. 21, R/O
        NEXT_SAMPLE_ADDRESS_CH0,    // p. 36, R/O
        DELAYED_REARMS_COUNT,       // R/O, I/O Intr

        // Last hardware timestamps
        LAST_HW_TIME,  // raw absolute timestamp
        LAST_REL_TIME, // relative timestamp in seconds

        // Channel enabled
        CHANNEL_ENABLED_FIRST,
        CHANNEL_ENABLED_LAST = CHANNEL_ENABLED_FIRST + MaxChannelNum - 1,

        // Channel actual value
        CHANNEL_ACTUAL_VALUE_FIRST,
        CHANNEL_ACTUAL_VALUE_LAST = CHANNEL_ACTUAL_VALUE_FIRST + MaxChannelNum - 1,
        
        // Channel DAC setting
        CHANNEL_DAC_FIRST,
        CHANNEL_DAC_LAST = CHANNEL_DAC_FIRST + MaxChannelNum - 1,
        
        NUM_ASYN_PARAMS
    };

    enum TriggeringMode {
        TriggerSoft,
        TriggerAutostartFPStop,
        TriggerFPStartFPStop,
        TriggerFPStartAutostop,
    };

    // These directly correspond to the values defined on p. 20 of the manual
    enum ClockSource {
        ClockSource100Mhz = 0,
        ClockSource50Mhz,
        ClockSource25Mhz,
        ClockSource10Mhz,
        ClockSource1Mhz,
        ClockSourceExternalRandom,
        ClockSourceExternalSymmetric,
        ClockSource2nd100Mhz,
    };
    
    enum TestDataMode {
        TestDataDisabled,
        TestData16bit,
        TestData32bit
    };

    enum AveragedSamplesLog {
        AveragedSamplesLog0 = 0,
        AveragedSamplesLog1,
        AveragedSamplesLog2,
        AveragedSamplesLog3,
        AveragedSamplesLog4,
        AveragedSamplesLog5,
        AveragedSamplesLog6,
        AveragedSamplesLog7,
        AveragedSamplesLogTooBig
    };
    
    // DATA MEMBERS

    // List of regular parameters' asyn indices
    int m_reg_params[NUM_ASYN_PARAMS];

    // Concrete configuration parameters
    // NOTE: update NumConfigParams on any change!
    TRConfigParam<int>                 m_config_triggering_mode;
    TRConfigParam<int>                 m_config_averaged_samples_log;
    TRConfigParam<int>                 m_config_clock_source;
    TRConfigParam<double>              m_config_external_clock_rate;
    TRConfigParam<int, double>         m_config_events_per_group;
    TRConfigParam<int, double>         m_config_events_buffer_size;
    TRConfigParam<int>                 m_config_test_data_mode;
    TRConfigParam<int, double>         m_config_test_data_start;
    
    // Internal configuration parameters
    
    // Assumed period of timestamp counter, set in checkClockAndAveragingSettings
    // and used in readBurst to calculate relative burst time (internal parameter).
    TRConfigParam<double>              m_config_timestamp_period;

    static const int NumConfigParams = 9;

    // Hardware configuration, determined at initialization
    uint32_t m_vme_a32_base_address;
    volatile char *m_pci_base_address;
    uint32_t m_interrupt_level;
    uint32_t m_interrupt_vector;

    // Handle for scheduling DMA transfers
    DMA_ID m_dma_id;

    // Acquisition settings (all of this is computed within checkSettings)
    
    // Whether we need page-wrap behavior (or wrap within entire buffer)
    bool m_use_paged_acquisition;
    // Whether we need autostart enabled
    bool m_autostart;
    // Value put into SAMPLE_LENGTH registers
    uint32_t m_event_or_page_length;
    // Value put into ADC_INPUT_MODE registers (test data generation config)
    uint32_t m_adc_input_mode;
    // Selected page size index or -1 for wrap in whole memory because no
    // page was large enough. Defined only when m_use_paged_acquisition.
    int m_page_code;
    // The number of sampling periods to delay the stop signal. Only used with
    // paged acquisition.
    uint32_t m_stop_delay;
    
    // End of acquisition settings

    // Thread which handles interrupts with runnable helper.
    InterruptThreadRunnable m_interrupt_thread_runnable;
    epicsThread m_interrupt_thread;
    
    // Mutex used for protecting certain data that is accessed both by the interrupt
    // thread and other threads such as the read thread.
    epicsMutex m_interrupt_mutex;
    
    // Synchronization events:
    
    // This event is signaled in the interruptHandler, and waited on in the
    // interruptThread which proceeds to call handleSingleInterrupt.
    epicsEvent m_interrupt_handler_event;
    
    // This event is signaled in handleSingleInterrupt after event information
    // has been stored and the device was possibly rearmed, and is waited on in
    // the read thread to wait for events (waitForEvents). Additionally it is
    // signaled when disarming was requested (interruptReading). Finally it is
    // also used for acknowledging a stop synchronization request
    // (synchronizeStopWithInterruptThread).
    epicsEvent m_interrupt_event;
    
    // This event is signaled in dmaInterruptHandler when a DMA transfer is
    // complete and is waited on in doDmaForChannelAndPage.
    epicsEvent m_dma_event;
    
    // Synchronization flags
    bool m_opened;
    bool m_armed;
    bool m_disarm;
    bool m_stop_interrupt;
    
    // Read thread <-> interrupt handler synchronization:
    // whether digitizer needs to be rearmed from read thread
    bool m_need_rearm;
    // sample memory address where data begins
    SampleRingPointer m_buffer_start_pointer;
    // index of first event in vector below
    EventRingPointer m_events_start;
    // index of last event in vector below (start==end -> empty buffer)
    EventRingPointer m_events_end;
    // vector for the event buffer
    std::vector<EventInfo> m_event_infos;

    // Vector of length eventsPerGroup holding per-event calculated values.
    // This is populated before processing an event group so that quantities do
    // not have to be recalculated for each enabled channel.
    std::vector<ProcdEventInfo> m_procd_event_infos;
    
    // Hardware timestamp of previous event group or zero if none since arming
    uint64_t m_prev_hw_timestamp;
    
    // Event group ID counter
    int m_event_group_id;

    // Remaining bursts counter.
    // It is used in the interrupt handler to avoid re-arming if TRBaseDriver
    // would anyway not process the data. Negative value means an unlimited
    // number of bursts (event groups).
    int m_remaining_bursts;

    // Temporary buffer for sample data, it is populated using DMA at the
    // beginning of each channel's processing.
    std::vector<uint16_t> m_sample_vector; 
    
    // The index of the last sample buffer element + 1 that has been populated
    // through DMA. Necessary to keep track of during multi-page transfers.
    int m_sample_buffer_pos;
    
    // The currently selected ADC page available on the VME address space.
    // It is used to determine when the page needs to be changed.
    uint32_t m_current_adc_page;

    // Interrupt timestamps were found to be broken in certain cases. The
    // following flag marks whether that is true. In that case, a warning
    // message is printed when the problem is first detected, interrupt
    // processing times are not calculated, and NDArrays are submitted with
    // timestamps obtained in the read thread.
    bool m_interrupt_time_broken;

    // Informative counter of how many times during the last acquisition the
    // digitizer's sample buffer was too full for the interrupt handler to
    // internally rearm it. Instead, a delayed rearm occurred on the read
    // thread after it processed an event group, freeing up enough space in the
    // digitizer's buffer.
    int m_delayed_rearms_count;

public:
    // Called after port creation to open the device.
    bool Open();

protected:
    // Small inline functions
    
    inline static bool isChannelNumber(int channel)
    {
        return channel >= 0 && channel < MaxChannelNum;
    }
    
    inline int eventsBufferSize()
    {
        return m_config_events_buffer_size.getSnapshotFast();
    }
    
    // Mutex pointer passed to TR_SIS_InterruptLock constructor for locking
    // data that is also accessed by handleSingleInterrupt. If we ever support
    // configuable interrupt handling mode (raw or thread), this could make
    // a decision to return null or &m_interrupt_mutex respectively.
    inline epicsMutex * interruptLockArg()
    {
        return &m_interrupt_mutex;
    }

    // Called after the device is opened, updates device info and sets m_opened.
    virtual void finalizeOpen(uint32_t moduleRegValue);
    
    // Register access functions
    void writeRegister(uint32_t offset, uint32_t value, bool from_interrupt = false);
    uint32_t readRegister(uint32_t offset, bool from_interrupt = false);
    
    // A memory barrier which ensures ordering between memory stores and I/O stores.
    static void ioMemoryBarrier();
    
    // A memory barrier which ensures synchronization between CPU memory accesses
    // and DMA accesses.
    static void dmaMemoryBarrier();

    // Asyn parameter access handlers
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value); // override
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value); // override

    // Handlers for requests via asyn parameter writes.
    virtual asynStatus handleSendSoftwareTrigger();
    virtual asynStatus updateActualValues();
    virtual asynStatus updateStates();
    virtual asynStatus handleReset();
    virtual asynStatus handleClearTimestamp();
    virtual asynStatus handleSetChannelDac(int channel, int value);
    
    // Reset digitizer with verification.
    virtual bool resetAndVerify();
    
    // Set channel DAC value.
    virtual bool setChannelDac(int channel, uint16_t value);

    // Set a clock source to the desired-parameter value, used to ensure the
    // clock source is set to the desired value while disarmed since it affects
    // actual value readbacks.
    virtual bool setDesiredClockSource();
    
    // This is called before startAcquisition, it checks settings and comptes
    // many variables used later. The remaining three functions do specific
    // parts of this.
    virtual bool checkSettings(TRArmInfo &arm_info); // override
    virtual bool checkClockAndAveragingSettings(TRArmInfo &arm_info);
    virtual bool checkEventConfiguration(TRArmInfo &arm_info);
    virtual bool checkTestDataConfiguration();

    // Finds the best page large enough to contain the specified eventLength.
    // Sets m_page_code and m_event_or_page_length.
    virtual bool findPageLength(uint32_t eventLength);

    // Prepare hardware for reading data.
    virtual bool startAcquisition(bool had_overflow); // override

    // Release memory in the three vectors used for data processing.
    virtual bool releaseAllVectors();

    // Called when disarming is requested after startAcquisition was successful
    // to make sure readBurst returns soon.
    virtual void interruptReading (); // override
    
    // Disarm hardware (reverse of startAcquisition).
    virtual void stopAcquisition(); // override
    
    // Synchronize stop of acqisition with the interrupt thread.
    virtual void synchronizeStopWithInterruptThread();
    
    // Called when the digitizer is considered no longer armed, used
    // to apply a possible desired clock source change.
    virtual void onDisarmed (); // override
    
    // Set m_armed under port lock and interrupt lock.
    virtual void setArmed(bool value);

    // Interrupt handler static functions, just call correspondingly named
    // non-static functions.
    static void interruptHandlerTrampoline(void *arg);
    static void dmaInterruptHandlerTrampoline(void *arg);
    
    // The interrupt handlers as nonstatic functions.
    void interruptHandler();
    void dmaInterruptHandler();
    
    // The interrupt handling thread.
    virtual void interruptThread();
    
    // Handle a single interrupt.
    virtual void handleSingleInterrupt(epicsMutex *non_interrupt_mutex);
    
    // Rearm if there is space available in the events buffer and sample buffer.
    virtual bool rearmIfSpaceAvailable(
        int ring_space, int eventsPerGroup, SampleRingPointer buffer_start_pointer,
        uint32_t last_event_stop, bool from_interrupt);
    
    // Register commit helper functions
    
    // Verifies that a specific part of a register matches the expected value.
    // This is done by reading it and checking that the possibly masked value
    // matches the expected value. If maxRetries is greater than 0 then we poll
    // the register that many times before giving up, otherwise a match is
    // expected immediately. For most registers retries are not even necessary,
    // but they are for example with  write-only "ALL" registers that update
    // several other registers. This is also used to wait for DAC commands.
    // Since the hardware typically responds quickly, on the order of microseconds,
    // we do not issue any sleep commands between retries.
    virtual bool verifyRegister(uint32_t offset, uint32_t desiredReadbackValue,
        uint32_t readbackComparisonMask = FullBitMask, int maxRetries = 20);

    // Writes then verifies a register.
    virtual bool commitRegister(uint32_t offset, uint32_t commitValue,
        uint32_t desiredReadbackValue, uint32_t readbackComparisonMask = FullBitMask,
        int maxRetries = 20);

    // Convenience function for configuring a common type of register in which
    // the lower 16 bits set specific bits and the upper bits clear those bits.
    // Since in some cases only a subset of the bits work like that, we support
    // a mask which specifies the bits that we want to configure (the mask should
    // have low bits only).
    virtual bool commitSetClearRegister(uint32_t offset, uint32_t value, uint32_t mask);
    
    // Data reading and related helper functions
    
    // Wait for and read a burst of data.
    virtual bool readBurst(); // override

    // Wait for some events to be ready.
    virtual bool waitForEvents(EventRingPointer events_start);
    
    // Check if interrupt time is broken and possibly warn.
    bool interruptTimeBroken(const epicsTimeStamp &timeStamp);
    
    // Check if data for a channel should be read (used from read thread without lock).
    bool channelEnabledForRead(int channel);
    
    // Transfer lengthInSamples samples from offset srcSampleAddress in the
    // digitizer's buffer to the local buffer dest, using VME DMA.
    virtual bool doDmaForChannel(int channel, uint32_t srcSampleAddress,
                                 uint16_t *dest, uint32_t lengthInSamples);

    // Perform a single DMA transfer, for a single channel and ADC page.
    virtual bool doDmaForChannelAndPage(int channelNumber, int pageNumber,
        uint32_t offsetInPage, uint16_t *dest, uint32_t lengthInSamples);

    // Calculate metadata about an event in form of a ProcdEventInfo structure.
    virtual ProcdEventInfo calculateEventAlignment(
        const EventInfo &eventInfo, bool softTrigger);

    // Publish information about event group, called at the end of readBurst.
    virtual void publishMetaInfo(
        const epicsTimeStamp &readBurstStartTimeTs, double interruptTime,
        uint64_t hw_timestamp, uint64_t rel_hw_time, int delayed_rearms_count);
    
    // This was meant to process data previously read in readBurst but since
    // we also process data there to improve memory effficiency, it does nothing.
    virtual bool processBurstData(); // override

    // NOTE: We don't override requestedSampleRateChanged because the clock selection
    // mechanism offered by TRBaseDriver is of no use to us, the hardware only
    // supports several clock sources.

    // Transforms an epicsTimeStamp to the corresponding double, or
    // uses the current time if NULL is passed in
    static double getTimeDouble(const epicsTimeStamp *epicsTimestampPtr = NULL);

    // Copy 16-bit samples to floats, used in readBurst.
    static void copySamplesToFloats(const uint16_t *src, float *dest, int count);

    // Fill destination buffer with all NaNs, used in readBurst when stop
    // trigger arrives to soon in paged mode.
    static void fillFloatsWithNan(float *dest, int count);

    // Return the beginning of the next page after the page containing a given
    // pointer. The return value is not wrapped, meaning that this function
    // never returns 0 and may return SampleBufferSize. The pageSize has to be
    // a power of 2 for this function to return sensible results.
    static uint32_t getNextPageStart(uint32_t bufferPointer, uint32_t pageSize);
};

#endif
