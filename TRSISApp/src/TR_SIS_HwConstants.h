/* This file is part of the SIS Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the SIS Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#ifndef TR_SIS_HW_CONSTANTS_H
#define TR_SIS_HW_CONSTANTS_H

#include <stdint.h>

#include "SIS3302.h"

// Size of the VME address space, for the memory mapping
static const int SIS3302AddressSpaceSize = 0x08000000; // 128 MB

// NOTE: even though the sample buffers alone have a capacity of 8 * 64 MB =
// 512 MB > 128 MB, the above value makes sense as only an eighth of each
// channel's sample directories is available in memory at any given time, as
// per below

// Size of ADC memory page available on VME address space (4 MSamples = 8 MB)
static const int SIS3302AdcMemoryPageSize = 0x0400000;
// Number of ADC memory pages
static const int SIS3302AdcMemoryPageNum = 8;

// Offset between actual sample value registers
static const uint32_t SIS3302ChannelActualValueRegSpacing =
    SIS3302_ACTUAL_SAMPLE_VALUE_ADC34 - SIS3302_ACTUAL_SAMPLE_VALUE_ADC12;

// ACQUISTION_CONTROL register bits
static const int AcqControlClockShift = 12;
static const int AcqControlArmedShift = 16;
static const int AcqControlBusyShift = 17;
static const uint32_t AcqControlAutostartMask = (uint32_t)0x1 << 4;
static const uint32_t AcqControlMultiEventMask = (uint32_t)0x1 << 5;
static const uint32_t AcqControlFpStartStopMask = (uint32_t)0x1 << 8;
static const uint32_t AcqControlBigEndiannessMask = (uint32_t)0x1 << 11;
static const uint32_t AcqControlClockMask = (uint32_t)0x7 << AcqControlClockShift;
static const uint32_t AcqControlArmedMask = (uint32_t)0x1 << AcqControlArmedShift;

// STOP_DELAY register limit
static const uint32_t MaxStopDelay = 0xffffff;

// IRQ_CONFIG register bits
static const int IrqConfIrqVectorShift = 0;
static const int IrqConfIrqLevelShift = 8;
static const uint32_t IrqConfIrqEnableMask = (uint32_t)0x1 << 11;
static const uint32_t IrqConfROAKModeMask = (uint32_t)0x1 << 12;

// IRQ_CONTROL register bits
static const uint32_t IrqControlSourceEndLastEvent = 1 << 1;
static const uint32_t IrqControlSourcesMask = 0xff;

// ADC_MEMORY_PAGE register bits
static const uint32_t AdcMemPageCodeMask = 0xF;

// EVENT_CONFIG register bits
static const int EventConfWrapPageSizeShift = 0;
static const int EventConfPageWrapShift = 4;
static const int EventConfEventLengthStopShift = 5;
static const int EventConfAverageModeShift = 12;

// How much smaller value needs to be written to SAMPLE_LENGTH
// register to achieve the desired number of samples
static const int EventLengthValueSubtract = 4;

// CONTROL_STATUS register bits
static const uint32_t CntlStatusUserLed = 0x1;

// DAC_CONTROL_STATUS register bits
static const int DacControlChannelNumShift = 4;
static const uint32_t DacControlLoadShiftReg = 1;
static const uint32_t DacControlLoadDAC = 2;
static const uint32_t DacControlBusy = (uint32_t)1 << 15;

// ADC_INPUT_MODE register bits
static const uint32_t AdcInputModeTestData32bit = (uint32_t)1 << 17;
static const uint32_t AdcInputModeTestDataEnable = (uint32_t)1 << 16;

// Event directory entry bits
// EventDirStopPointerMask extracts the actual stop pointer from an event
// directory entry, ignoring the higher flag bits and lowest two bits
// with boundary information.
static const uint32_t EventDirStopPointerMask = 0x1fffffc;
static const uint32_t EventDirFullStopPointerMask = 0x1ffffff;
static const uint32_t EventDirWrapBitMask = (uint32_t)1 << 28;
static const uint32_t EventDirDiscardedSamplesMask = 0x3;

// Size of HW sample buffer for each channel in samples (32 M-samples)
static const int32_t SampleBufferSize = 0x02000000;

// Available page sizes, in samples
static const uint32_t PageLengths[] = {
    16*1024*1024,
    4*1024*1024,
    1024*1024,
    256*1024,
    64*1024,
    16*1024,
    4*1024,
    1024,
    512,
    256,
    128,
    64
};

// Maximum sample rate, also timestamp rate when using external
// random clock.
static const double MaxInternalFreq = 100e6;

// Depth of event and timestamp directories
static const int EventDirectorySize = 512;

#endif
