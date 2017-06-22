/* This file is part of the SIS Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the SIS Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

/* EPICS glue to the RTEMS VMEDMA API */
/* This is a modified version of the header compatible with C++. */

#ifndef DRV_RTEMS_VME_DMA_SUP_H
#define DRV_RTEMS_VME_DMA_SUP_H

#include <stdint.h>

typedef void 		(*VOIDFUNCPTR)(void*);
typedef uint32_t UINT32;
typedef uint16_t UINT16;
typedef int      STATUS;

typedef struct dmaRequest *DMA_ID;

#ifdef __cplusplus
extern "C" {
#endif

DMA_ID
rtemsVmeDmaCreate(VOIDFUNCPTR callback, void *context);

STATUS
rtemsVmeDmaStatus(DMA_ID dmaId);

/* retrieve the raw status (as passed from device) of a terminated DMA */
uint32_t
rtemsVmeDmaStatusRaw(DMA_ID dmaId);

STATUS
rtemsVmeDmaFromVme(DMA_ID dmaId, void *pLocal, UINT32 vmeAddr,
    int adrsSpace, int length, int dataWidth);

STATUS
rtemsVmeDmaToVme(DMA_ID dmaId, UINT32 vmeAddr, int adrsSpace,
    void *pLocal, int length, int dataWidth);

#ifdef __cplusplus
}
#endif

#endif
