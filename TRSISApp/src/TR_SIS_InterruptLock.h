/* This file is part of the SIS Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the SIS Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#ifndef TR_SIS_INTERRUPT_LOCK_H
#define TR_SIS_INTERRUPT_LOCK_H

#include <epicsInterrupt.h>

#include <TRNonCopyable.h>

class TR_SIS_InterruptLock : private TRNonCopyable {
private:
    int m_key;
    
public:
    inline TR_SIS_InterruptLock()
    {
        m_key = epicsInterruptLock();
    }
    
    inline ~TR_SIS_InterruptLock()
    {
        epicsInterruptUnlock(m_key);
    }
};

#endif
