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

#include <stddef.h>

#include <epicsInterrupt.h>
#include <epicsMutex.h>

#include <TRNonCopyable.h>

/**
 * Lock for synchronization with an interrupt handler.
 * 
 * It supports two modes:
 * - Synchronization with an actual interrupt handler. In this case it uses
 *   epicsInterruptLock and epicsInterruptUnlock.
 * - Synchronization with an interrupt handler that runs in a thread. In this
 *   case, it uses a mutex which the interrupt handler must also use.
 * 
 * The intended approach is that the interrupt handler uses TR_SIS_InterruptSideLock
 * with the same mutex parameter in the constructor, either both null or both the
 * same mutex.
 */
class TR_SIS_InterruptLock : private TRNonCopyable {
private:
    epicsMutex *m_non_interrupt_mutex;
    int m_key;
    
public:
    inline TR_SIS_InterruptLock(epicsMutex *non_interrupt_mutex) :
        m_non_interrupt_mutex(non_interrupt_mutex)
    {
        if (m_non_interrupt_mutex != NULL) {
            m_non_interrupt_mutex->lock();
        } else {
            m_key = epicsInterruptLock();
        }
    }
    
    inline ~TR_SIS_InterruptLock()
    {
        if (m_non_interrupt_mutex != NULL) {
            m_non_interrupt_mutex->unlock();
        } else {
            epicsInterruptUnlock(m_key);
        }
    }
};

/**
 * Interrupt-side lock that goes together with TR_SIS_InterruptLock.
 * 
 * If the mutex argument is null, it does nothing at all, and if it is
 * not null then it locks and unlocks that mutex.
 */
class TR_SIS_InterruptSideLock : private TRNonCopyable {
private:
    epicsMutex *m_non_interrupt_mutex;
    
public:
    inline TR_SIS_InterruptSideLock(epicsMutex *non_interrupt_mutex) :
        m_non_interrupt_mutex(non_interrupt_mutex)
    {
        if (m_non_interrupt_mutex != NULL) {
            m_non_interrupt_mutex->lock();
        }
    }
    
    inline ~TR_SIS_InterruptSideLock()
    {
        if (m_non_interrupt_mutex != NULL) {
            m_non_interrupt_mutex->unlock();
        }
    }
};

#endif
