/* This file is part of the SIS Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the SIS Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#ifndef TR_SIS_DROP_THREAD_PRIORITY_H
#define TR_SIS_DROP_THREAD_PRIORITY_H

#include <epicsThread.h>
#include <epicsAssert.h>

#include <TRNonCopyable.h>

class TR_SIS_DropThreadPriority : private TRNonCopyable {
private:
    epicsThreadId m_thread_id;
    unsigned int m_orig_priority;
    
public:
    // Constructor only, does not drop priority (use dropPriority).
    inline TR_SIS_DropThreadPriority()
    : m_thread_id(NULL),
      m_orig_priority(0) // just to avoid warning
    {
    }
    
    // Destructor, restores priority if it was dropped.
    inline ~TR_SIS_DropThreadPriority()
    {
        if (m_thread_id != NULL) {
            epicsThreadSetPriority(m_thread_id, m_orig_priority);
        }
    }
    
    // Drops thead priority to drop_priority.
    // Does not change priority if drop_priority is not less than
    // the current priority. Must not be called more than once.
    void dropPriority(unsigned int drop_priority)
    {
        assert(m_thread_id == NULL);
        
        m_thread_id = epicsThreadGetIdSelf();
        m_orig_priority = epicsThreadGetPriority(m_thread_id);
        
        if (drop_priority < m_orig_priority) {
            epicsThreadSetPriority(m_thread_id, drop_priority);
        } else {
            m_thread_id = NULL;
        }
    }
};

#endif
