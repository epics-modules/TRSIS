/* This file is part of the SIS Digitizer Driver.
 * It is subject to the license terms in the LICENSE.txt file found in the
 * top-level directory of this distribution and at
 * https://confluence.slac.stanford.edu/display/ppareg/LICENSE.html. No part
 * of the SIS Digitizer Driver, including this file, may be copied,
 * modified, propagated, or distributed except according to the terms
 * contained in the LICENSE.txt file.
 */

#ifndef TR_SIS_RING_POINTER_H
#define TR_SIS_RING_POINTER_H

#include <epicsAssert.h>

template <typename IndexType>
class TR_SIS_RingPointer {
    // IndexType must be a signed integer type.
    STATIC_ASSERT((IndexType)-1 == -1); // poor man's std::is_signed
    
public:
    IndexType index;
    
public:
    inline TR_SIS_RingPointer()
    {
    }
    
    inline TR_SIS_RingPointer(IndexType index)
    : index(index)
    {
    }
    
    inline bool isValid(IndexType ringSize) const
    {
        return index >= 0 && index < ringSize;
    }
    
    inline operator IndexType() const
    {
        return index;
    }
    
    inline TR_SIS_RingPointer ringAdd(TR_SIS_RingPointer const &other, IndexType ringSize) const
    {
        IndexType res_index = index + other.index;
        if (res_index >= ringSize) {
            res_index -= ringSize;
        }
        return res_index;
    }
    
    inline TR_SIS_RingPointer ringSub(TR_SIS_RingPointer const &other, IndexType ringSize) const
    {
        IndexType res_index = index - other.index;
        if (res_index < 0) {
            res_index += ringSize;
        }
        return res_index;
    }
    
    inline void ringInc(IndexType ringSize)
    {
        index++;
        if (index == ringSize) {
            index = 0;
        }
    }
    
    TR_SIS_RingPointer ringComplement(IndexType ringSize)
    {
        return ringSize - 1 - index;
    }
};

#endif
