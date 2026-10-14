// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "magma_v2.h"
#include <magma_lapack.h>
#include <magma_operators.h>

namespace aliceVision 
{
namespace uncertainty
{

struct MagmaBuffer
{
    double* ptr = nullptr;

    MagmaBuffer() = default;

    explicit MagmaBuffer(magma_int_t count)
    {
        if (magma_dmalloc(&ptr, count) != MAGMA_SUCCESS)
        {
            ptr = nullptr;
        }
    }

    ~MagmaBuffer() { magma_free(ptr); }

    MagmaBuffer(const MagmaBuffer &) = delete;
    MagmaBuffer & operator=(const MagmaBuffer &) = delete;

    MagmaBuffer(MagmaBuffer && o) noexcept : ptr(o.ptr) { o.ptr = nullptr; }
    
    MagmaBuffer & operator=(MagmaBuffer && o) noexcept
    {
        if (this != &o) 
        { 
            magma_free(ptr); 
            ptr = o.ptr; 
            o.ptr = nullptr; 
        }
        return *this;
    }

    bool valid() const { return ptr != nullptr; }
    operator double*() const { return ptr; }
};


}
}