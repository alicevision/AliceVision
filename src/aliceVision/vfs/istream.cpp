// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "istream.hpp"

namespace aliceVision {
namespace vfs {

void istream::open(const char* filename, std::ios_base::openmode mode)
{
    _buffer = std::make_unique<std::filebuf>();
    set_rdbuf(_buffer.get());
    if (_buffer->open(filename, mode | std::ios_base::in))
    {
        clear();
    }
    else
    {
        setstate(std::ios_base::failbit);
    }
}

} // namespace vfs
} // namespace aliceVision
