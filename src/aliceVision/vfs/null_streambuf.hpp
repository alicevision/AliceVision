// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <iostream>

namespace aliceVision {
namespace vfs {

/*
 * This is a streambuf representing an empty stream sequence.
 */
class null_streambuf : public std::streambuf {
public:
    null_streambuf() = default;
};

} // namespace vfs
} // namespace aliceVision
