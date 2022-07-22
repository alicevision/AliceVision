// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace vfs {

using boost::system::error_code;
using boost::filesystem::copy_option;
using boost::filesystem::directory_options;
using boost::filesystem::file_status;
using boost::filesystem::space_info;

} // namespace vfs
} // namespace aliceVision
