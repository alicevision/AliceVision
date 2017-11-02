// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/sfmDataIO.hpp>

namespace aliceVision {
namespace sfm {

/// Save the structure and camera positions of a SfMData container as 3D points in a PLY ASCII file.
bool Save_PLY(const SfMData & sfm_data,
              const std::string & filename,
              ESfMData flags_part);

} // namespace sfm
} // namespace aliceVision
