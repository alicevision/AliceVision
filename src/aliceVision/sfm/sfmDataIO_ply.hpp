// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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
