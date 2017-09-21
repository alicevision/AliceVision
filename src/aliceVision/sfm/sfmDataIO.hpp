// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/sfm/SfMData.hpp"

namespace aliceVision {
namespace sfm {

enum ESfMData
{
  VIEWS           = 1,
  EXTRINSICS      = 2,
  INTRINSICS      = 4,
  STRUCTURE       = 8,
  OBSERVATIONS    = 16,
  CONTROL_POINTS  = 32,
  ALL = VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE | OBSERVATIONS | CONTROL_POINTS
};

///Check that each pose have a valid intrinsic and pose id in the existing View ids
bool ValidIds(const SfMData & sfm_data, ESfMData flags_part);

/// Load SfMData SfM scene from a file
bool Load(SfMData & sfm_data, const std::string & filename, ESfMData flags_part);

/// Save SfMData SfM scene to a file
bool Save(const SfMData & sfm_data, const std::string & filename, ESfMData flags_part);

} // namespace sfm
} // namespace aliceVision
