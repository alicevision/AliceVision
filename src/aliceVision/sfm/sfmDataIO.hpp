// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/SfMData.hpp>

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
  LANDMARKS_UNCERTAINTY  = 64,
  POSES_UNCERTAINTY = 128,

  UNCERTAINTY = LANDMARKS_UNCERTAINTY | POSES_UNCERTAINTY,
  ALL = VIEWS | EXTRINSICS | INTRINSICS | STRUCTURE | OBSERVATIONS | CONTROL_POINTS | UNCERTAINTY
};

///Check that each pose have a valid intrinsic and pose id in the existing View ids
bool ValidIds(const SfMData& sfmData, ESfMData partFlag);

/// Load SfMData SfM scene from a file
bool Load(SfMData& sfmData, const std::string& filename, ESfMData partFlag);

/// Save SfMData SfM scene to a file
bool Save(const SfMData& sfmData, const std::string& filename, ESfMData partFlag);

} // namespace sfm
} // namespace aliceVision
