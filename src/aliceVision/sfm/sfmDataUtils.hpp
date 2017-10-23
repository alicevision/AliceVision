// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/SfMData.hpp>

namespace aliceVision {
namespace sfm {

struct SfMData;

// Group camera models that share common camera properties
// It modifies the intrinsic_id of the view field and change the sfm_data.intrinsics length
// Grouping is simplified by using a hash function over the camera intrinsics
// - it allow to merge camera model that share common camera parameters & image sizes
void GroupSharedIntrinsics(SfMData & sfm_data);

} // namespace sfm
} // namespace aliceVision
