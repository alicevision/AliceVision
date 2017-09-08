// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

namespace aliceVision {
namespace sfm {

struct SfM_Data;

// Group camera models that share common camera properties
// It modifies the intrinsic_id of the view field and change the sfm_data.intrinsics length
// Grouping is simplified by using a hash function over the camera intrinsics
// - it allow to merge camera model that share common camera parameters & image sizes
void GroupSharedIntrinsics(SfM_Data & sfm_data);

} // namespace sfm
} // namespace aliceVision
