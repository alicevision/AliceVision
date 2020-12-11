// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/feature/PointFeature.hpp>
#include <aliceVision/feature/Regions.hpp>

namespace aliceVision {
namespace feature {

/// Define the classic SIFT Keypoint
using SIFT_Regions = ScalarRegions<unsigned char,128>;
/// Define the classic SIFT features using float representation
using SIFT_Float_Regions = ScalarRegions<float,128>;
/// Define the classic CCTag Keypoint
using CCTAG_Regions = ScalarRegions<unsigned char,128>;
/// Define the AprilTag tag16h5 family Keypoint (30 instance with 5 points each)
using APRILTAG_Regions = ScalarRegions<unsigned char,150>;
/// Define the AKAZE Keypoint (with a float descriptor)
using AKAZE_Float_Regions = ScalarRegions<float,64>;
/// Define the AKAZE Keypoint (with a LIOP descriptor)
using AKAZE_Liop_Regions = ScalarRegions<unsigned char,144>;

/// Define the AKAZE Keypoint (with a binary descriptor saved in an uchar array)
using AKAZE_BinaryRegions = BinaryRegions<64>;

} // namespace feature
} // namespace aliceVision
