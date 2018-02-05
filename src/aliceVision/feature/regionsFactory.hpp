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
typedef ScalarRegions<SIOPointFeature,unsigned char,128> SIFT_Regions;
/// Define the classic SIFT features using float representation
typedef ScalarRegions<SIOPointFeature,float,128> SIFT_Float_Regions;
/// Define the classic CCTag Keypoint
typedef ScalarRegions<SIOPointFeature,unsigned char,128> CCTAG_Regions;
/// Define the AKAZE Keypoint (with a float descriptor)
typedef ScalarRegions<SIOPointFeature,float,64> AKAZE_Float_Regions;
/// Define the AKAZE Keypoint (with a LIOP descriptor)
typedef ScalarRegions<SIOPointFeature,unsigned char,144> AKAZE_Liop_Regions;

/// Define the AKAZE Keypoint (with a binary descriptor saved in an uchar array)
typedef BinaryRegions<SIOPointFeature,64> AKAZE_BinaryRegions;

} // namespace feature
} // namespace aliceVision
