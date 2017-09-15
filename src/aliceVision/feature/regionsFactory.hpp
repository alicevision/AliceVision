// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include "aliceVision/feature/PointFeature.hpp"
#include "aliceVision/feature/Regions.hpp"

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

//--
// Register region type for serialization
//--
#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>

CEREAL_REGISTER_TYPE_WITH_NAME(aliceVision::feature::SIFT_Regions, "SIFT_Regions");
CEREAL_REGISTER_TYPE_WITH_NAME(aliceVision::feature::SIFT_Float_Regions, "SIFT_Float_Regions");
CEREAL_REGISTER_TYPE_WITH_NAME(aliceVision::feature::AKAZE_Float_Regions, "AKAZE_Float_Regions");
CEREAL_REGISTER_TYPE_WITH_NAME(aliceVision::feature::AKAZE_Liop_Regions, "AKAZE_Liop_Regions");
CEREAL_REGISTER_TYPE_WITH_NAME(aliceVision::feature::AKAZE_BinaryRegions, "AKAZE_BinaryRegions");

