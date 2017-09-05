// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_FEATURES_REGIONS_FACTORY_HPP
#define OPENMVG_FEATURES_REGIONS_FACTORY_HPP

#include "openMVG/features/feature.hpp"
#include "openMVG/features/regions.hpp"

namespace openMVG {
namespace features {

/// Define the classic SIFT Keypoint
typedef Scalar_Regions<SIOPointFeature,unsigned char,128> SIFT_Regions;
/// Define the classic SIFT features using float representation
typedef Scalar_Regions<SIOPointFeature,float,128> SIFT_Float_Regions;
/// Define the classic CCTag Keypoint
typedef Scalar_Regions<SIOPointFeature,unsigned char,128> CCTAG_Regions;
/// Define the AKAZE Keypoint (with a float descriptor)
typedef Scalar_Regions<SIOPointFeature,float,64> AKAZE_Float_Regions;
/// Define the AKAZE Keypoint (with a LIOP descriptor)
typedef Scalar_Regions<SIOPointFeature,unsigned char,144> AKAZE_Liop_Regions;

/// Define the AKAZE Keypoint (with a binary descriptor saved in an uchar array)
typedef Binary_Regions<SIOPointFeature,64> AKAZE_Binary_Regions;

} // namespace features
} // namespace openMVG

//--
// Register region type for serialization
//--
#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>
#include <cereal/archives/json.hpp>
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::SIFT_Regions, "SIFT_Regions");
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::SIFT_Float_Regions, "SIFT_Float_Regions");
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::AKAZE_Float_Regions, "AKAZE_Float_Regions");
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::AKAZE_Liop_Regions, "AKAZE_Liop_Regions");
CEREAL_REGISTER_TYPE_WITH_NAME(openMVG::features::AKAZE_Binary_Regions, "AKAZE_Binary_Regions");

#endif // OPENMVG_FEATURES_REGIONS_FACTORY_HPP
