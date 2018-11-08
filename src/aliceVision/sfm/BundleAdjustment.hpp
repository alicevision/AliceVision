// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/detail/bitmask.hpp>

namespace aliceVision {

namespace sfmData {
class SfMData;
} // namespace sfmData

namespace sfm {

class BundleAdjustment
{
  public:

  enum ERefineOptions
  {
    REFINE_NONE = 0,
    REFINE_ROTATION = 1,                                  //< refine pose rotations
    REFINE_TRANSLATION = 2,                               //< refine pose translations
    REFINE_STRUCTURE = 4,                                 //<  refine structure (i.e. 3D points)
    REFINE_INTRINSICS_FOCAL = 8,                          //< refine the focal length
    REFINE_INTRINSICS_OPTICALCENTER_ALWAYS = 16,          //< refine the optical center
    REFINE_INTRINSICS_OPTICALCENTER_IF_ENOUGH_DATA = 32,  //< refine the optical center only if we have a minimum number of cameras
    REFINE_INTRINSICS_DISTORTION = 64,                    //< refine the distortion parameters
    /// Refine all intrinsics parameters
    REFINE_INTRINSICS_ALL = REFINE_INTRINSICS_FOCAL | REFINE_INTRINSICS_OPTICALCENTER_IF_ENOUGH_DATA | REFINE_INTRINSICS_DISTORTION,
    /// Refine all parameters
    REFINE_ALL = REFINE_ROTATION | REFINE_TRANSLATION | REFINE_INTRINSICS_ALL | REFINE_STRUCTURE,
  };

  /**
   * @brief Perform a Bundle Adjustment on the SfM scene with refinement of the requested parameters
   * @param[in,out] sfmData: sfmData scene to modify with the BA
   * @param[in] refineOptions: choose what you want to refine
   * @return 
   */
  virtual bool adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions = REFINE_ALL) = 0;

  // TODO: Use filter to say wich parameter is const or not (allow to refine only a subpart of the intrinsics or the poses)
};

BOOST_BITMASK(BundleAdjustment::ERefineOptions)

} // namespace sfm
} // namespace aliceVision
