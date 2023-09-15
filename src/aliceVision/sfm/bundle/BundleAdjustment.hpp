// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <boost/detail/bitmask.hpp>
#include <string>
#include <algorithm>
#include <stdexcept>

namespace aliceVision {

namespace sfmData {
class SfMData;
} // namespace sfmData

namespace sfm {

/**
 * @brief Defines basic, scale and covariance options for features that can be used in a bundle adjustment.
 */
enum class EFeatureConstraint
{
  BASIC = 0,
  SCALE = 1
};

/**
*@brief convert an enum ESfMobservationConstraint to its corresponding string
*
*/
inline std::string ESfMobservationConstraint_enumToString(EFeatureConstraint m)
{
  switch(m)
  {
    case EFeatureConstraint::BASIC: return "Basic";
    case EFeatureConstraint::SCALE: return "Scale";
  }
  throw std::out_of_range("Invalid ESfMobservationConstraint enum: " + std::to_string(int(m)));
}

/**
* @brief convert a string featureConstraint to its corresponding enum featureConstraint
* @param String
* @return ESfMobservationConstraint
*/
inline EFeatureConstraint ESfMobservationConstraint_stringToEnum(const std::string& m)
{
  std::string featureConstraint = m;
  std::transform(featureConstraint.begin(), featureConstraint.end(), featureConstraint.begin(), ::tolower);

  if(featureConstraint == "basic") return EFeatureConstraint::BASIC;
  if(featureConstraint == "scale") return EFeatureConstraint::SCALE;

  throw std::out_of_range("Invalid ESfMobservationConstraint: " + m);
}

inline std::ostream& operator<<(std::ostream& os, EFeatureConstraint m)
{
    return os << ESfMobservationConstraint_enumToString(m);
}

inline std::istream& operator>>(std::istream& in, EFeatureConstraint& m)
{
    std::string token;
    in >> token;
    m = ESfMobservationConstraint_stringToEnum(token);
    return in;
}

class BundleAdjustment
{
public:

  /**
   * @brief Defines all the types of parameter adjusted during bundle adjustment.
   */
  enum class EParameter : std::uint8_t
  {
    POSE = 0,      //< The pose
    INTRINSIC = 1, //< The intrinsic
    LANDMARK = 2   //< The landmark
  };

  /**
   * @brief Defines the state of the all parameter of the reconstruction during bundle adjustment.
   */
  enum class EParameterState : std::uint8_t
  {
    REFINED = 0,  //< will be adjusted by the BA solver
    CONSTANT = 1, //< will be set as constant in the sover
    IGNORED = 2   //< will not be set into the BA solver
  };

  /**
   * @brief Defines all the refine options that can be used in a bundle adjustment.
   */
  enum ERefineOptions
  {
    REFINE_NONE = 0,
    REFINE_ROTATION = 1,                                  //< refine pose rotations
    REFINE_TRANSLATION = 2,                               //< refine pose translations
    REFINE_STRUCTURE = 4,                                 //< refine structure (i.e. 3D points)
    REFINE_INTRINSICS_FOCAL = 8,                          //< refine the focal length
    REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS = 16,          //< refine the optical offset from the center
    REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA = 32,  //< refine the optical offset only if we have a minimum number of cameras
    REFINE_INTRINSICS_DISTORTION = 64,                    //< refine the distortion parameters
    /// Refine all intrinsics parameters
    REFINE_INTRINSICS_ALL = REFINE_INTRINSICS_FOCAL | REFINE_INTRINSICS_OPTICALOFFSET_IF_ENOUGH_DATA | REFINE_INTRINSICS_DISTORTION,
    /// Refine all parameters
    REFINE_ALL = REFINE_ROTATION | REFINE_TRANSLATION | REFINE_INTRINSICS_ALL | REFINE_STRUCTURE,
  };

  /**
   * @brief Perform a Bundle Adjustment on the SfM scene with refinement of the requested parameters
   * @param[in,out] sfmData The input SfMData contains all the information about the reconstruction
   * @param[in] refineOptions: choose what you want to refine
   * @return false if the bundle adjustment failed else true
   */
  virtual bool adjust(sfmData::SfMData& sfmData, ERefineOptions refineOptions = REFINE_ALL) = 0;

  // TODO: Use filter to say wich parameter is const or not (allow to refine only a subpart of the intrinsics or the poses)
};

BOOST_BITMASK(BundleAdjustment::ERefineOptions)

} // namespace sfm
} // namespace aliceVision
