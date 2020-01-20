// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <iostream>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace sfm {

enum ERotationAveragingMethod
{
  ROTATION_AVERAGING_L1 = 1,
  ROTATION_AVERAGING_L2 = 2
};

enum ERelativeRotationInferenceMethod
{
  TRIPLET_ROTATION_INFERENCE_NONE = 0,
  TRIPLET_ROTATION_INFERENCE_COMPOSITION_ERROR = 1
};

inline std::string ERotationAveragingMethod_enumToString(ERotationAveragingMethod eRotationAveragingMethod)
{
  switch(eRotationAveragingMethod)
  {
    case ERotationAveragingMethod::ROTATION_AVERAGING_L1:
      return "L1_minimization";
    case ERotationAveragingMethod::ROTATION_AVERAGING_L2:
      return "L2_minimization";
  }
  throw std::out_of_range("Invalid rotation averaging method type");
}

inline ERotationAveragingMethod ERotationAveragingMethod_stringToEnum(const std::string& RotationAveragingMethodName)
{
  if(RotationAveragingMethodName == "L1_minimization")      return ERotationAveragingMethod::ROTATION_AVERAGING_L1;
  if(RotationAveragingMethodName == "L2_minimization")   return ERotationAveragingMethod::ROTATION_AVERAGING_L2;

  throw std::out_of_range("Invalid rotation averaging method name : '" + RotationAveragingMethodName + "'");
}

inline std::ostream& operator<<(std::ostream& os, ERotationAveragingMethod e)
{
    return os << ERotationAveragingMethod_enumToString(e);
}

inline std::istream& operator>>(std::istream& in, ERotationAveragingMethod& rotationType)
{
    std::string token;
    in >> token;
    rotationType = ERotationAveragingMethod_stringToEnum(token);
    return in;
}

} // namespace sfm
} // namespace aliceVision

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/graph/graph.hpp>
#include <aliceVision/multiview/rotationAveraging/common.hpp>

namespace aliceVision {
namespace sfm {

class GlobalSfMRotationAveragingSolver
{
private:
  /// pair that are considered as valid by the rotation averaging solver
  mutable PairSet used_pairs;

public:
  bool Run(ERotationAveragingMethod eRotationAveragingMethod,
           ERelativeRotationInferenceMethod eRelativeRotationInferenceMethod,
           const rotationAveraging::RelativeRotations& relativeRot_In,
           const double max_angular_error,
           HashMap<IndexT, Mat3>& map_globalR) const;

  /**
   * @brief Reject edges of the view graph that do not produce triplets with tiny
   * angular error once rotation composition have been computed.
   */
  void TripletRotationRejection(const double max_angular_error,
                                std::vector<graph::Triplet>& vec_triplets,
                                rotationAveraging::RelativeRotations& relativeRotations) const;
  /**
   * @brief Return the pairs validated by the GlobalRotation routine (inference can remove some)
   * @return pairs validated by the GlobalRotation routine
   */
  PairSet GetUsedPairs() const;
};

} // namespace sfm
} // namespace aliceVision
