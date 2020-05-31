// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>

#include <string>
#include <iostream>

namespace aliceVision {
namespace robustEstimation {

enum class ERobustEstimator
{
  START = 0,
  ACRANSAC = 1,        //< A-Contrario Ransac.
  RANSAC = 2,          //< Classic Ransac.
  LSMEDS = 3,          //< Variant of RANSAC using Least Median of Squares.
  LORANSAC = 4,        //< LO-Ransac.
  MAXCONSENSUS = 5,    //< Naive implementation of RANSAC without noise and iteration reduction options.
  END
};

inline std::string ERobustEstimator_enumToString(ERobustEstimator estimator)
{
  switch(estimator)
  {
    case ERobustEstimator::ACRANSAC:
      return "acransac";
    case ERobustEstimator::RANSAC:
      return "ransac";
    case ERobustEstimator::LSMEDS:
      return "lsmeds";
    case ERobustEstimator::LORANSAC:
      return "loransac";
    case ERobustEstimator::MAXCONSENSUS:
      return "maxconsensus";
    case ERobustEstimator::START:
    case ERobustEstimator::END:
      break;
  }
  throw std::out_of_range("Invalid Ransac type Enum");
}

inline ERobustEstimator ERobustEstimator_stringToEnum(const std::string& estimator)
{
  if(estimator == "acransac")
    return ERobustEstimator::ACRANSAC;
  if(estimator == "ransac")
    return ERobustEstimator::RANSAC;
  if(estimator == "lsmeds")
    return ERobustEstimator::LSMEDS;
  if(estimator == "loransac")
    return ERobustEstimator::LORANSAC;
  if(estimator == "maxconsensus")
    return ERobustEstimator::MAXCONSENSUS;
  throw std::out_of_range("Invalid Ransac type string " + estimator);
}

inline std::ostream& operator<<(std::ostream& os, ERobustEstimator e)
{
    return os << ERobustEstimator_enumToString(e);
}

inline std::istream& operator>>(std::istream& in, ERobustEstimator& estimatorType)
{
    std::string token;
    in >> token;
    estimatorType = robustEstimation::ERobustEstimator_stringToEnum(token);
    return in;
}

/**
 * @brief It adjust the value for the reprojection/matching error
 * to ensure the compatibility with the given robust estimator.
 * The value 0 will be converted to a default value:
 * - LORansac: use defaultLoRansac parameter
 * - ACRansac: use infinity (ie estimate the threshold during ransac process)
 *
 * @param e The estimator to be checked.
 * @param value The value for the reprojection or matching error.
 * @return true if the value is compatible
 */
inline bool adjustRobustEstimatorThreshold(ERobustEstimator e, double &value, double defaultLoRansac)
{
  if(e != ERobustEstimator::LORANSAC &&
     e != ERobustEstimator::ACRANSAC)
  {
    ALICEVISION_CERR("Only " << ERobustEstimator::ACRANSAC
            << " and " << ERobustEstimator::LORANSAC
            << " are supported.");
    return false;
  }
  if(value == 0)
  {
    if (e == ERobustEstimator::ACRANSAC)
    {
      // for acransac set it to infinity
      value = std::numeric_limits<double>::infinity();
    }
    else if(e == ERobustEstimator::LORANSAC)
    {
      // for loransac we need a threshold > 0
      value = defaultLoRansac;
    }
  }

  return true;
}

} //namespace robustEstimation
} //namespace aliceVision
