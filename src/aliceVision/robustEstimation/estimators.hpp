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

inline std::istream& operator>>(std::istream& in, robustEstimation::ERobustEstimator& estimatorType)
{
    std::string token;
    in >> token;
    estimatorType = robustEstimation::ERobustEstimator_stringToEnum(token);
    return in;
}

/**
 * @brief It checks if the value for the reprojection error or the matching error
 * is compatible with the given robust estimator. The value cannot be 0 for 
 * LORansac, for ACRansac a value of 0 means to use infinity (ie estimate the 
 * threshold during ransac process)
 * @param e The estimator to be checked.
 * @param value The value for the reprojection or matching error.
 * @return true if the value is compatible
 */
inline bool checkRobustEstimator(ERobustEstimator e, double &value)
{
  if(e != ERobustEstimator::LORANSAC &&
     e != ERobustEstimator::ACRANSAC)
  {
    ALICEVISION_CERR("Only " << ERobustEstimator::ACRANSAC
            << " and " << ERobustEstimator::LORANSAC
            << " are supported.");
    return false;
  }
  if(value == 0 && 
     e == ERobustEstimator::ACRANSAC)
  {
    // for acransac set it to infinity
    value = std::numeric_limits<double>::infinity();
  }
  // for loransac we need thresholds > 0
  if(e == ERobustEstimator::LORANSAC)
  {
    const double minThreshold = 1e-6;
    if( value <= minThreshold)
    {
      ALICEVISION_CERR("Error: errorMax and matchingError cannot be 0 with " 
              << ERobustEstimator::LORANSAC
              << " estimator.");
      return false;     
    }
  }

  return true;
}

} //namespace robustEstimation
} //namespace aliceVision
