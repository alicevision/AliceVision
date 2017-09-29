// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

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

inline std::istream& operator>>(std::istream& in, robustEstimation::ERobustEstimator& estimatorType)
{
    std::string token;
    in >> token;
    estimatorType = robustEstimation::ERobustEstimator_stringToEnum(token);
    return in;
}

} //namespace robustEstimation
} //namespace aliceVision
