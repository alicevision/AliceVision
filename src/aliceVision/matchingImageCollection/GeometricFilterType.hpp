// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>

namespace aliceVision {
namespace matchingImageCollection {

enum class EGeometricFilterType
{
  NO_FILTERING
  , FUNDAMENTAL_MATRIX
  , FUNDAMENTAL_WITH_DISTORTION
  , ESSENTIAL_MATRIX
  , HOMOGRAPHY_MATRIX
  , HOMOGRAPHY_GROWING
};

/**
 * @brief Get informations about each geometric filter type
 * @return informations String
 */
inline std::string EGeometricFilterType_informations()
{
  return "Geometric filter type: Pairwise correspondences filtering thanks to robust model estimation:\n"
         "* no_filtering:                no geometric filtering\n"
         "* fundamental_matrix:          fundamental matrix\n"
         "* fundamental_with_distortion: fundamental matrix with F10 solver [Z.Kukelova, 2015]\n"
         "* essential_matrix:            essential matrix\n"
         "* homography_matrix:           homography matrix\n"
         "* homography_growing:          multiple homography matrices [F.Srajer, 2016]\n";
}

/**
 * @brief Convert an enum EGeometricFilterType to its corresponding string
 * @param[in] EGeometricFilterType enum
 * @return corresponding string
 */
inline std::string EGeometricFilterType_enumToString(EGeometricFilterType geometricFilterType)
{
  switch(geometricFilterType)
  {
    case EGeometricFilterType::NO_FILTERING:                 return "no_filtering";
    case EGeometricFilterType::FUNDAMENTAL_MATRIX:           return "fundamental_matrix";
    case EGeometricFilterType::FUNDAMENTAL_WITH_DISTORTION:  return "fundamental_with_distortion";
    case EGeometricFilterType::ESSENTIAL_MATRIX:             return "essential_matrix";
    case EGeometricFilterType::HOMOGRAPHY_MATRIX:            return "homography_matrix";
    case EGeometricFilterType::HOMOGRAPHY_GROWING:           return "homography_growing";
  }
  throw std::out_of_range("Invalid geometricFilterType enum");
}

/**
 * @brief Convert a string geometricFilterType to its corresponding enum EGeometricFilterType
 * @param[in] String corresponding to an EGeometricFilterType enum
 * @return corresponding EGeometricFilterType
 */
inline EGeometricFilterType EGeometricFilterType_stringToEnum(const std::string& geometricFilterType)
{
  std::string model = geometricFilterType;
  std::transform(model.begin(), model.end(), model.begin(), ::tolower); //tolower

  if(model == "no_filtering")                 return EGeometricFilterType::NO_FILTERING;
  if(model == "fundamental_matrix")           return EGeometricFilterType::FUNDAMENTAL_MATRIX;
  if(model == "fundamental_with_distortion")  return EGeometricFilterType::FUNDAMENTAL_WITH_DISTORTION;
  if(model == "essential_matrix")             return EGeometricFilterType::ESSENTIAL_MATRIX;
  if(model == "homography_matrix")            return EGeometricFilterType::HOMOGRAPHY_MATRIX;
  if(model == "homography_growing")           return EGeometricFilterType::HOMOGRAPHY_GROWING;

  throw std::out_of_range("Invalid geometricFilterType: " + geometricFilterType);
}

inline std::ostream& operator<<(std::ostream& os, const EGeometricFilterType geometricFilterType)
{
  os << EGeometricFilterType_enumToString(geometricFilterType);
  return os;
}

inline std::istream& operator>>(std::istream& in, EGeometricFilterType& geometricFilterType)
{
  std::string token;
  in >> token;
  geometricFilterType = EGeometricFilterType_stringToEnum(token);
  return in;
}

} // namespace matchingImageCollection
} // namespace aliceVision
