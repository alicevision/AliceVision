// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

namespace aliceVision {
namespace matchingImageCollection {

enum EGeometricFilterType
{
  FUNDAMENTAL_MATRIX,
  ESSENTIAL_MATRIX,
  HOMOGRAPHY_MATRIX,
  HOMOGRAPHY_GROWING 
};

std::string EGeometricFilterType_informations()
{
  return "Pairwise correspondences filtering thanks to robust model estimation:\n"
         "* FUNDAMENTAL_MATRIX: fundamental matrix\n"
         "* ESSENTIAL_MATRIX: essential matrix\n"
         "* HOMOGRAPHY_MATRIX: homography matrix\n"
         "* HOMOGRAPHY_GROWING: multiple homography matrices [F.Srajer, 2016]\n";
}

/**
 * @brief convert an enum EGeometricFilterType to its corresponding string
 * @param EGeometricFilterType
 * @return geometricFilterType string
 */
std::string EGeometricFilterType_enumToString(const EGeometricFilterType geometricFilterType)
{
  switch(geometricFilterType)
  {
    case EGeometricFilterType::FUNDAMENTAL_MATRIX: return "FUNDAMENTAL_MATRIX";
    case EGeometricFilterType::ESSENTIAL_MATRIX:   return "ESSENTIAL_MATRIX";
    case EGeometricFilterType::HOMOGRAPHY_MATRIX:  return "HOMOGRAPHY_MATRIX";
    case EGeometricFilterType::HOMOGRAPHY_GROWING: return "HOMOGRAPHY_GROWING";
  }
  throw std::out_of_range("Invalid geometricFilterType enum");
}

/**
 * @brief convert a string geometricFilterType to it's corresponding enum EGeometricFilterType
 * @param geometricFilterType string
 * @return EGeometricFilterType
 */
EGeometricFilterType EGeometricFilterType_stringToEnum(const std::string& geometricFilterType)
{
  if(geometricFilterType == "FUNDAMENTAL_MATRIX")  return EGeometricFilterType::FUNDAMENTAL_MATRIX;
  if(geometricFilterType == "ESSENTIAL_MATRIX")    return EGeometricFilterType::ESSENTIAL_MATRIX;
  if(geometricFilterType == "HOMOGRAPHY_MATRIX")   return EGeometricFilterType::HOMOGRAPHY_MATRIX;
  if(geometricFilterType == "HOMOGRAPHY_GROWING")  return EGeometricFilterType::HOMOGRAPHY_GROWING;
  throw std::out_of_range("Invalid geometricFilterType : " + geometricFilterType);
}

/**
 * @brief converte a geometricFilterType name to a short notation.
 * @param geometricFilterType standart/long name
 * @return geometricFilterType short notation
 */
std::string fromLongToShortNotation(const std::string& geometricFilterType)
{
  if(geometricFilterType == "FUNDAMENTAL_MATRIX")  return "f"; 
  if(geometricFilterType == "ESSENTIAL_MATRIX")    return "e";
  if(geometricFilterType == "HOMOGRAPHY_MATRIX")   return "h";
  if(geometricFilterType == "HOMOGRAPHY_GROWING")  return "hg";
  throw std::out_of_range("Invalid geometricFilterType : " + geometricFilterType);
}

inline std::ostream& operator<<(std::ostream& os, const EGeometricFilterType geometricFilterType)
{
  os << EGeometricFilterType_enumToString(geometricFilterType);
  return os;
}

inline std::istream& operator>>(std::istream& in, EGeometricFilterType &geometricFilterType)
{
  std::string token;
  in >> token;
  geometricFilterType = EGeometricFilterType_stringToEnum(token);
  return in;
}

} // namespace matchingImageCollection
} // namespace aliceVision

