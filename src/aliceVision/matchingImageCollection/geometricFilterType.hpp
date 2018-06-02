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
  HOMOGRAPHY_GROWING,
  NO_FILTERING
};

std::string EGeometricFilterType_informations()
{
  return "Pairwise correspondences filtering thanks to robust model estimation:\n"
         "* f  - FUNDAMENTAL_MATRIX:  fundamental matrix\n"
         "* e  - ESSENTIAL_MATRIX:    essential matrix\n"
         "* h  - HOMOGRAPHY_MATRIX:   homography matrix\n"
         "* hg - HOMOGRAPHY_GROWING:  multiple homography matrices [F.Srajer, 2016]\n"
         "* none - NO_FILTERING:      no geometric filtering\n";
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
    case EGeometricFilterType::NO_FILTERING: return "NO_FLITERING";
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
  if(geometricFilterType == "FUNDAMENTAL_MATRIX" || geometricFilterType == "f")  
    return EGeometricFilterType::FUNDAMENTAL_MATRIX;
  if(geometricFilterType == "ESSENTIAL_MATRIX" || geometricFilterType == "e")    
    return EGeometricFilterType::ESSENTIAL_MATRIX;
  if(geometricFilterType == "HOMOGRAPHY_MATRIX" || geometricFilterType == "h")   
    return EGeometricFilterType::HOMOGRAPHY_MATRIX;
  if (geometricFilterType == "HOMOGRAPHY_GROWING" || geometricFilterType == "hg")
      return EGeometricFilterType::HOMOGRAPHY_GROWING;
  if (geometricFilterType == "NO_FILTERING" || geometricFilterType == "none")
      return EGeometricFilterType::NO_FILTERING;
  throw std::out_of_range("Invalid geometricFilterType : " + geometricFilterType);
}

/**
 * @brief converte a geometricFilterType name to a short notation.
 * @param[in] geometricFilterType long or short name
 * @return geometricFilterType short notation
 */
std::string shortNotation(const std::string& geometricFilterType)
{
  if(geometricFilterType == "FUNDAMENTAL_MATRIX" || geometricFilterType == "f")  return "f"; 
  if(geometricFilterType == "ESSENTIAL_MATRIX" || geometricFilterType == "e")    return "e";
  if(geometricFilterType == "HOMOGRAPHY_MATRIX" || geometricFilterType == "h")   return "h";
  if(geometricFilterType == "HOMOGRAPHY_GROWING" || geometricFilterType == "hg") return "hg";
  if(geometricFilterType == "NO_FILTERING" || geometricFilterType == "none")     return "none";
  throw std::out_of_range("Invalid geometricFilterType : " + geometricFilterType);
}

/**
 * @brief converte a geometricFilterType name to a long notation.
 * @param[in] geometricFilterType long or short name
 * @return geometricFilterType long notation
 */
std::string longNotation(const std::string& geometricFilterType)
{
  if(geometricFilterType == "FUNDAMENTAL_MATRIX" || geometricFilterType == "f")  return "FUNDAMENTAL_MATRIX"; 
  if(geometricFilterType == "ESSENTIAL_MATRIX" || geometricFilterType == "e")    return "ESSENTIAL_MATRIX";
  if(geometricFilterType == "HOMOGRAPHY_MATRIX" || geometricFilterType == "h")   return "HOMOGRAPHY_MATRIX";
  if (geometricFilterType == "HOMOGRAPHY_GROWING" || geometricFilterType == "hg") return "HOMOGRAPHY_GROWING";
  if (geometricFilterType == "NO_FILTERING" || geometricFilterType == "none") return "NO_FILTERING";
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

