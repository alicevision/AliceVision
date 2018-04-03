// This file is part of the AliceVision project.
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

enum class EGeometricModel
{
  FUNDAMENTAL_MATRIX   = 0
  , ESSENTIAL_MATRIX   = 1
  , HOMOGRAPHY_MATRIX  = 2
};

/**
 * @brief get informations about each geometric model
 * @return String
 */
std::string EGeometricModel_informations()
{
  return "Geometric model: Pairwise correspondences filtering thanks to robust model estimation:\n"
         "* fundamental_matrix\n"
         "* essential_matrix\n"
         "* homography_matrix\n";
}

/**
 * @brief convert an enum EGeometricModel to its corresponding string
 * @param EGeometricModel
 * @return String
 */
inline std::string EGeometricModel_enumToString(EGeometricModel geometricModel)
{
  switch(geometricModel)
  {
    case EGeometricModel::FUNDAMENTAL_MATRIX:    return "fundamental_matrix";
    case EGeometricModel::ESSENTIAL_MATRIX:      return "essential_matrix";
    case EGeometricModel::HOMOGRAPHY_MATRIX:     return "homography_matrix";
  }
  throw std::out_of_range("Invalid sharpnessPreset enum");
}

/**
 * @brief convert a string geometricModel to its corresponding enum EGeometricModel
 * @param String
 * @return EGeometricModel
 */
inline EGeometricModel EGeometricModel_stringToEnum(const std::string& geometricModel)
{
  std::string model = geometricModel;
  std::transform(model.begin(), model.end(), model.begin(), ::tolower); //tolower

  if(model == "fundamental_matrix")    return EGeometricModel::FUNDAMENTAL_MATRIX;
  if(model == "essential_matrix")      return EGeometricModel::ESSENTIAL_MATRIX;
  if(model == "homography_matrix")     return EGeometricModel::HOMOGRAPHY_MATRIX;

  throw std::out_of_range("Invalid sharpnessPreset : " + geometricModel);
}

/**
 * @brief EImageDescriberType_stringToEnums
 * @param describerMethods
 * @return EImageDescriberType vector
 */
std::vector<EGeometricModel> EGeometricModel_stringToEnums(const std::string& geometricModels);

inline std::ostream& operator<<(std::ostream& os, const EGeometricModel geometricModel)
{
  os << EGeometricModel_enumToString(geometricModel);
  return os;
}

inline std::istream& operator>>(std::istream& in, EGeometricModel& geometricModel)
{
  std::string token;
  in >> token;
  geometricModel = EGeometricModel_stringToEnum(token);
  return in;
}

inline std::ostream& operator<<(std::ostream& os, const std::vector<EGeometricModel>& geometricModels)
{
  for(const EGeometricModel descType : geometricModels)
    os << descType;
  return os;
}

inline std::istream& operator>>(std::istream& in, std::vector<EGeometricModel>& geometricModels)
{
  std::string token;
  in >> token;
  geometricModels = EGeometricModel_stringToEnums(token);
  return in;
}

} // namespace matchingImageCollection
} // namespace aliceVision
