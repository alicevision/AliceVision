// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "GeometricModel.hpp"

#include <boost/algorithm/string.hpp>

namespace aliceVision {
namespace matchingImageCollection {

std::vector<EGeometricModel> EGeometricModel_stringToEnums(const std::string& geometricModels)
{
  std::vector<EGeometricModel> out;
  std::vector<std::string> geometricModelsVec;
  boost::split(geometricModelsVec, geometricModels, boost::is_any_of(","));

  for(const auto& geometricModel: geometricModelsVec)
  {
    out.push_back(EGeometricModel_stringToEnum(geometricModel));
  }
  return out;
}

} // namespace matchingImageCollection
} // namespace aliceVision
