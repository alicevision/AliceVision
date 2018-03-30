// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Datasheet.hpp"

#include <boost/algorithm/string.hpp>

#include <iterator>
#include <algorithm>
#include <vector>

namespace aliceVision {
namespace sensorDB {

bool Datasheet::operator==(const Datasheet& ds) const
{
  bool isEqual = false;
  std::vector<std::string> vec_brand;
  boost::split(vec_brand, ds._brand, boost::is_any_of(" "));

  std::string brandlower = _brand;
  boost::algorithm::to_lower(brandlower);

  for(const auto& brand : vec_brand)
  {
    std::string brandlower2 = brand;
    boost::algorithm::to_lower(brandlower2);

    //ALICEVISION_LOG_DEBUG(brandlower << "\t" << brandlower2);
    if (brandlower == brandlower2)
    {
      std::vector<std::string> vec_model1;
      boost::split(vec_model1, ds._model, boost::is_any_of(" "));
      std::vector<std::string> vec_model2;
      boost::split(vec_model2, _model, boost::is_any_of(" "));
      bool isAllFound = true;

      for(const auto& model1 : vec_model1)
      {
        if(!std::any_of(model1.begin(), model1.end(), ::isdigit))
        {
          continue;
        }

        std::string modellower1 = model1;
        boost::algorithm::to_lower(modellower1);

        bool isFound = false;
        for(const auto& model2 : vec_model2)
        {
          std::string modellower2 = model2;
          boost::algorithm::to_lower(modellower2);

          if (modellower2 == modellower1)
          {
            isFound = true;
          }
        }
        if ( !isFound )
        {
          isAllFound = false;
          break;
        }
      }
      if ( isAllFound )
        isEqual = true;
    }
  }
  return isEqual;
}

} // namespace sensorDB
} // namespace aliceVision
