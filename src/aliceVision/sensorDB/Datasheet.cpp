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

  std::transform(brandlower.begin(), brandlower.end(),
    brandlower.begin(), ::tolower);

  for(const auto& brand : vec_brand)
  {
    std::string brandlower2 = brand;
    std::transform(brandlower2.begin(), brandlower2.end(),
      brandlower2.begin(), ::tolower);
    //ALICEVISION_LOG_DEBUG(brandlower << "\t" << brandlower2);
    if (brandlower == brandlower2)
    {
      std::vector<std::string> vec_model1;
      boost::split(vec_model1, ds._model, boost::is_any_of(" "));
      std::vector<std::string> vec_model2;
      boost::split(vec_model2, _model, boost::is_any_of(" "));
      bool isAllFind = true;

      for(const auto& model1 : vec_model1)
      {
        bool hasDigit = false;
        for(const char& c : (model1))
        {
          if(isdigit(c))
          {
            hasDigit = true;
            break;
          }
        }
        if ( hasDigit )
        {
          std::string modellower1 = model1;
          for(char &ch : modellower1)
          {
            // the behavior of std::tolower is undefined if the argument's value is neither
            // representable as unsigned char nor equal to EOF. To use these functions safely
            // with plain chars (or signed chars), the argument should first be converted
            // to unsigned char (http://en.cppreference.com/w/cpp/string/byte/tolower)
            ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
          }
          bool isFind = false;
          for(const auto& model2 : vec_model2)
          {
            std::string modellower2 = model2;
            for (char &ch : modellower2)
            {
              ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            }
            if (modellower2 == modellower1)
            {
              isFind = true;
            }
          }
          if ( !isFind )
          {
            isAllFind = false;
            break;
          }
        }
      }
      if ( isAllFind )
        isEqual = true;
    }
  }
  return isEqual;
}

} // namespace sensorDB
} // namespace aliceVision
