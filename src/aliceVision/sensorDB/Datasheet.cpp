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

  for ( std::vector<std::string>::const_iterator iter_brand = vec_brand.begin();
          iter_brand != vec_brand.end();
          ++iter_brand )
  {
    std::string brandlower2 = *iter_brand;
    std::transform(brandlower2.begin(), brandlower2.end(),
      brandlower2.begin(), ::tolower);
    //ALICEVISION_LOG_DEBUG(brandlower << "\t" << brandlower2);
    if ( brandlower.compare( brandlower2 ) == 0 )
    {
      std::vector<std::string> vec_model1;
      boost::split(vec_model1, ds._model, boost::is_any_of(" "));
      std::vector<std::string> vec_model2;
      boost::split(vec_model2, _model, boost::is_any_of(" "));
      bool isAllFind = true;
      for ( std::vector<std::string>::const_iterator iter_model1 = vec_model1.begin();
          iter_model1 != vec_model1.end();
          ++iter_model1 )
      {
        bool hasDigit = false;
        for(std::string::const_iterator c = (*iter_model1).begin(); c != (*iter_model1).end(); ++c )
        {
          if(isdigit(*c))
          {
            hasDigit = true;
            break;
          }
        }
        if ( hasDigit )
        {
          std::string modellower1 = *iter_model1;
          for ( int index = 0; index < modellower1.length(); index++ )
          {
            modellower1[index] = tolower(modellower1[index]);
          }
          bool isFind = false;
          for ( std::vector<std::string>::const_iterator iter_model2 = vec_model2.begin();
                iter_model2 != vec_model2.end();
                ++iter_model2 )
          {
            std::string modellower2 = *iter_model2;
            for ( int index = 0; index < modellower2.length(); index++ )
            {
              modellower2[index] = tolower(modellower2[index]);
            }
            if ( modellower2.compare( modellower1 ) == 0 )
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
