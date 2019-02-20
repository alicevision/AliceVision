// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Datasheet.hpp"

#include <string>
#include <algorithm>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/predicate.hpp>

namespace aliceVision {
namespace sensorDB {

bool Datasheet::operator==(const Datasheet& other) const
{
  std::string brandA = _brand;
  std::string brandB = other._brand;

  boost::algorithm::to_lower(brandA);
  boost::algorithm::to_lower(brandB);

  brandA.erase(std::remove_if(brandA.begin(), brandA.end(), ::ispunct), brandA.end()); //remove punctuation
  brandB.erase(std::remove_if(brandB.begin(), brandB.end(), ::ispunct), brandB.end()); //remove punctuation

  brandA.erase(std::remove_if(brandA.begin(), brandA.end(), ::isspace), brandA.end()); //remove spaces
  brandB.erase(std::remove_if(brandB.begin(), brandB.end(), ::isspace), brandB.end()); //remove spaces

  if((brandA == brandB) ||
     (boost::algorithm::starts_with(brandA, brandB)) ||
     (boost::algorithm::starts_with(brandB, brandA)))
  {
    std::string modelA = _model;
    std::string modelB = other._model;

    boost::algorithm::to_lower(modelA);
    boost::algorithm::to_lower(modelB);

    modelA.erase(std::remove_if(modelA.begin(), modelA.end(), ::ispunct), modelA.end()); //remove punctuation
    modelB.erase(std::remove_if(modelB.begin(), modelB.end(), ::ispunct), modelB.end()); //remove punctuation

    modelA.erase(std::remove_if(modelA.begin(), modelA.end(), ::isspace), modelA.end()); //remove spaces
    modelB.erase(std::remove_if(modelB.begin(), modelB.end(), ::isspace), modelB.end()); //remove spaces

    if((modelA == modelB) ||
       (boost::algorithm::ends_with(modelA, modelB)) ||
       (boost::algorithm::ends_with(modelB, modelA)))
      return true;
  }

  return false;
}

} // namespace sensorDB
} // namespace aliceVision
