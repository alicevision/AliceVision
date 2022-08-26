// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "parseDatabase.hpp"
#include <aliceVision/sensorDB/Datasheet.hpp>
#include <aliceVision/vfs/filesystem.hpp>
#include <aliceVision/vfs/istream.hpp>

#include <boost/algorithm/string.hpp>

#include <vector>
#include <string>
#include <algorithm>
#include <iterator>

namespace aliceVision {
namespace sensorDB {

bool parseDatabase(const std::string& databaseFilePath, std::vector<Datasheet>& databaseStructure)
{
  vfs::istream fileIn(databaseFilePath);
  if (!fileIn || !vfs::exists(databaseFilePath) || !vfs::is_regular_file(databaseFilePath))
    return false;

  std::string line;
  while(fileIn.good())
  {
    getline( fileIn, line);
    if(!line.empty())
    {
      if(line[0] != '#')
      {
        std::vector<std::string> values;
        boost::split(values, line, boost::is_any_of(";"));

        if(values.size() >= 4)
        {
          const std::string brand = values[0];
          const std::string model = values[1];
          const double sensorWidth = std::stod(values[2]);
          databaseStructure.emplace_back(brand, model, sensorWidth);
        }
      }
    }
  }
  return true;
}

bool getInfo(const std::string& brand, const std::string& model, const std::vector<Datasheet>& databaseStructure, Datasheet& datasheetContent)
{
  Datasheet refDatasheet(brand, model, -1.);
  auto datasheet = std::find(databaseStructure.begin(), databaseStructure.end(), refDatasheet);

  if(datasheet == databaseStructure.end())
    return false;

  datasheetContent = *datasheet;
  return true;
}

} // namespace sensorDB
} // namespace aliceVision
