// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "IFeed.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/vfs/filesystem.hpp>
#include <aliceVision/vfs/istream.hpp>

#include <exception>

namespace aliceVision{
namespace dataio{

// the structure of the file is
// int #image width
// int #image height
// double #focal
// double #ppx principal point x-coord
// double #ppy principal point y-coord
// double #k0
// double #k1
// double #k2
void readCalibrationFromFile(vfs::filesystem& fs, const std::string &filename,
                             camera::PinholeRadialK3 &camIntrinsics)
{
  auto stream = fs.open_read_text(filename);
  if (!stream.is_open())
  {
    ALICEVISION_LOG_WARNING("Unable to open the calibration file " << filename);
    throw std::invalid_argument("Unable to open the calibration file "+filename);
  }
  int width = 0;
  int height = 0;
  const size_t numParam = 6;
  std::vector<double> params(numParam, 0);
  
  stream >> width;
  stream >> height;
  for(size_t i = 0; i < numParam; ++i)
  {
    stream >> params[i];
  }
  camIntrinsics = camera::PinholeRadialK3(width, height, 
                                  params[0], params[1], params[2],
                                  params[3], params[4], params[5]);
  
  stream.close();
}

}//namespace dataio 
}//namespace aliceVision
