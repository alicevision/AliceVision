// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/SfMData.hpp>
#include <aliceVision/sfm/sfmDataIO.hpp>

#include <string>

namespace aliceVision {
namespace sfm {

class AlembicImporter 
{
public:
  explicit AlembicImporter(const std::string &filename);
  ~AlembicImporter();

  void populate(sfm::SfMData &sfmdata, sfm::ESfMData flags_part = sfm::ESfMData::ALL);

private:
  
  struct DataImpl;
  std::unique_ptr<DataImpl> _objImpl;
};

} // namespace sfm
} // namespace aliceVision

