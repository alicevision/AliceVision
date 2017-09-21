// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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

