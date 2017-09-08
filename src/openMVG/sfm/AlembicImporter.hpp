// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/sfm/sfm_data.hpp>
#include <aliceVision/sfm/sfm_data_io.hpp>

#include <string>

namespace aliceVision {
namespace sfm {

class AlembicImporter 
{
public:
  explicit AlembicImporter(const std::string &filename);
  ~AlembicImporter();

  void populate(sfm::SfM_Data &sfmdata, sfm::ESfM_Data flags_part = sfm::ESfM_Data::ALL);

private:
  
  struct DataImpl;
  std::unique_ptr<DataImpl> _objImpl;
};

} // namespace sfm
} // namespace aliceVision

