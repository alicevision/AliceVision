// Copyright (c) 2015 cpichard.
//
// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
      
#pragma once

#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>

#include <string>

namespace openMVG {
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
} // namespace openMVG

