// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#pragma once

#include <aliceVision/sfm/sfmDataIO.hpp>

namespace aliceVision {
namespace sfm {

bool Generate_SfM_Report(const SfMData & sfm_data, const std::string & htmlFilename);


} // namespace sfm
} // namespace aliceVision
