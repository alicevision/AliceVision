// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <cereal/cereal.hpp>
#include <cereal/types/polymorphic.hpp>

namespace aliceVision {
namespace sfmDataIO {

bool saveCerealJSON(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);
bool loadCerealJSON(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

bool saveCerealBinary(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);
bool loadCerealBinary(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);


}  // namespace sfmDataIO
}  // namespace aliceVision


CEREAL_FORCE_DYNAMIC_INIT(cerealIO)