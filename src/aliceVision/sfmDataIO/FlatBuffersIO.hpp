// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

namespace aliceVision {
namespace sfmDataIO {

/**
 * @brief Load a FlatBuffers SfMData file.
 * @param[out] sfmData The output SfMData
 * @param[in] filename The filename
 * @param[in] partFlag The ESfMData load flag
 * @return true if completed
 */
bool loadFlatBuffers(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

/**
 * @brief Save an SfMData in a FlatBuffers file
 * @param[in] sfmData The input SfMData
 * @param[in] filename The filename
 * @param[in] partFlag The ESfMData save flag
 * @return true if completed
 */
bool saveFlatBuffers(const sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

}
}