// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <string>

namespace aliceVision {
namespace sfmDataIO {

/**
 * @brief Return the path of the companion binary landmarks payload file for a given USD file.
 * @param mainUsdFilename Path to the main .usd/.usda/.usdc file.
 * @return Path of the companion _landmarks.usdc file in the same directory.
 */
std::string landmarksPayloadPath(const std::string& mainUsdFilename);

/**
 * @brief Save SfMData to USD file format.
 * @param sfmData Input scene.
 * @param filename Output .usd/.usda/.usdc file path used for writing (can be temporary).
 * @param finalFilename Final .usd/.usda/.usdc file path used to author stable payload references.
 * @param partFlag Requested scene parts.
 * @return true if export succeeded.
 */
bool saveUSD(const sfmData::SfMData& sfmData,
			 const std::string& filename,
			 const std::string& finalFilename,
			 ESfMData partFlag);

/**
 * @brief Load SfMData from USD file format.
 * @param sfmData Output scene.
 * @param filename Input .usd/.usda/.usdc file.
 * @param partFlag Requested scene parts.
 * @return true if import succeeded.
 */
bool loadUSD(sfmData::SfMData& sfmData, const std::string& filename, ESfMData partFlag);

}  // namespace sfmDataIO
}  // namespace aliceVision
