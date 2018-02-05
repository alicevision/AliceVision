// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sensorDB/Datasheet.hpp>

#include <vector>
#include <string>

namespace aliceVision {
namespace sensorDB {

/**
 * @brief Parse the given sensor database
 * @param[in] databaseFilePath The file path of the given database
 * @param[out] databaseStructure The database in memory
 * @return True if ok
 */
bool parseDatabase(const std::string& databaseFilePath, std::vector<Datasheet>& databaseStructure);

/**
 * @brief Get information for the given camera brand / model
 * @param[in] brand The camera brand
 * @param[in] model The camera model
 * @param[in] databaseStructure The database in memory
 * @param[out] datasheetContent The corresponding datasheet
 * @return True if ok
 */
bool getInfo(const std::string& brand, const std::string& model, const std::vector<Datasheet>& databaseStructure, Datasheet& datasheetContent);

} // namespace sensorDB
} // namespace aliceVision
