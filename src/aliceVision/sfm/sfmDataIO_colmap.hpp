// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfm/sfmDataIO.hpp>

#include <string>

namespace aliceVision {
	namespace sfm {

		/**
		* @brief Load the structure and camera positions to a SfMData container.
		* @param[out] sfmData The input SfMData
		* @param[in] filename The filename
		* @return true if completed
		*/
		bool loadColmap(SfMData &sfmData,
			const std::string& input_dir);

	} // namespace sfm
} // namespace aliceVision