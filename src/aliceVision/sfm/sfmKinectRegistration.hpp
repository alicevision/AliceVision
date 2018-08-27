// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/system/Logger.hpp>
#include "aliceVision/image/all.hpp"

#include <algorithm>

namespace aliceVision {
	namespace sfm {

		struct ipo {
			IndexT _pt3D_id;
			Vec2 _obs;
			ipo(IndexT pt3D_id, Vec2 obs) : _pt3D_id(pt3D_id), _obs(obs) {}
		};

		
		/**
		* @brief Compute the scale for SfM from Kinect depthmaps.
		*
		* @param[in] sfmData
		* @param[in] depthmapsPath path to the folder with depthmaps registered to Kinect HD images
		* @param[out] out_S output scale factor for SfM to be in millimeters 
		*
		* Kinect points in 3D can be transformed to the world coordinate system in millimeters: 
		*			X = sfm.view.rotation.transpose() * kinect.X + out_S * sfm.view.camera_center
		*/
		bool computeSfMScaleFromKinect(const sfmData::SfMData& sfmData,
			const std::string depthmapsPath,
			double* out_S);

	}
}