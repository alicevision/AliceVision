// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE scaleFromKinect

#include <aliceVision/sfm/sfmKinectRegistration.hpp>

#include <aliceVision/system/Logger.hpp>
#include "aliceVision/image/all.hpp"
#include <aliceVision/sfm/sfm.hpp>
#include <boost/filesystem.hpp>

#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <sstream>
#include <cstdio>
#include <iostream>
#include <vector>
#include <string>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::sfmDataIO;
using namespace aliceVision::image;
using namespace aliceVision::sfm;

BOOST_AUTO_TEST_CASE(scaleFromKinect_computeScaleForKnownDataset)
{
	// INPUT DATA - SFM
	std::string filename = "d:/Help/Reconstructions/Kinect/rgbd2/AliceVision/MeshroomCache/StructureFromMotion/2e60d29117d352a99ad747548da731cdd99b03fc/sfm.abc";
	sfmData::SfMData sfmData;
	ESfMData flags_part = ALL;
	BOOST_CHECK(Load(sfmData, filename, flags_part));

	// INPUT DATA - IMGS PATH
	std::string imgs_path("d:/Help/Reconstructions/Kinect/rgbd2/bigdepth");

	// EXPECTED RESULT
	double scale = 5882.8035847493;

	// PROCESS
	double out_scale = 0;
	BOOST_CHECK(computeSfMScaleFromKinect(sfmData, imgs_path, &out_scale));

	// TEST
	BOOST_CHECK(std::abs(scale - out_scale) < (scale * 0.02));		// the error is smaller than 2%
}
