// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE PANORAMA_SFM_RADIAL3_OUTLIERS

#include "panoramaSfM_test_common.hpp"
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

BOOST_AUTO_TEST_CASE(PANORAMA_SFM_RADIAL3_OUTLIERS)
{
    auto intrinsic_gt = camera::createIntrinsic(camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 1357.0, 1357.0, 0, 0);
    auto intrinsic_est = camera::createIntrinsic(camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 1000.0, 1000.0, 40, -20);

    test_panorama(intrinsic_gt, intrinsic_est, 0.3);
}
