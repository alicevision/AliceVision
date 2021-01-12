// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/camera/Distortion.hpp>
#include <aliceVision/camera/DistortionBrown.hpp>
#include <aliceVision/camera/DistortionFisheye.hpp>
#include <aliceVision/camera/DistortionFisheye1.hpp>
#include <aliceVision/camera/DistortionRadial.hpp>

#define BOOST_TEST_MODULE distortion
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;
using namespace aliceVision::camera;

//-----------------
BOOST_AUTO_TEST_CASE(distortion_distort_undistort)
{
    std::array<std::unique_ptr<Distortion>, 6> distortionsModels;
    distortionsModels[0].reset(new DistortionBrown(-0.25349, 0.11868, -0.00028, 0.00005, 0.0000001));
    distortionsModels[1].reset(new DistortionFisheye(0.02, -0.03, 0.1, -0.2));
    distortionsModels[2].reset(new DistortionFisheye1(0.02));
    distortionsModels[3].reset(new DistortionRadialK1(0.02));
    distortionsModels[4].reset(new DistortionRadialK3(-1.8061369278146561e-01, 1.8759742680633607e-01, -2.5341468279930644e-02));
    distortionsModels[5].reset(new DistortionRadialK3PT(-1.8061369278146561e-01, 1.8759742680633607e-01, -2.5341468279930644e-02));

    const double epsilon = 1e-4;
    const std::size_t numPts{1000};
    for(std::size_t i = 0; i < numPts; ++i)
    {
        // random point in [-lim, lim]x[-lim, lim]
        const double lim{0.8};
        const Vec2 ptImage = lim*Vec2::Random();

        for(const auto& model : distortionsModels)
        {
            const auto distorted = model->addDistortion(ptImage);
            const auto undistorted = model->removeDistortion(distorted);

            // distortion actually happened
            BOOST_CHECK(!(distorted == ptImage));

            EXPECT_MATRIX_CLOSE_FRACTION(ptImage, undistorted, epsilon);
        }
    }
}
