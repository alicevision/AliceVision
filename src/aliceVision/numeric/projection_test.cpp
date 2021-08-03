// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/projection.hpp>

#define BOOST_TEST_MODULE projection

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(Numeric_Cheirality)
{
    const Eigen::Index numPts{10};
    Mat3X pt3d(3, numPts);
    pt3d <<   0.680375,    0.59688,  -0.329554,    0.10794,  -0.270431,    0.83239,  -0.716795,  -0.514226,  -0.686642,  -0.782382,
        -0.211234,   0.823295,   0.536459, -0.0452059,  0.0268018,   0.271423,   0.213938,  -0.725537,  -0.198111,   0.997849,
        0.566198,  -0.604897,  -0.444451,   0.257742,   0.904459,   0.434594,  -0.967399,   0.608354,  -0.740419,  -0.563486;

    const Mat3 R = RotationAroundZ(0.3) * RotationAroundX(0.1) * RotationAroundY(0.2);

    const Vec3 t = {0.0258648, 0.678224,  0.22528};

    Vecb expected = Vecb(numPts);
    expected << true, false, false, true, true, true, false, true, false, false;

    const Vecb test = cheiralityTest(R, t, pt3d);
    for(Eigen::Index i = 0; i < numPts; ++i)
    {
        BOOST_CHECK_EQUAL(expected(i), test(i));
    }

    BOOST_TEST(!cheiralityTestAll(R, t, pt3d));
}