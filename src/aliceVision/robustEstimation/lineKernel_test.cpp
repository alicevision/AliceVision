// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "aliceVision/robustEstimation/LineKernel.hpp"

#include <vector>

#define BOOST_TEST_MODULE LineKernel
#include <boost/test/included/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;
using namespace aliceVision::robustEstimation;

// Since the line fitter isn't so simple, test it in isolation.
BOOST_AUTO_TEST_CASE(LineFitter_ItWorks) {

  Mat2X xy(2, 5);
  // y = 2x + 1
  xy << 1, 2, 3, 4,  5,
        3, 5, 7, 9, 11;
  std::vector<Vec2> models;
  LineKernel kernel(xy);
  std::vector<size_t> samples;
  for (size_t i = 0; i < xy.cols(); ++i) {
    samples.push_back(i);
  }
  kernel.Fit(samples, &models);
  BOOST_CHECK_EQUAL(1, models.size());
  BOOST_CHECK_SMALL(2.0-models[0][1], 1e-9);
  BOOST_CHECK_SMALL(1.0-models[0][0], 1e-9);
}
