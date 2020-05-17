// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>
#include <aliceVision/multiview/triangulation/triangulationDLT.hpp>

#define BOOST_TEST_MODULE triangulationDLT

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(Triangulation_TriangulateDLT)
{
  NViewDataSet d = NRealisticCamerasRing(2, 12);

  for (int i = 0; i < d._X.cols(); ++i)
  {
    Vec2 x1, x2;
    x1 = d._x[0].col(i);
    x2 = d._x[1].col(i);
    Vec3 X_estimated, X_gt;
    X_gt = d._X.col(i);
    multiview::TriangulateDLT(d.P(0), x1, d.P(1), x2, &X_estimated);
    BOOST_CHECK_SMALL(DistanceLInfinity(X_estimated, X_gt), 1e-8);
  }
}
