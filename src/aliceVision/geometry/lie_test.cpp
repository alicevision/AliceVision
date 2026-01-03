// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE lie

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>


BOOST_AUTO_TEST_CASE(PoseFilter_lie_conversions)
{
    // Check lie algebra

    using namespace Eigen;
    using namespace indexing;
    using namespace aliceVision::SO3;

    // Rotation axes generated from points on a regular grid
    std::vector<Vector3d> rotationAxes;
    rotationAxes.reserve(728);
    for (int x=-4; x<=4; x++)
    for (int y=-4; y<=4; y++)
    for (int z=-4; z<=4; z++)
    {
        if (x==0 && y==0 && z==0)
        {
            continue;
        }
        rotationAxes.push_back(Vector3d(x, y, z).normalized());
    }

    // Test modes:
    // 0: rotation angles in ]0, 0.2*Pi[ and rotation axes generated from points on a regular grid
    // 1: rotation angles in ]0, 0.2*Pi[ and random rotation axes
    // 2: rotation angles in ]0.1*Pi, Pi[ and rotation axes generated from points on a regular grid
    // 3: rotation angles in ]0.1*Pi, Pi[ and random rotation axes
    for (int testMode=0; testMode<4; testMode++)
    {
      // only used for statistics
      std::vector<long> histo(64, 0);
      std::vector<double> errorSo3(64, 0);
      std::vector<double> errorMatrix(64, 0);

      for (const Vector3d& rotAxis : rotationAxes)
      {
        // The rotation angles are randomly sampled, so that their distribution is roughly logarithmically uniform around 0 and around pi
        for (double scaleIter=1.; scaleIter > 1e-13; scaleIter *= .125)
        {
            double maxErrorSo3 = 0.;
            double maxErrorMatrix = 0.;

            for (int mainIter=0; mainIter < 100; mainIter++)
            {
                bool checkMinNumLimit = scaleIter==1. && mainIter == 0;

                double rnd = checkMinNumLimit ? std::numeric_limits<double>::min() : abs(VectorXd::Random(1)(0));

                // logIndex is only used for statistics
                int logIndex = checkMinNumLimit ? 63 : round(-log(abs(rnd * scaleIter)));
                histo.at(logIndex)++;

                Vector3d rotationAxis = (testMode&1) ? Vector3d::Random().normalized() : rotAxis;

                double angle2d = (testMode < 2) ? rnd * .2 * M_PI * scaleIter : M_PI - rnd * .9 * M_PI * scaleIter; // ]0, 0.2*Pi[ and ]0.1*Pi, Pi[

                AngleAxis<double> aa(angle2d, rotationAxis);
                Matrix3d rotation = aa.toRotationMatrix();

                Vector3d so3 = logm(rotation);

                double error = Array3d(so3 - (angle2d * rotationAxis)).abs().maxCoeff();
                if ( error > errorSo3.at(logIndex) )
                {
                    errorSo3.at(logIndex) = error;
                }
                if ( error > maxErrorSo3 )
                {
                    maxErrorSo3 = error;
                }

                aa.fromRotationMatrix(rotation);
                Matrix3d mat3 = expm(aa.angle()*aa.axis());

                error = Array33d(mat3 - rotation).abs().maxCoeff();
                if ( error > errorMatrix.at(logIndex) )
                {
                    errorMatrix.at(logIndex) = error;
                }
                if ( error > maxErrorMatrix )
                {
                    maxErrorMatrix = error;
                }
            }
            // With the current lie algebra implementation, errors are bigger around pi
            BOOST_CHECK_SMALL(maxErrorSo3, testMode < 2 ? 1e-15 : 1e-7);
            BOOST_CHECK_SMALL(maxErrorMatrix, testMode < 2 ? 1e-15 : 3e-15);
        }
      }

      for (int histIdx=0; histIdx<64; histIdx++)
      {
          if (histo[histIdx] == 0)
          {
              continue;
          }
          ALICEVISION_LOG_DEBUG(" logarithmic bin: " << histIdx <<
                                " size: " << histo[histIdx] <<
                                " error_so3: " << errorSo3[histIdx] <<
                                " error_matrix: " << errorMatrix[histIdx]);
      }
    }
}
