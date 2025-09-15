// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE poseFilter
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>
#include <aliceVision/sfm/utils/poseFilter.hpp>


BOOST_AUTO_TEST_CASE(PoseFilter_diffFilter)
{
    // Check that diffFilter (dfc) applied to the temporal delta signal
    // gives same results as the original filter (fc) applied to the same signal (s)
    // i.e. sum( dfc(i) * (s(i+1)-s(i)) ) = sum( fc(i) * s(i) )

    using namespace Eigen;
    using namespace indexing;

    tempFilter tFilter;

    tFilter.init();

    MatrixXd sample(3, 100);

    for (int idx=0; idx<sample.cols(); idx++)
    {
        sample(0, idx) = 1.25 + double(idx) * (.025 - 0.125 * double(idx) ) + 100. * std::cos(double(idx));
        sample(1, idx) = 1.5 + double(idx) * (.05 + 0.125 * double(idx) ) + 100. * std::cos(1.25*double(idx));
        sample(2, idx) = 1.75 + double(idx) * (.075 + 0.25 * double(idx) ) + 100. * std::cos(.25*double(idx));
    }

    // Apply the filter to the sample signal
    MatrixXd filteredSample(tFilter.apply(sample, false));

    // Compute the temporal delta signal
    MatrixXd diffSignal(sample(all, seqN(1, sample.cols()-1)) - sample(all, seqN(0, sample.cols()-1)));

    MatrixXd diffFilteredSignal(sample.rows(), sample.cols());

    // Apply the diff filter to the temporal delta signal
    tFilter.applyCoreFilter(diffSignal, diffFilteredSignal, true);

    // Compute the filtered signal
    MatrixXd filteredSignal_fromDiff = sample + diffFilteredSignal;

    EXPECT_MATRIX_NEAR(filteredSignal_fromDiff, filteredSample, 1e-11);
}


BOOST_AUTO_TEST_CASE(PoseFilter_polynomial)
{
    // Check that the filter converges to a 2nd order polynomial

    using namespace Eigen;
    using namespace indexing;

    tempFilter tFilter;

    tFilter.init();

    MatrixXd sample(60, 20);

    // Generate 2nd-order polynomial samples
    for (int row=0; row<sample.rows(); row++)
    {
        double coeff_a = std::cos(double(row));
        double coeff_b = std::cos(.5 * double(row) + .25);
        double coeff_c = std::cos(.25 * double(row) + .5);

        for (int idx=0; idx<sample.cols(); idx++)
        {
            sample(row, idx) = coeff_a + double(idx) * (coeff_b + coeff_c * double(idx));
        }
    }

    MatrixXd distortedSample(sample);

    // Create a binary mask
    Matrix<bool, Dynamic, Dynamic> distortionMask(sample.rows(), sample.cols());
    distortionMask.setZero();

    for (int row=0; row<sample.rows(); row++)
    {
        //index of the value to distort
        int idx = row%sample.cols();

        // Create a binary mask to be able to easily restore the original values
        distortionMask(row, idx) = true;

        // Distort the signal (a single value per row)
        distortedSample(row, idx) = 100. * std::cos(double(4+row));
    }

    for (int iter=0; iter<1000; iter++)
    {
        // Apply the filter to the distorted signal
        distortedSample = tFilter.apply(distortedSample, false);

        // Restore the original values except for the distorted values
        distortedSample = distortionMask.select(distortedSample, sample);
    }

    // Run it once without restoring any value
    distortedSample = tFilter.apply(distortedSample, false);

    EXPECT_MATRIX_NEAR(distortedSample, sample, 1e-11);
}


BOOST_AUTO_TEST_CASE(PoseFilter_angles)
{
    // Check that the filter correctly filters orientations

    using namespace Eigen;
    using namespace indexing;
    using namespace aliceVision;

    tempFilter tFilter;

    tFilter.init();

    int posesNb = 60;

    for (int mainIter=0; mainIter < 1000; mainIter++)
    {
        Vector3d arbitraryAxis = Vector3d::Random();
        arbitraryAxis = arbitraryAxis / arbitraryAxis.norm();

        Vector3d arbitraryAxis2 = Vector3d::Random();
        arbitraryAxis2 = arbitraryAxis2 / arbitraryAxis2.norm();

        MatrixXd rotationSample(4, posesNb);
        MatrixXd saferRotationSample(4, posesNb);

        // Generate poses on a circle
        for (int idPV = 0; idPV < posesNb; idPV++)
        {
            double angle2d = (double(idPV) * 2. * M_PI) / posesNb;
            AngleAxisd aa = (angle2d > M_PI) ? AngleAxisd(2. * M_PI - angle2d, -arbitraryAxis) : AngleAxisd(angle2d, arbitraryAxis);
            rotationSample.col(idPV) << aa.angle(), aa.axis();

            angle2d = (double(idPV) * 2. * M_PI + .0001) / posesNb;
            aa = (angle2d > M_PI) ? AngleAxisd(2. * M_PI - angle2d, -arbitraryAxis) : AngleAxisd(angle2d, arbitraryAxis);
            saferRotationSample.col(idPV) << aa.angle(), aa.axis();
        }

        MatrixXd filteredSample;

        filteredSample = tFilter.apply(saferRotationSample, true);

        for (int idPV = 0; idPV < posesNb; idPV++)
        {
            // Check in so(3) space
            EXPECT_MATRIX_NEAR((filteredSample(0, idPV) * filteredSample(seqN(1,3), idPV)),
                               (saferRotationSample(0, idPV) * saferRotationSample(seqN(1,3), idPV)), 2e-15);

            // Check in SO(3) space (rotation matrices)
            AngleAxisd aa(saferRotationSample(0, idPV), saferRotationSample(seqN(1,3), idPV));
            AngleAxisd filteredAA(filteredSample(0, idPV), filteredSample(seqN(1,3), idPV));
            EXPECT_MATRIX_NEAR(aa.toRotationMatrix(), filteredAA.toRotationMatrix(), 2e-15);
        }

        filteredSample = tFilter.apply(rotationSample, true);
        // In rotationSample, there is a rotation with a rotation equal to pi
        // As a rotation of pi around a given axis is equivalent to a rotation of pi around the inverted axis (-axis)
        // there are two equivalent so(3) solutions for this rotation, thus we only use rotation matrices to check this set
        for (int idPV = 0; idPV < posesNb; idPV++)
        {
            AngleAxisd aa(rotationSample(0, idPV), rotationSample(seqN(1,3), idPV));
            AngleAxisd filteredAA(filteredSample(0, idPV), filteredSample(seqN(1,3), idPV));
            EXPECT_MATRIX_NEAR(aa.toRotationMatrix(), filteredAA.toRotationMatrix(), 2e-15);
        }
    }
}
