// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE poseFilter
#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>
#include <aliceVision/unitTest.hpp>
#include <aliceVision/sfmDataIO/sceneSample.hpp>
#include <aliceVision/sfmData/ImageSequence.hpp>
#include <aliceVision/sfmData/ImageSet.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
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


BOOST_AUTO_TEST_CASE(PoseFilter_interpolateMissingPoses)
{
    // Check that PoseFilter correctly interpolates missing poses

    using namespace aliceVision;

    makeRandomOperationsReproducible();

    sfmData::SfMData sfmData;
    sfmDataIO::generateSphereScene(sfmData, 100, 240);

    // Generate new ImageSet
    size_t imageGroupID2 = 3454613548;
    for (IndexT idPV = 300; idPV <410; idPV++)
    {
        IndexT grpID1_viewID = idPV - 300;
        sfmData.getPoses().assign(idPV, sfmData.getPose(sfmData.getView(grpID1_viewID)));
        sfmData.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, 100, 100));
        sfmData::View& view = sfmData.getView(idPV);
        view.setFrameId(idPV);
        view.setImageGroupId(imageGroupID2);
    }
    auto imageGroupPtr2 = std::make_shared<sfmData::ImageSet>();
    sfmData.getImageGroups().emplace(imageGroupID2, imageGroupPtr2);

    // Generate new ImageSequence
    size_t imageGroupID3 = 845461354;
    for (IndexT idPV = 500; idPV <640; idPV++)
    {
        IndexT grpID1_viewID = idPV - 500;
        sfmData.getPoses().assign(idPV, sfmData.getPose(sfmData.getView(grpID1_viewID)));
        sfmData.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, 100, 100));
        sfmData.getView(idPV).setFrameId(idPV);
        sfmData.getView(idPV).setImageGroupId(imageGroupID3);
    }
    auto imageGroupPtr3 = std::make_shared<sfmData::ImageSequence>();
    sfmData.getImageGroups().emplace(imageGroupID3, imageGroupPtr3);

    IndexT poses2remove[] = {0, 1, 2, 3, 24, 45, 46, 47, 48, 239, 350,
                            500, 501, 502, 503, 504, 518, 519, 520, 521, 522, 523, 524,
                            560, 561, 562, 563, 633, 634, 635, 636, 637, 638, 639};
    for (IndexT i=0; i<sizeof(poses2remove)/sizeof(IndexT); i++)
    {
        sfmData.erasePose(poses2remove[i]);
    }

    for (IndexT i=0; i<sizeof(poses2remove)/sizeof(IndexT); i++)
    {
        BOOST_CHECK(!sfmData.isPoseDefined(poses2remove[i]));
    }

    sfm::poseFilter poseFilter;
    BOOST_CHECK(poseFilter.interpolateMissingPoses(sfmData));
    for (IndexT i=0; i<sizeof(poses2remove)/sizeof(IndexT); i++)
    {
        if (poses2remove[i] < 240 || poses2remove[i] >= 500)
        {
            BOOST_CHECK(sfmData.isPoseDefined(poses2remove[i]));
        }
        else
        {
            BOOST_CHECK(!sfmData.isPoseDefined(poses2remove[i]));
        }
    }
    BOOST_CHECK(poseFilter.interpolateMissingPoses(sfmData));
    for (IndexT i=0; i<sizeof(poses2remove)/sizeof(IndexT); i++)
    {
        if (poses2remove[i] < 240 || poses2remove[i] >= 500)
        {
            BOOST_CHECK(sfmData.isPoseDefined(poses2remove[i]));
        }
        else
        {
            BOOST_CHECK(!sfmData.isPoseDefined(poses2remove[i]));
        }
    }

    // Generate new ImageSequence with a missing frame in the sequence
    size_t imageGroupID4 = 875461355;
    for (IndexT idPV = 800; idPV < 900; idPV++)
    {
        if (idPV == 850)
        {
            // Skip this view to create a missing frame in the sequence
            continue;
        }
        IndexT grpID1_viewID = idPV - 800;
        sfmData.getPoses().assign(idPV, sfmData.getPose(sfmData.getView(grpID1_viewID)));
        sfmData.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, 100, 100));
        sfmData.getView(idPV).setFrameId(idPV);
        sfmData.getView(idPV).setImageGroupId(imageGroupID4);
    }
    auto imageGroupPtr4 = std::make_shared<sfmData::ImageSequence>();
    sfmData.getImageGroups().emplace(imageGroupID4, imageGroupPtr4);

    // InterpolateMissingPoses should fail because there is a missing frame in the sequence
    BOOST_CHECK(!poseFilter.interpolateMissingPoses(sfmData));

    IndexT idPV = 850;
    IndexT grpID1_viewID = idPV - 800;
    sfmData.getPoses().assign(idPV, sfmData.getPose(sfmData.getView(grpID1_viewID)));
    sfmData.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, 100, 100));
    sfmData.getView(idPV).setFrameId(idPV);
    sfmData.getView(idPV).setImageGroupId(imageGroupID4);

    // InterpolateMissingPoses should now succeed because the missing frame has been added back
    BOOST_CHECK(poseFilter.interpolateMissingPoses(sfmData));

    idPV = 900;
    grpID1_viewID = idPV - 800;
    sfmData.getPoses().assign(idPV, sfmData.getPose(sfmData.getView(grpID1_viewID)));
    sfmData.getViews().emplace(idPV, std::make_shared<sfmData::View>("", idPV, 0, idPV, 100, 100));
    sfmData.getView(idPV).setFrameId(850);
    sfmData.getView(idPV).setImageGroupId(imageGroupID4);

    // InterpolateMissingPoses should now fail because there is a duplicate frame in the sequence
    BOOST_CHECK(!poseFilter.interpolateMissingPoses(sfmData));
}
