// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/multiview/NViewDataSet.hpp>
#include <aliceVision/fuseCut/DelaunayGraphCut.hpp>

#include <boost/filesystem.hpp>

#include <boost/math/constants/constants.hpp>

#include <string>

#define BOOST_TEST_MODULE fuseCut

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>


using namespace aliceVision;
using namespace aliceVision::fuseCut;
using namespace aliceVision::sfmData;

BOOST_AUTO_TEST_CASE(fuseCut_solidAngle)
{
    using namespace boost::math;

    const Point3d O = {0.0, 0.0, 0.0};
    const Point3d A = {5.0, 0.0, 0.0};
    const Point3d B = {0.0, 1.8, 0.0};
    const Point3d C = {0.0, 0.0, 2.1};

    {
        const double s = tetrahedronSolidAngle(A - O, B - O, C - O);
        ALICEVISION_LOG_TRACE("tetrahedronSolidAngle: " << s);
        BOOST_CHECK_CLOSE(s, 0.5 * constants::pi<double>(), 0.0001);
    }

    {
        const double s = tetrahedronSolidAngle(B - O, C - O, A - O);
        ALICEVISION_LOG_TRACE("tetrahedronSolidAngle: " << s);
        BOOST_CHECK_CLOSE(s, 0.5 * constants::pi<double>(), 0.0001);
    }

    {
        const double s = tetrahedronSolidAngle(B - O, A - O, C - O);
        ALICEVISION_LOG_TRACE("tetrahedronSolidAngle: " << s);
        BOOST_CHECK_CLOSE(s, 0.5 * constants::pi<double>(), 0.0001);
    }
}

SfMData generateSfm(const NViewDatasetConfigurator& config, const size_t size = 3, camera::EINTRINSIC eintrinsic = camera::EINTRINSIC::PINHOLE_CAMERA_RADIAL3);

BOOST_AUTO_TEST_CASE(fuseCut_delaunayGraphCut)
{
    system::Logger::get()->setLogLevel(system::EVerboseLevel::Trace);

    const NViewDatasetConfigurator config(1000, 1000, 500, 500, 1, 0);
    SfMData sfmData = generateSfm(config, 6);

    mvsUtils::MultiViewParams mp(sfmData, "", "", "", false);

    mp.userParams.put("LargeScale.universePercentile", 0.999);
    mp.userParams.put("delaunaycut.forceTEdgeDelta", 0.1f);
    mp.userParams.put("delaunaycut.seed", 1);

    std::array<Point3d, 8> hexah;

    Fuser fs(mp);
    const size_t minObservations = 2;
    const float minObservationsAngle = 0.01f;
    fs.divideSpaceFromSfM(sfmData, &hexah[0], minObservations, minObservationsAngle);

    StaticVector<int> cams;
    cams.resize(mp.getNbCameras());
    for (int i = 0; i < cams.size(); ++i)
        cams[i] = i;
    
    const std::string tempDirPath = boost::filesystem::temp_directory_path().generic_string();
    
    DelaunayGraphCut delaunayGC(mp);
    ALICEVISION_LOG_TRACE("Creating dense point cloud witout support pts.");

    // delaunayGC.createDensePointCloud(&hexah[0], cams, &sfmData, nullptr);
    const float minDist = (hexah[0] - hexah[1]).size() / 1000.0f;
    // add points for cam centers
    delaunayGC.addPointsFromCameraCenters(cams, minDist);
    // add points from sfm
    delaunayGC.addPointsFromSfM(&hexah[0], cams, sfmData);

    ALICEVISION_LOG_TRACE("Generated pts:");
    for (size_t i = 0; i < delaunayGC._verticesCoords.size(); i++)
        ALICEVISION_LOG_TRACE("[" << i << "]: " << delaunayGC._verticesCoords[i].x << ", " << delaunayGC._verticesCoords[i].y << ", " << delaunayGC._verticesCoords[i].z);

    delaunayGC.createGraphCut(&hexah[0], cams, tempDirPath + "/", tempDirPath + "/SpaceCamsTracks/", false, false);
    /*
    delaunayGC.computeDelaunay();
    delaunayGC.displayStatistics();
    delaunayGC.computeVerticesSegSize(true, 0.0f);
    delaunayGC.voteFullEmptyScore(cams, tempDirPath);
    delaunayGC.reconstructGC(&hexah[0]);
    */

    ALICEVISION_LOG_TRACE("CreateGraphCut Done.");
}

/**
 * @brief Generate syntesize dataset with succesion of n(size) alignaed regular thetraedron and two camera on the last thetrahedron.
 * 
 * @param size
 * @param eintrinsic
 * @return 
 */
SfMData generateSfm(const NViewDatasetConfigurator& config, const size_t size, camera::EINTRINSIC eintrinsic)
{
    assert(size > 0);

    SfMData sfm_data;

    const double heightT = std::sqrt(0.75);
    const double centerMinT = 0.5 / (2.0 * heightT);

    // Y axis UP
    std::vector<Vec3> camsPts;
    camsPts.push_back({ static_cast<double>(size) + 10.0, 0.0, 0.0 });
    camsPts.push_back({ static_cast<double>(size) - 0.5 + 10.0, 0.0, heightT - 0.0});

    const size_t ptsSize = 5 * size - 3;
    // Construct our pts (Y axis UP)
    Mat3X matPts(3, ptsSize);

    // Bottom down
    for (size_t i = 0; i < size; i++)
        matPts.col(i) << i, 0.0, 0.0;
    
    // Bottom up
    for (size_t i = 0; i < size - 1; i++)
        matPts.col(size + i) << (i + 0.5), 0.0, heightT;

    // Top
    for (size_t i = 0; i < 2 * size - 1; i++)
        matPts.col(2 * size - 1 + i) << (i / 2.0 + 0.5), heightT, (i % 2 == 0 ? centerMinT : heightT - centerMinT);

    // Center
    for (size_t i = 0; i < size - 1; i ++)
        matPts.col(4 * size - 2 + i) << (i + (1 + (i % 2)) * 0.5), centerMinT + (i % 2) * 0.1, (i % 2 == 0 ? centerMinT : heightT - centerMinT); // center

    // jitters all pts
    matPts += Mat3X::Random(3, ptsSize) * 0.05;

    // Same internals parameters for all cams
    Mat3 internalParams;
    internalParams << config._fx, 0, config._cx,
        0, config._fy, config._cy,
        0, 0, 1;

    // 1. Views
    for (int i = 0; i < camsPts.size(); ++i)
    {
        const IndexT id_view = i, id_pose = i, id_intrinsic = 0; // shared intrinsics
        sfm_data.views[i] = std::make_shared<View>("", id_view, id_intrinsic, id_pose, config._cx * 2, config._cy * 2);
    }

    // 2. Poses
    // and compute projected pts for cam
    std::vector<Mat2X> projectedPtsPerCam;
    for (size_t i = 0; i < camsPts.size(); i++)
    {
        Mat34 P;
        // All cam look to the first thetraherdron center
        const Vec3 camCenter = camsPts[i];
        const Mat3 rotationMat = LookAt(matPts.col(0) - camCenter);
        const Vec3 translation = -rotationMat * camCenter;
        P_from_KRt(internalParams, rotationMat, translation, &P);
        projectedPtsPerCam.push_back(project(P, matPts));

        geometry::Pose3 pose(rotationMat, camCenter);
        sfm_data.setPose(*sfm_data.views.at(i), CameraPose(pose));
    }

    // 3. Intrinsic data (shared, so only one camera intrinsic is defined)
    {
        const unsigned int w = config._cx * 2;
        const unsigned int h = config._cy * 2;
        sfm_data.intrinsics[0] = createIntrinsic(eintrinsic, w, h, config._fx, config._cx, config._cy);
    }

    // 4. Landmarks
    const double unknownScale = 0.0;
    for (size_t i = 0; i < ptsSize; ++i)
    {
        Landmark landmark;
        landmark.X = matPts.col(i);
        for (int j = 0; j < camsPts.size(); ++j)
        {
            const Vec2 pt = projectedPtsPerCam[j].col(i);
            landmark.observations[j] = Observation(pt, i, unknownScale);
        }
        sfm_data.structure[i] = landmark;
    }

    return sfm_data;
}
