// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE bundleAdjustment_enhanced

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentCeres.hpp>
#include <aliceVision/sfm/bundle/BundleAdjustmentSymbolicCeres.hpp>
#include <aliceVision/geometry/lie.hpp>
#include <aliceVision/geometry/Intersection.hpp>
#include <aliceVision/sfm/utils/statistics.hpp>

#include <boost/test/unit_test.hpp>
#include <boost/test/tools/floating_point_comparison.hpp>

using namespace aliceVision;

bool getPointOnSphere(Vec3& result, const camera::IntrinsicBase& intrinsic, const Vec2& pt, double distanceToUnitCenter)
{
    Vec3 campt = intrinsic.toUnitSphere(intrinsic.removeDistortion(intrinsic.ima2cam(pt)));
    Vec3 camorigin(0.0, 0.0, -distanceToUnitCenter);
    Vec3 worldcampt = campt + camorigin;

    return geometry::rayIntersectUnitSphere(result, camorigin, campt);
}

void createScene(sfmData::SfMData& sfmData, const camera::IntrinsicBase& intrinsic)
{
    // Create a scene where the points lie on a sphere of unit 1
    const int countNeededPointsPerImageLine = 30;
    const int countNeededPointsPerImageColumn = 15;
    double distanceToUnitCenter = 1.3;

    // Simulate a camera at a given distance of the sphere
    // The projected points will be used to determine the properties of the cameras
    Vec3 borderLeft;
    if (!getPointOnSphere(borderLeft, intrinsic, Vec2(0, intrinsic.h() / 2.0), distanceToUnitCenter))
    {
        borderLeft = Vec3(-1, 0, 0);
    }

    Vec3 borderRight;
    if (!getPointOnSphere(borderRight, intrinsic, Vec2(intrinsic.w(), intrinsic.h() / 2.0), distanceToUnitCenter))
    {
        borderRight = Vec3(1, 0, 0);
    }

    Vec3 borderTop;
    if (!getPointOnSphere(borderTop, intrinsic, Vec2(intrinsic.w() / 2.0, 0.0), distanceToUnitCenter))
    {
        borderTop = Vec3(0, -1, 0);
    }

    Vec3 borderBottom;
    if (!getPointOnSphere(borderBottom, intrinsic, Vec2(intrinsic.w() / 2.0, intrinsic.h()), distanceToUnitCenter))
    {
        borderBottom = Vec3(0, 1, 0);
    }

    // The camera observe a part of the sphere, compute the arc length.
    double horizontalAnglePerCamera = std::acos(borderLeft.normalized().dot(borderRight.normalized()));
    double verticalAnglePerCamera = std::acos(borderBottom.normalized().dot(borderTop.normalized()));

    // Estimated the required number of cameras and the scene properties
    double overlap = 0.5;
    int countCameras = std::ceil(2.0 * M_PI / (horizontalAnglePerCamera * (1.0 - overlap)));
    const int countNeededPointsPerLine = std::ceil(countNeededPointsPerImageLine * 2.0 * M_PI / horizontalAnglePerCamera);
    const double angleBetweenLines = verticalAnglePerCamera / countNeededPointsPerImageColumn;

    // Place landmarks on the unit sphere
    size_t count = 0;
    for (int y = -countNeededPointsPerImageColumn / 2; y <= countNeededPointsPerImageColumn / 2; y++)
    {
        for (int x = 0; x < countNeededPointsPerLine; x++)
        {
            double longitude = x * 2.0 * M_PI / countNeededPointsPerLine;
            double latitude = y * angleBetweenLines;

            Vec3 pos;
            pos.x() = cos(latitude) * sin(longitude);
            pos.y() = sin(latitude);
            pos.z() = cos(latitude) * cos(longitude);

            sfmData.getLandmarks().emplace(count, pos);
            count++;
        }
    }

    // Place cameras and observations
    for (int idview = 0; idview < countCameras; idview++)
    {
        double angle = idview * (2.0 * M_PI / countCameras);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = SO3::expm(Eigen::Vector3d::UnitY() * angle);
        T.block<3, 1>(0, 3) = Eigen::Vector3d::UnitZ() * distanceToUnitCenter;

        geometry::Pose3 pose(T);

        sfmData.getPoses().emplace(idview, sfmData::CameraPose(pose));
        sfmData.getViews().emplace(idview, std::make_shared<sfmData::View>("", idview, 0, idview, intrinsic.w(), intrinsic.h()));

        Vec3 origin = pose(Vec3(0, 0, 0));

        int count = 0;
        for (auto& pl : sfmData.getLandmarks())
        {
            const auto& pt = pl.second.X;

            Vec3 cpt = pose(pt);
            Vec3 dir = (cpt - origin).normalized();

            // Point normal is backward
            if (dir.z() > 0.0)
            {
                continue;
            }

            Vec2 imagept = intrinsic.project(pose, pt.homogeneous(), true);
            if (imagept.x() < 0 || imagept.y() < 0 || imagept.x() >= intrinsic.w() || imagept.y() >= intrinsic.h())
            {
                continue;
            }

            sfmData::Observation obs(imagept, count, 1.0);

            pl.second.getObservations().emplace(idview, obs);
            count++;
        }
    }

    // Create intrinsic storage
    sfmData.getIntrinsics().emplace(0, intrinsic.clone());
}

using cameraPair = std::pair<std::shared_ptr<camera::IntrinsicBase>, std::shared_ptr<camera::IntrinsicBase>>;

std::vector<cameraPair> buildIntrinsics()
{
    std::vector<cameraPair> cameras;

    cameras.push_back(std::make_pair(camera::createPinhole(camera::PINHOLE_CAMERA, 1920, 1080, 900, 900, 80, 50),
                                     camera::createPinhole(camera::PINHOLE_CAMERA, 1920, 1080, 1200, 1200, 0, 0)));

    cameras.push_back(std::make_pair(camera::createPinhole(camera::PINHOLE_CAMERA_RADIAL1, 1920, 1080, 900, 900, 80, 50, {0.5}),
                                     camera::createPinhole(camera::PINHOLE_CAMERA_RADIAL1, 1920, 1080, 1200, 1200, 0, 0, {0.0})));

    cameras.push_back(std::make_pair(camera::createPinhole(camera::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 900, 900, 80, 50, {0.5, -0.4, 1.2}),
                                     camera::createPinhole(camera::PINHOLE_CAMERA_RADIAL3, 1920, 1080, 1200, 1200, 0, 0, {0.0, 0.0, 0.0})));

    cameras.push_back(
      std::make_pair(camera::createPinhole(camera::PINHOLE_CAMERA_BROWN, 1920, 1080, 900, 900, 80, 50, {-0.054, 0.014, 0.006, 0.001, -0.001}),
                     camera::createPinhole(camera::PINHOLE_CAMERA_BROWN, 1920, 1080, 1200, 1200, 0, 0, {0, 0, 0, 0, 0})));

    cameras.push_back(std::make_pair(camera::createPinhole(camera::PINHOLE_CAMERA_FISHEYE, 1920, 1080, 900, 900, 80, 50, {0.5, -0.4, 0.1, 0.2}),
                                     camera::createPinhole(camera::PINHOLE_CAMERA_FISHEYE, 1920, 1080, 1200, 1200, 0, 0, {0.0, 0.0, 0.0, 0.0})));

    cameras.push_back(std::make_pair(camera::createPinhole(camera::PINHOLE_CAMERA_FISHEYE1, 1920, 1080, 900, 900, 80, 50, {0.5}),
                                     camera::createPinhole(camera::PINHOLE_CAMERA_FISHEYE1, 1920, 1080, 1200, 1200, 0, 0, {1.2})));

    /*cameras.push_back(std::make_pair(
        camera::createPinhole(camera::PINHOLE_CAMERA_3DEANAMORPHIC4, 1920, 1080, 900, 900, 80, 50, {0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, 1.0, 1.0}), camera::createPinhole(camera::PINHOLE_CAMERA_3DEANAMORPHIC4, 1920, 1080, 1200, 1200, 0, 0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0})
    ));*/

    cameras.push_back(
      std::make_pair(camera::createPinhole(camera::PINHOLE_CAMERA_3DERADIAL4, 1920, 1080, 900, 900, 80, 50, {0.2, 0.0, 0.0, 0.0, 0.0, 0.0}),
                     camera::createPinhole(camera::PINHOLE_CAMERA_3DERADIAL4, 1920, 1080, 1200, 1200, 0, 0, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0})));

    cameras.push_back(
      std::make_pair(camera::createPinhole(camera::PINHOLE_CAMERA_3DECLASSICLD, 1920, 1080, 900, 900, 80, 50, {0.2, 1.0, 0.0, 0.0, 0.0}),
                     camera::createPinhole(camera::PINHOLE_CAMERA_3DECLASSICLD, 1920, 1080, 1200, 1200, 0, 0, {0.0, 1.0, 0.0, 0.0, 0.0})));

    cameras.push_back(std::make_pair(camera::createEquidistant(camera::EQUIDISTANT_CAMERA, 1920, 1080, 1500, 80, 50),
                                     camera::createEquidistant(camera::EQUIDISTANT_CAMERA, 1920, 1080, 1300, 0, 0)));

    cameras.push_back(std::make_pair(camera::createEquidistant(camera::EQUIDISTANT_CAMERA_RADIAL3, 1920, 1080, 1500, 0, 0, {0.11, -0.30, 0.1}),
                                     camera::createEquidistant(camera::EQUIDISTANT_CAMERA_RADIAL3, 1920, 1080, 900, 10, 20, {0.0, 0.0, 0.0})));

    return cameras;
}

BOOST_AUTO_TEST_CASE(test_intrinsics)
{
    auto listIntrinsics = buildIntrinsics();

    for (auto pairIntrinsics : listIntrinsics)
    {
        sfmData::SfMData sfmData;

        createScene(sfmData, *pairIntrinsics.first);

        sfmData.getIntrinsics().at(0) = pairIntrinsics.second;

        sfm::BundleAdjustmentSymbolicCeres::CeresOptions options;
        sfm::BundleAdjustment::ERefineOptions refineOptions = sfm::BundleAdjustment::REFINE_INTRINSICS_FOCAL |
                                                              sfm::BundleAdjustment::REFINE_INTRINSICS_OPTICALOFFSET_ALWAYS |
                                                              sfm::BundleAdjustment::REFINE_INTRINSICS_DISTORTION;
        options.summary = false;

        double rmseBefore = sfm::RMSE(sfmData);
        sfm::BundleAdjustmentSymbolicCeres BA(options);
        const bool success = BA.adjust(sfmData, refineOptions);
        double rmseAfter = sfm::RMSE(sfmData);

        BOOST_TEST_CONTEXT(EINTRINSIC_enumToString(pairIntrinsics.first->getType()))
        {
            BOOST_CHECK_LT(rmseAfter, rmseBefore);
            BOOST_CHECK_LT(rmseAfter, 1e-3);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_landmarks)
{
    auto listIntrinsics = buildIntrinsics();

    for (auto pairIntrinsics : listIntrinsics)
    {
        sfmData::SfMData sfmData;

        createScene(sfmData, *pairIntrinsics.first);

        srand(0);
        for (auto& lpt : sfmData.getLandmarks())
        {
            lpt.second.X += Eigen::Vector3d::Random() * 0.1;
        }

        sfm::BundleAdjustmentSymbolicCeres::CeresOptions options;
        sfm::BundleAdjustment::ERefineOptions refineOptions = sfm::BundleAdjustment::REFINE_STRUCTURE;
        options.summary = false;

        double rmseBefore = sfm::RMSE(sfmData);
        sfm::BundleAdjustmentSymbolicCeres BA(options);
        const bool success = BA.adjust(sfmData, refineOptions);
        double rmseAfter = sfm::RMSE(sfmData);

        BOOST_TEST_CONTEXT(EINTRINSIC_enumToString(pairIntrinsics.first->getType()))
        {
            BOOST_CHECK_LT(rmseAfter, rmseBefore);
            BOOST_CHECK_LT(rmseAfter, 1e-3);
        }
    }
}

BOOST_AUTO_TEST_CASE(test_poses)
{
    auto listIntrinsics = buildIntrinsics();

    for (auto pairIntrinsics : listIntrinsics)
    {
        sfmData::SfMData sfmData;

        createScene(sfmData, *pairIntrinsics.first);

        srand(0);
        for (auto& lps : sfmData.getPoses())
        {
            geometry::Pose3 pose3 = lps.second.getTransform();
            Eigen::Matrix4d T = pose3.getHomogeneous();

            Eigen::Vector3d nt = Eigen::Vector3d::Random() * 0.1 + Eigen::Vector3d::UnitZ() * 0.1;
            Eigen::Matrix3d nR = SO3::expm(Eigen::Vector3d::Random() * 0.01);

            Eigen::Matrix4d U = Eigen::Matrix4d::Identity();
            U.block<3, 3>(0, 0) = nR;
            U.block<3, 1>(0, 3) = nt;

            lps.second.setTransform(geometry::Pose3(U * T));
        }

        sfm::BundleAdjustmentSymbolicCeres::CeresOptions options;
        sfm::BundleAdjustment::ERefineOptions refineOptions = sfm::BundleAdjustment::REFINE_ROTATION | sfm::BundleAdjustment::REFINE_TRANSLATION;
        options.summary = false;

        double rmseBefore = sfm::RMSE(sfmData);
        sfm::BundleAdjustmentSymbolicCeres BA(options);
        const bool success = BA.adjust(sfmData, refineOptions);
        double rmseAfter = sfm::RMSE(sfmData);

        BOOST_TEST_CONTEXT(EINTRINSIC_enumToString(pairIntrinsics.first->getType()))
        {
            BOOST_CHECK_LT(rmseAfter, rmseBefore);
            BOOST_CHECK_LT(rmseAfter, 1e-3);
        }
    }
}
