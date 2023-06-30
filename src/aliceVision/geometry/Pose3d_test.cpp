// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#define BOOST_TEST_MODULE Pose3d

#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/unitTest.hpp>
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

using namespace aliceVision;

BOOST_AUTO_TEST_CASE(DefaultConstructorTest)
{
    geometry::Pose3 pose;
    const double precision{1e-5};
    EXPECT_MATRIX_NEAR(pose.rotation(), Mat3::Identity(), precision);
    EXPECT_MATRIX_NEAR(pose.center(), Vec3::Zero(), precision);
    EXPECT_MATRIX_NEAR(pose.translation(), Vec3::Zero(), precision);
}

BOOST_AUTO_TEST_CASE(ConstructorTest)
{
    const Mat3 rotation = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    //  rotation << 1, 0, 0,
    //              0, 1, 0,
    //              0, 0, 1;
    const Vec3 center(-1.34, 2.432, 3.396);
    const double precision{1e-5};

    geometry::Pose3 pose(rotation, center);

    EXPECT_MATRIX_NEAR(pose.rotation(), rotation, precision);
    EXPECT_MATRIX_NEAR(pose.center(), center, precision);
    EXPECT_MATRIX_NEAR(pose.translation(), Vec3(-rotation*center), precision);
}

BOOST_AUTO_TEST_CASE(RotationAccessorsTest)
{
    const Mat3 rotation = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    const double precision{1e-5};
    geometry::Pose3 pose;
    pose.setRotation(rotation);

    EXPECT_MATRIX_NEAR(pose.rotation(), rotation, precision);
}

BOOST_AUTO_TEST_CASE(CenterAccessorsTest)
{
    const double precision{1e-5};
    const Vec3 center(-1.34, 2.432, 3.396);

    geometry::Pose3 pose;
    pose.setCenter(center);

    EXPECT_MATRIX_NEAR(pose.center(), center, precision);
}

BOOST_AUTO_TEST_CASE(TranslationTest)
{
    const double precision{1e-5};
    const Mat3 rotation = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    const Vec3 center(-1.34, 2.432, 3.396);
    const Vec3 translation = -rotation * center;

    geometry::Pose3 pose(rotation, center);

    EXPECT_MATRIX_NEAR(pose.translation(), translation, precision);
}

BOOST_AUTO_TEST_CASE(ApplyPoseTest)
{
    const double precision{1e-5};
    const Mat3 rotation = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    const Vec3 center(-1.34, 2.432, 3.396);
    const Vec3 translation = -rotation * center;

    const Mat4 transform = Eigen::Affine3d(Eigen::Translation3d(translation) * rotation).matrix();

    const geometry::Pose3 pose(rotation, center);

    const Mat3X points = Mat3X::Random(3, 10);

    const Mat3X result = pose(points);

    const Mat3X expected = (transform * points.colwise().homogeneous()).colwise().hnormalized();

    EXPECT_MATRIX_NEAR(result, expected, precision);
}

BOOST_AUTO_TEST_CASE(CompositionTest)
{
    const double precision{1e-5};
    const Mat3 rotation1 = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    const Vec3 center1(-1.34, 2.432, 3.396);
    const Vec3 translation1 = -rotation1 * center1;
    const Mat4 homogeneous1 = Eigen::Affine3d(Eigen::Translation3d(translation1) * rotation1).matrix();

    const Mat3 rotation2 = Eigen::AngleAxisd(-0.965, Vec3(-0.034, 1, 0.2).normalized()).toRotationMatrix();
    const Vec3 center2(10.4, -25.2, 9.5);
    const Vec3 translation2 = -rotation2 * center2;
    const Mat4 homogeneous2 = Eigen::Affine3d(Eigen::Translation3d(translation2) * rotation2).matrix();

    const geometry::Pose3 pose1(rotation1, center1);
    const geometry::Pose3 pose2(rotation2, center2);

    const geometry::Pose3 result = pose1 * pose2;
    const Mat4 expected = homogeneous1 * homogeneous2;

    EXPECT_MATRIX_NEAR(result.getHomogeneous(), expected, precision);
}

BOOST_AUTO_TEST_CASE(EqualityTest)
{
    const Mat3 rotation = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    const Vec3 center(-1.34, 2.432, 3.396);

    const geometry::Pose3 pose(rotation, center);

    BOOST_CHECK(pose == pose);
}

BOOST_AUTO_TEST_CASE(InverseTest)
{
    const double precision{1e-5};
    const Mat3 rotation = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    const Vec3 center(-1.34, 2.432, 3.396);

    const geometry::Pose3 pose(rotation, center);
    const geometry::Pose3 inverse = pose.inverse();

    const Mat3 expectedRotation = rotation.transpose();
    const Vec3 expectedCenter = pose.translation();

    EXPECT_MATRIX_NEAR(inverse.rotation(), expectedRotation, precision);
    EXPECT_MATRIX_NEAR(inverse.center(), expectedCenter, precision);
    EXPECT_MATRIX_NEAR((inverse * pose).getHomogeneous(), Mat4::Identity(), precision);

}

BOOST_AUTO_TEST_CASE(DepthTest)
{
    const double precision{1e-5};
    const Mat3 rotation = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    const Vec3 center(-1.34, 2.432, 3.396);
    const Vec3 translation = -rotation * center;
    const Mat4 homogeneous = Eigen::Affine3d(Eigen::Translation3d(translation) * rotation).matrix();

    geometry::Pose3 pose(rotation, center);

    const Mat3X points = Mat3X::Random(3, 10);

    for(Eigen::Index i{0}; i < points.cols(); ++i)
    {
        const auto& point = points.col(i);
        const auto& transformedPoint = (homogeneous * point.homogeneous()).hnormalized();
        const double expectedDepth = transformedPoint(2);
        const double depth = pose.depth(point);

        BOOST_CHECK_CLOSE(depth, expectedDepth, precision);
    }
}

//BOOST_AUTO_TEST_CASE(TransformSRtTest)
//{
//    const double precision{1e-5};
//    Mat3 rotation;
//    rotation << 1, 0, 0, 0, 1, 0, 0, 0, 1;
//    const Vec3 center(1, 2, 3);
//
//    const geometry::Pose3 pose(rotation, center);
//
//    double scale{2.0};
//    Mat3 newRotation;
//    newRotation << 0, -1, 0, 1, 0, 0, 0, 0, 1;
//    const Vec3 translation(4, 5, 6);
//
//    const geometry::Pose3 transformedPose = pose.transformSRt(scale, newRotation, translation);
//
//    const Vec3 expectedCenter = scale * newRotation * center + translation;
//    const Mat3 expectedRotation = rotation * newRotation.transpose();
//
//    EXPECT_MATRIX_NEAR(transformedPose.center(), expectedCenter, precision);
//    EXPECT_MATRIX_NEAR(transformedPose.rotation(), expectedRotation, precision);
//}

BOOST_AUTO_TEST_CASE(GetHomogeneousTest)
{
    const double precision{1e-5};
    const Mat3 rotation = Eigen::AngleAxisd(-0.324, Vec3(-0.236, 1, 0.874).normalized()).toRotationMatrix();
    const Vec3 center(-1.34, 2.432, 3.396);
    const Vec3 translation = -rotation * center;

    const geometry::Pose3 pose(rotation, center);

    const Mat4 expectedHomogeneous = Eigen::Affine3d(Eigen::Translation3d(translation) * rotation).matrix();

    const Mat4 homogeneous = pose.getHomogeneous();

    EXPECT_MATRIX_NEAR(homogeneous, expectedHomogeneous, precision);
}

