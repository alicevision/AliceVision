// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="pyalicevision") geometry

%include <aliceVision/numeric/eigen.i>
%eigen_typemaps(Vec2)
%eigen_typemaps(Vec3)
%eigen_typemaps(Eigen::Vector3d)
%eigen_typemaps(Eigen::Matrix<double, 2, 2, Eigen::RowMajor>)
%eigen_typemaps(Eigen::Matrix<double, 3, 3, Eigen::RowMajor>)
%eigen_typemaps(Eigen::Matrix3d)
%eigen_typemaps(Mat3)
%eigen_typemaps(Mat4)
%eigen_typemaps(Mat3X)
%eigen_typemaps(SE3::Matrix)

%include <aliceVision/geometry/Frustum.i>
%include <aliceVision/geometry/HalfPlane.i>
%include <aliceVision/geometry/Intersection.i>
%include <aliceVision/geometry/Pose3.i>
%include <aliceVision/geometry/RigidTransformation3D.i>
%include <aliceVision/geometry/Similarity3.i>
%include <aliceVision/geometry/lie.i>