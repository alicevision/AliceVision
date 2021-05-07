// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once
#include <string>
#include <vector>

void normal2PQ(aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, Eigen::MatrixXd& p, Eigen::MatrixXd& q, bool perspective, Eigen::Matrix3f K);

void getDivergenceField(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f);

void setBoundaryConditions(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f);

void normalIntegration(aliceVision::image::Image<aliceVision::image::RGBfColor>& normals, aliceVision::image::Image<float>& depth, bool perspective, const Eigen::Matrix3f& K);

