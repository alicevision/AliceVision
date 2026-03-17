// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// Eigen
#include <Eigen/Dense>


namespace aliceVision {
namespace lightingEstimation {

void coarseDirectionnalLightEstimation(
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    double var, 
    Eigen::Vector3f &lightingDirection);

void coloredDirectionnalLightEstimation(
    const Eigen::MatrixX3f& normals, 
    const Eigen::MatrixX3f& pixelsIntensity, 
    const Eigen::Vector3f &lightingDirection,
    double epsilon, 
    Eigen::Vector3f &lightingIntensity);

void coarsePunctualLightEstimation(
    const Eigen::MatrixX3f& points, 
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    const Eigen::Vector3f& sceneCenter,
    const Eigen::Vector3f& lightingDirection,
    float lightingIntensity,
    double var, 
    float &lightingDistance);

void pointSourceModelRefinement(
    const Eigen::MatrixX3f& points, 
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    double var, 
    Eigen::Vector3f &lightPosition, 
    float &lightIntensity);

void coloredPointSourceModelRefinement(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::MatrixX3f& pixelsRGBIntensity, 
	const Eigen::Vector3f &lightingPosition, 
	double epsilon, 
	Eigen::Vector3f &lightingRGBIntensity);

} // namespace lightingEstimation
} // namespace aliceVision