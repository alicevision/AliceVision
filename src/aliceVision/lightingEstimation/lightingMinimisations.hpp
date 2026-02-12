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
    double epsilon, 
    Eigen::Vector3f &lightingDirection);

void coarsePunctualLightEstimation(
    const Eigen::MatrixX3f& points, 
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    const Eigen::Vector3f& sceneCenter,
    const Eigen::Vector3f& lightingDirection,
    float lightingIntensity,
    double epsilon, 
    float &lightingDistance);

void pointSourceModelRefinement(
    const Eigen::MatrixX3f& points, 
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    double epsilon, 
    Eigen::Vector3f &lightPosition, 
    float &lightIntensity);

void coloredPointSourceModelRefinement(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::MatrixX3f& pixelsRGBIntensity, 
	double epsilon, 
	Eigen::Vector3f &lightingPosition, 
	Eigen::Vector3f &lightingRGBIntensity);

void LEDModelRefinement(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::MatrixX3f& pixelsRGBf, 
	double epsilon, 
	Eigen::Vector3f &lightingPosition, 
	Eigen::Vector3f &lightingDirection, 
	Eigen::Vector3f &lightingRGBIntensity,
	Eigen::Vector3f &lightingRGBAnisotropy);

} // namespace lightingEstimation
} // namespace aliceVision