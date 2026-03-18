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
/**
 * Estimate an initial directional light direction from grayscale intensities and surface normals.
 * @param[in] normals Surface normals associated with the observations.
 * @param[in] pixelsIntensity Grayscale intensity observed at each normal.
 * @param[out] lightingDirection Output estimated light direction.
 * @param[in] epsilonHuberLoss Huber loss threshold used for robust fitting.
 */
void coarseDirectionnalLightEstimation(
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    Eigen::Vector3f &lightingDirection, 
	double epsilonHuberLoss = 2./255.);

/**
 * Estimate RGB directional light intensity for a fixed light direction.
 * @param[in] normals Surface normals associated with the observations.
 * @param[in] pixelsIntensity RGB intensity observed at each normal.
 * @param[in] lightingDirection Fixed light direction used during estimation.
 * @param[out] lightingIntensity Input estimation/Output estimated RGB light intensity.
 * @param[in] epsilonHuberLoss Huber loss threshold used for robust fitting.
 */
void coloredDirectionnalLightEstimation(
    const Eigen::MatrixX3f& normals, 
    const Eigen::MatrixX3f& pixelsIntensity, 
    const Eigen::Vector3f &lightingDirection, 
    Eigen::Vector3f &lightingIntensity,
    double epsilonHuberLoss = 2.0/255.);

/**
 * Estimate an initial point-light distance from grayscale observations and a known direction.
 * @param[in] points 3D points associated with the observations.
 * @param[in] normals Surface normals at the input points.
 * @param[in] pixelsIntensity Grayscale intensity observed at each point.
 * @param[in] sceneCenter Scene reference center used for the initialization.
 * @param[in] lightingDirection Known light direction.
 * @param[in] lightingIntensity Fixed scalar light intensity.
 * @param[out] lightingDistance Input estimation/Output estimated distance from the scene center.
 * @param[in] varTerminator Variance threshold used to identify terminator pixels.
 */
void coarsePunctualLightEstimation(
    const Eigen::MatrixX3f& points, 
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    const Eigen::Vector3f& sceneCenter,
    const Eigen::Vector3f& lightingDirection,
    const float lightingIntensity, 
    float &lightingDistance,
    double varTerminator = 0.01);

/**
 * Refine point-light position and scalar intensity from grayscale observations.
 * @param[in] points 3D points associated with the observations.
 * @param[in] normals Surface normals at the input points.
 * @param[in] pixelsIntensity Grayscale intensity observed at each point.
 * @param[out] lightPosition Input estimation/Output estimated point-light position.
 * @param[out] lightIntensity Input estimation/Output estimated scalar light intensity.
 * @param[in] varTerminator Variance threshold used to identify terminator pixels.
 */
void pointSourceModelRefinement(
    const Eigen::MatrixX3f& points, 
    const Eigen::MatrixX3f& normals, 
    const Eigen::VectorXf& pixelsIntensity, 
    Eigen::Vector3f &lightPosition, 
    float &lightIntensity, 
    double varTerminator = 0.01);

/**
 * Estimate RGB point-light intensity for a fixed point-light position.
 * @param[in] points 3D points associated with the observations.
 * @param[in] normals Surface normals at the input points.
 * @param[in] pixelsRGBIntensity RGB intensity observed at each point.
 * @param[in] lightingPosition Fixed point-light position used during estimation.
 * @param[out] lightingRGBIntensity Input estimation/Output estimated RGB light intensity.
 * @param[in] epsilonHuberLoss Huber loss threshold used for robust fitting.
 */
void coloredPointSourceModelRefinement(
	const Eigen::MatrixX3f& points, 
	const Eigen::MatrixX3f& normals, 
	const Eigen::MatrixX3f& pixelsRGBIntensity, 
	const Eigen::Vector3f &lightingPosition, 
	Eigen::Vector3f &lightingRGBIntensity, 
	double epsilonHuberLoss = 2./255.);

} // namespace lightingEstimation
} // namespace aliceVision
