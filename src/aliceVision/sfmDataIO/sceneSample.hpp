// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/SfMData.hpp>

namespace aliceVision {
namespace sfmDataIO {

/// Type of synthetic scene geometry to generate.
enum ESceneType
{
    SCENE_CUBE,            ///< Scene whose landmarks are placed on the faces of a cube.
    SCENE_SPHERE,          ///< Scene whose landmarks are distributed over a sphere.
    SCENE_PLANAR,          ///< Planar / bas-relief scene: all landmarks lie on the z=0 plane, making depth poorly constrained.
    SCENE_COLLINEAR,       ///< Collinear cameras: all camera centres lie on a single line, producing strongly anisotropic covariance.
    SCENE_FORWARD_MOTION,  ///< Forward-motion scene: cameras move along their optical axis, making depth and lateral translation hard to separate.
    SCENE_NARROW_BASELINE, ///< Narrow-baseline scene: very small camera circle radius relative to scene depth → high depth uncertainty.
    SCENE_UNEVEN_COVERAGE, ///< Uneven-coverage scene: some landmarks seen by all cameras, others by only two adjacent cameras.
    SCENE_PURE_ROTATION,          ///< Pure-rotation scene: all cameras share the same centre; radial depth of every landmark is unobservable.
    SCENE_SPARSE_OBSERVATIONS,    ///< Sparse-observations scene: each camera observes only a very small number of landmarks, so camera positions are weakly constrained and carry high positional uncertainty.
    SCENE_CLUSTERED_OBSERVATIONS, ///< Clustered-observations scene: all landmark observations fall within a tiny 15% x 15% region of the image, creating severe directional ambiguity and high uncertainty.
};

/// @brief Convert a string representation to the corresponding ESceneType enum value.
/// @param SScene String name of the scene type (e.g. "cube" or "sphere").
/// @return The matching ESceneType value.
ESceneType ESceneType_stringToEnum(const std::string& SScene);

/// @brief Convert an ESceneType enum value to its string representation.
/// @param EScene The scene type enum value to convert.
/// @return A string name for the given scene type.
std::string ESceneType_enumToString(const ESceneType EScene);

/// @brief Stream insertion operator for ESceneType (writes the string representation).
std::ostream& operator<<(std::ostream& os, ESceneType p);

/// @brief Stream extraction operator for ESceneType (reads from string representation).
std::istream& operator>>(std::istream& in, ESceneType& p);

/**
 * @brief Create an SfmData with some arbitrary content.
 * This is used in unit tests to validate read/write sfmData files.
 * @param[out] output The SfMData structure to populate.
 * @param scene The type of synthetic scene to generate (default: SCENE_CUBE).
 */
void generateSampleScene(sfmData::SfMData& output, ESceneType scene=ESceneType::SCENE_CUBE);

/**
 * @brief Populate an SfMData with a synthetic cube scene.
 * Landmarks are placed at the corners and face centres of a unit cube,
 * with cameras positioned around it.
 * @param[out] output The SfMData structure to populate.
 */
void generateCubeScene(sfmData::SfMData& output);

/**
 * @brief Populate an SfMData with a synthetic sphere scene.
 * Landmarks are distributed over the surface of a sphere, with cameras
 * arranged uniformly around it.
 * @param[out] output  The SfMData structure to populate.
 * @param pointsNb     Number of 3D landmarks to generate.
 * @param posesNb      Number of camera poses to generate.
 */
void generateSphereScene(sfmData::SfMData& output, int pointsNb, int posesNb);

/**
 * @brief Populate an SfMData with a planar (bas-relief) scene.
 * All landmarks lie on the z=0 plane. Cameras look down at the plane from a circle
 * above it. The depth direction (Z) of each landmark is poorly constrained, mimicking
 * the bas-relief ambiguity seen in real photogrammetric reconstructions.
 * @param[out] output   The SfMData structure to populate.
 * @param gridSize      Side length of the square grid of landmarks (total = gridSize²).
 * @param posesNb       Number of camera poses arranged in a circle above the plane.
 */
void generatePlanarScene(sfmData::SfMData& output, int gridSize, int posesNb);

/**
 * @brief Populate an SfMData with a collinear-cameras scene.
 * All camera centres lie on the X axis. The scene has no Y or Z baseline, so
 * the covariance of landmark positions is strongly anisotropic: uncertainty
 * is much larger in the direction orthogonal to the baseline.
 * @param[out] output   The SfMData structure to populate.
 * @param pointsNb      Number of 3D landmarks to generate.
 * @param posesNb       Number of camera poses uniformly spaced along the X axis.
 */
void generateCollinearCamerasScene(sfmData::SfMData& output, int pointsNb, int posesNb);

/**
 * @brief Populate an SfMData with a forward-motion (corridor) scene.
 * All cameras move along their common optical axis (+Z). Because the baseline
 * is parallel to the viewing direction, depth and the translational component
 * along Z are poorly decorrelated, producing high depth uncertainty.
 * @param[out] output   The SfMData structure to populate.
 * @param pointsNb      Number of 3D landmarks to generate.
 * @param posesNb       Number of camera poses uniformly spaced along the Z axis.
 */
void generateForwardMotionScene(sfmData::SfMData& output, int pointsNb, int posesNb);

/**
 * @brief Populate an SfMData with a narrow-baseline scene.
 * Identical geometry to the sphere scene, but the camera circle radius is
 * very small (0.05) relative to the scene depth (unit sphere). The large
 * depth-to-baseline ratio causes high depth uncertainty for all landmarks.
 * @param[out] output   The SfMData structure to populate.
 * @param pointsNb      Number of 3D landmarks to generate.
 * @param posesNb       Number of camera poses uniformly spaced on the narrow circle.
 */
void generateNarrowBaselineScene(sfmData::SfMData& output, int pointsNb, int posesNb);

/**
 * @brief Populate an SfMData with an uneven-coverage scene.
 * @c wellObservedNb landmarks (near the origin) are seen by every camera.
 * @c poorlyObservedNb landmarks (on the unit sphere surface) are each seen by
 * only two adjacent cameras, making their covariances much larger than those of
 * the well-observed set. This highlights non-uniform uncertainty in a single scene.
 * @param[out] output           The SfMData structure to populate.
 * @param wellObservedNb        Number of landmarks observed by all cameras.
 * @param poorlyObservedNb      Number of landmarks each observed by exactly 2 cameras.
 * @param posesNb               Number of camera poses arranged in a circle.
 */
void generateUnevenCoverageScene(sfmData::SfMData& output, int wellObservedNb, int poorlyObservedNb, int posesNb);

/**
 * @brief Populate an SfMData with a pure-rotation scene.
 * All camera poses share the same centre (the origin) and differ only in
 * orientation. The Jacobian derivative with respect to the radial depth of
 * any landmark is identically zero under perspective projection, so depths
 * are completely unobservable. The resulting covariance is dominated by
 * the regularisation term used to handle the gauge freedom.
 * @param[out] output   The SfMData structure to populate.
 * @param pointsNb      Number of 3D landmarks to generate.
 * @param posesNb       Number of camera poses with distinct rotations.
 */
void generatePureRotationScene(sfmData::SfMData& output, int pointsNb, int posesNb);

/**
 * @brief Populate an SfMData with a sparse-observations scene.
 * Cameras are arranged on a circle of radius 10, but each camera observes
 * only @p obsPerCamera landmarks (round-robin assignment). With so few
 * geometric constraints per camera, the camera translation parameters are
 * weakly determined by the bundle-adjustment Jacobian and carry high
 * positional uncertainty, while landmarks (each seen by multiple cameras)
 * remain reasonably well constrained.
 * @param[out] output      The SfMData structure to populate.
 * @param pointsNb         Number of 3D landmarks distributed near the origin.
 * @param posesNb          Number of camera poses arranged in a circle.
 * @param obsPerCamera     Number of landmarks each camera observes (must be > 0
 *                         and <= pointsNb).
 */
void generateSparseCameraObservationsScene(sfmData::SfMData& output, int pointsNb, int posesNb, int obsPerCamera);

/**
 * @brief Populate an SfMData with a clustered-observations scene.
 * Landmarks are positioned such that all their image-space observations fall
 * within a tiny region (approximately 15% × 15% of image dimensions).
 * This creates severe directional ambiguity: all cameras observe features in
 * nearly the same image direction, leading to high uncertainty in depth and
 * the lateral position of landmarks in the plane perpendicular to the viewing direction.
 * @param[out] output      The SfMData structure to populate.
 * @param pointsNb         Number of 3D landmarks to generate.
 * @param posesNb          Number of camera poses arranged in a circle.
 */
void generateClusteredObservationsScene(sfmData::SfMData& output, int pointsNb, int posesNb);

}  // namespace sfmDataIO
}  // namespace aliceVision
