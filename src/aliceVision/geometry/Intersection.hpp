// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <Eigen/Dense>

namespace aliceVision {
namespace geometry {

/**
 * @brief Find the intersection of a ray (composed of a starting point and a direction) with the unit sphere centered on (0,0,0).
 * @param coordinates the coordinates of the intersection closest to the starting point
 * @param start the starting point of the ray
 * @param direction the direction of the ray
 * @return true if an intersection is found
 */
bool rayIntersectUnitSphere(Vec3& coordinates, const Vec3& start, const Vec3& direction);

/**
 * @brief Check if a point is inside an axis aligned bounding box
 * @param bbMin the minimal values of the bounding box on each axis
 * @param bbMax the maximal values of the bounding box on each axis
 * @return true if the point is inside the bounding box
 */
inline bool isPointInsideAABB(const Eigen::Vector3d& bbMin, const Eigen::Vector3d& bbMax, const Eigen::Vector3d& pt)
{
    if (pt.x() + 1e-12 < bbMin.x())
        return false;
    if (pt.y() + 1e-12 < bbMin.y())
        return false;
    if (pt.z() + 1e-12 < bbMin.z())
        return false;
    if (pt.x() - 1e-12 > bbMax.x())
        return false;
    if (pt.y() - 1e-12 > bbMax.y())
        return false;
    if (pt.z() - 1e-12 > bbMax.z())
        return false;

    return true;
}

/**
 * @brief Compute the two intersections of the rays with the bounding box
 * @param bbMin the minimal values of the bounding box on each axis
 * @param bbMax the maximal values of the bounding box on each axis
 * @param start the starting coordinates for the ray
 * @param direction the direction of the ray
 * @param boundsMin the scale of the direction vector for which the ray enters the bounding box
 * @param boundsMin the scale of the direction vector for which the ray leaves the bounding box
 * @return false if the ray does not intersect the bounding box
 */
bool rayIntersectAABB(const Eigen::Vector3d& bbMin,
                      const Eigen::Vector3d& bbMax,
                      const Eigen::Vector3d& start,
                      const Eigen::Vector3d& direction,
                      double& boundsMin,
                      double& boundsMax);

/**
 * @brief Is my segment intersecting the bounding box?
 * @param bbMin the minimal values of the bounding box on each axis
 * @param bbMax the maximal values of the bounding box on each axis
 * @param start the starting coordinates for the segment
 * @param end the ending coordinates for the segment
 * @return true if the segment intersect the bounding box
 */
bool isSegmentIntersectAABB(const Eigen::Vector3d& bbMin, const Eigen::Vector3d& bbMax, const Eigen::Vector3d& start, const Eigen::Vector3d& end);

/**
 * @brief Is my segment intersecting the bounding box?
 * @param inputbbMin1 the minimal values of the first bounding box on each axis
 * @param inputbbMax1 the maximal values of the first bounding box on each axis
 * @param inputbbMin2 the minimal values of the second bounding box on each axis
 * @param inputbbMax2 the maximal values of the second bounding box on each axis
 * @param bbMin the minimal values of the bounding box on each axis which is the intersection of both bounding box
 * @param bbMax the maximal values of the bounding box on each axis which is the intersection of both bounding box
 * @return true if the bounding box intersects
 */
bool intersectionBetweenAABB(const Eigen::Vector3d& inputbbMin1,
                             const Eigen::Vector3d& inputbbMax1,
                             const Eigen::Vector3d& inputbbMin2,
                             const Eigen::Vector3d& inputbbMax2,
                             Eigen::Vector3d& bbMin,
                             Eigen::Vector3d& bbMax);

}  // namespace geometry
}  // namespace aliceVision
