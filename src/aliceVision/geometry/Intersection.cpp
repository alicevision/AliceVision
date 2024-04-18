// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Intersection.hpp"

namespace aliceVision {
namespace geometry {

bool rayIntersectUnitSphere(Vec3& coordinates, const Vec3& start, const Vec3& direction)
{
    /*
    "Which point on the sphere relates to this point ?"
    (lambda * directionx + startx)^2
    + (lambda * directiony + starty)^2
    + (lambda * directionz + startz)^2=1

    (lambda * directionx)^2 + startx^2 + 2.0 * lambda * directionoriginx * directionx
    + (lambda * directiony)^2 + starty^2 + 2.0 * lambda * directionoriginy * directiony
    + (lambda * directionz)^2 + startz^2 + 2.0 * lambda * directionoriginz * directionz
    = 1

    (lambda^2) * (directionx^2 + directiony^2 + directionz^2)
    + lambda * (2.0 * directionoriginx * directionx + 2.0 * directionoriginy * directiony + 2.0 * directionoriginz * directionz)
    + (startx^2 + startx^2 + startx^2) - 1 = 0
    */

    double a = direction.dot(direction);
    double b = direction.dot(start) * 2.0;
    double c = start.dot(start) - 1.0;
    double det = b * b - 4.0 * a * c;

    if (det < 0)
    {
        return false;
    }

    double x1 = (-b + sqrt(det)) / (2.0 * a);
    double x2 = (-b - sqrt(det)) / (2.0 * a);
    double lambda = std::min(x1, x2);

    coordinates = start + lambda * direction;

    return true;
}

bool rayIntersectAABB(const Eigen::Vector3d& bbMin,
                      const Eigen::Vector3d& bbMax,
                      const Eigen::Vector3d& start,
                      const Eigen::Vector3d& direction,
                      double& boundsMin,
                      double& boundsMax)
{
    // x+lambda*dx>bbmin.x
    // y+lambda*dy>bbmin.y
    // z+lambda*dz>bbmin.z
    // x+lambda*dx<bbmax.x
    // y+lambda*dy<bbmax.y
    // z+lambda*dz<bbmax.z

    // lambda > (bbmin.x-x)/dx
    // lambda > (bbmin.y-y)/dy
    // lambda > (bbmin.z-z)/dz
    // lambda < (bbmax.x-x)/dx
    // lambda < (bbmax.y-y)/dy
    // lambda < (bbmax.z-z)/dz

    boundsMin = std::numeric_limits<double>::lowest();
    boundsMax = std::numeric_limits<double>::max();

    if (std::abs(direction.x()) > 1e-12)
    {
        double lmin = (bbMin.x() - start.x()) / direction.x();
        double lmax = (bbMax.x() - start.x()) / direction.x();

        if (direction.x() > 0.0)
        {
            boundsMin = std::max(boundsMin, lmin);
            boundsMax = std::min(boundsMax, lmax);
        }
        else
        {
            boundsMin = std::max(boundsMin, lmax);
            boundsMax = std::min(boundsMax, lmin);
        }
    }
    else
    {
        if (start.x() < bbMin.x() || start.x() > bbMax.x())
        {
            return false;
        }
    }

    if (std::abs(direction.y()) > 1e-12)
    {
        double lmin = (bbMin.y() - start.y()) / direction.y();
        double lmax = (bbMax.y() - start.y()) / direction.y();

        if (direction.y() > 0.0)
        {
            boundsMin = std::max(boundsMin, lmin);
            boundsMax = std::min(boundsMax, lmax);
        }
        else
        {
            boundsMin = std::max(boundsMin, lmax);
            boundsMax = std::min(boundsMax, lmin);
        }
    }
    else
    {
        if (start.y() < bbMin.y() || start.y() > bbMax.y())
        {
            return false;
        }
    }

    if (std::abs(direction.z()) > 1e-12)
    {
        double lmin = (bbMin.z() - start.z()) / direction.z();
        double lmax = (bbMax.z() - start.z()) / direction.z();

        if (direction.z() > 0.0)
        {
            boundsMin = std::max(boundsMin, lmin);
            boundsMax = std::min(boundsMax, lmax);
        }
        else
        {
            boundsMin = std::max(boundsMin, lmax);
            boundsMax = std::min(boundsMax, lmin);
        }
    }
    else
    {
        if (start.z() < bbMin.z() || start.z() > bbMax.z())
        {
            return false;
        }
    }

    return true;
}

bool isSegmentIntersectAABB(const Eigen::Vector3d& bbMin, const Eigen::Vector3d& bbMax, const Eigen::Vector3d& start, const Eigen::Vector3d& end)
{
    double boundsMin, boundsMax;

    // Compute direction
    Eigen::Vector3d direction = end - start;
    double scale = direction.norm();
    if (scale < 1e-12)
    {
        return false;
    }
    direction = direction / scale;

    if (!rayIntersectAABB(bbMin, bbMax, start, direction, boundsMin, boundsMax))
    {
        return false;
    }

    if (boundsMax < 0.0)
    {
        return false;
    }

    if (boundsMax > scale)
    {
        return false;
    }

    return (boundsMin < boundsMax);
}

bool intersectionBetweenAABB(const Eigen::Vector3d& inputbbMin1,
                             const Eigen::Vector3d& inputbbMax1,
                             const Eigen::Vector3d& inputbbMin2,
                             const Eigen::Vector3d& inputbbMax2,
                             Eigen::Vector3d& bbMin,
                             Eigen::Vector3d& bbMax)
{
    bbMin.x() = std::max(inputbbMin1.x(), inputbbMin2.x());
    bbMax.x() = std::min(inputbbMax1.x(), inputbbMax2.x());
    bbMin.y() = std::max(inputbbMin1.y(), inputbbMin2.y());
    bbMax.y() = std::min(inputbbMax1.y(), inputbbMax2.y());
    bbMin.z() = std::max(inputbbMin1.z(), inputbbMin2.z());
    bbMax.z() = std::min(inputbbMax1.z(), inputbbMax2.z());

    if (bbMin.x() >= bbMax.x())
    {
        return false;
    }

    if (bbMin.y() >= bbMax.y())
    {
        return false;
    }

    if (bbMin.z() >= bbMax.z())
    {
        return false;
    }

    return true;
}

}  // namespace geometry
}  // namespace aliceVision
