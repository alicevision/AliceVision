// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshIntersection.hpp"

#include <aliceVision/system/Logger.hpp>

#include <geogram/basic/geometry.h>
#include <geogram/mesh/mesh_io.h>
#include <geogram/mesh/mesh_geometry.h>

namespace aliceVision {
namespace mesh {


bool MeshIntersection::initialize(const std::string & pathToModel)
{
    GEO::initialize();
    if (!GEO::mesh_load(pathToModel, _mesh))
    {
        ALICEVISION_LOG_ERROR("Impossible to load object file at " << pathToModel);
        return false;
    }

    _aabb.initialize(_mesh);

    return true;
}

bool MeshIntersection::pickPoint(Vec3 & output, const camera::IntrinsicBase & intrinsic, const Vec2 & imageCoords)
{
    const Vec3 posCamera = _pose.center();
    const Vec3 wdir = intrinsic.backprojectTransform(imageCoords, true, _pose, 1.0);
    const Vec3 dir = (wdir - posCamera).normalized();
    
    //Create geogram ray from alicevision ray
    GEO::Ray ray;
    ray.origin.x = posCamera.x();
    ray.origin.y = -posCamera.y();
    ray.origin.z = -posCamera.z();
    ray.direction.x = dir.x();
    ray.direction.y = -dir.y();
    ray.direction.z = -dir.z();

    GEO::MeshFacetsAABB::Intersection intersection;
    if (!_aabb.ray_nearest_intersection(ray, intersection)) 
    {
        return false;
    }

    const GEO::vec3 p = ray.origin + intersection.t * ray.direction;

    output.x() = p.x;
    output.y() = -p.y;
    output.z() = -p.z;

    return true;
}

bool MeshIntersection::pickNormal(Vec3 & output, const camera::IntrinsicBase & intrinsic, const Vec2 & imageCoords)
{
    const Vec3 posCamera = _pose.center();
    const Vec3 wdir = intrinsic.backprojectTransform(imageCoords, true, _pose, 1.0);
    const Vec3 dir = (wdir - posCamera).normalized();
    
    //Create geogram ray from alicevision ray
    GEO::Ray ray;
    ray.origin.x = posCamera.x();
    ray.origin.y = -posCamera.y();
    ray.origin.z = -posCamera.z();
    ray.direction.x = dir.x();
    ray.direction.y = -dir.y();
    ray.direction.z = -dir.z();

    GEO::MeshFacetsAABB::Intersection intersection;
    if (!_aabb.ray_nearest_intersection(ray, intersection)) 
    {
        return false;
    }

    const GEO::vec3 n = GEO::normalize(intersection.N);

    output.x() = n.x;
    output.y() = -n.y;
    output.z() = -n.z;

    return true;
}

bool MeshIntersection::pickPointAndNormal(Vec3 & point, Vec3 & normal, const camera::IntrinsicBase & intrinsic, const Vec2 & imageCoords)
{
    const Vec3 posCamera = _pose.center();
    const Vec3 wdir = intrinsic.backprojectTransform(imageCoords, true, _pose, 1.0);
    const Vec3 dir = (wdir - posCamera).normalized();
    
    //Create geogram ray from alicevision ray
    GEO::Ray ray;
    ray.origin.x = posCamera.x();
    ray.origin.y = -posCamera.y();
    ray.origin.z = -posCamera.z();
    ray.direction.x = dir.x();
    ray.direction.y = -dir.y();
    ray.direction.z = -dir.z();

    GEO::MeshFacetsAABB::Intersection intersection;
    if (!_aabb.ray_nearest_intersection(ray, intersection)) 
    {
        return false;
    }

    const GEO::vec3 p = ray.origin + intersection.t * ray.direction;

    point.x() = p.x;
    point.y() = -p.y;
    point.z() = -p.z;

    const GEO::vec3 n = GEO::normalize(intersection.N);

    normal.x() = n.x;
    normal.y() = -n.y;
    normal.z() = -n.z;

    return true;
}

bool MeshIntersection::getPointAndNormal(Vec3 & point, Vec3 & normal, const Vec3 & origin, const Vec3 & direction)
{
    //Create geogram ray from alicevision ray
    GEO::Ray ray;

    ray.origin.x = origin.x();
    ray.origin.y = -origin.y();
    ray.origin.z = -origin.z();
    ray.direction.x = direction.x();
    ray.direction.y = -direction.y();
    ray.direction.z = -direction.z();

    GEO::MeshFacetsAABB::Intersection intersection;
    if (!_aabb.ray_nearest_intersection(ray, intersection)) 
    {
        return false;
    }

    const GEO::vec3 p = ray.origin + intersection.t * ray.direction;

    point.x() = p.x;
    point.y() = -p.y;
    point.z() = -p.z;

    const GEO::vec3 n = GEO::normalize(intersection.N);

    normal.x() = n.x;
    normal.y() = -n.y;
    normal.z() = -n.z;

    return true;
}

}
}