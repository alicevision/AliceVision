// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MeshIntersection.hpp"

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

bool MeshIntersection::peekPoint(Vec3 & output, const camera::IntrinsicBase & intrinsic, const Vec2 & imageCoords)
{
    const Vec3 posCamera = _pose.center();
    const Vec3 wdir = intrinsic.backproject(imageCoords, true, _pose, 1.0);
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

bool MeshIntersection::peekNormal(Vec3 & output, const camera::IntrinsicBase & intrinsic, const Vec2 & imageCoords)
{
    const Vec3 posCamera = _pose.center();
    const Vec3 wdir = intrinsic.backproject(imageCoords, true, _pose, 1.0);
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

}
}