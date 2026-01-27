// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/mesh/MeshIntersection.hpp>

namespace aliceVision 
{
namespace sfmData
{

//This intermediate class is used as a proxy to not link
//sfm with mesh library
class MeshPointFetcher : public PointFetcher
{
public:
    /**
     * @brief initialize object. to be called before any other method
     * @param pathToModel path to obj file to use as mesh
    */
    bool initialize(const std::string & pathToModel)
    {
        return _mi.initialize(pathToModel);
    }

    /**
     * @brief Set the pose of the camera
     * @param pose the pose of the camera wrt some global coordinates frame
    */
    void setPose(const geometry::Pose3 & pose) override
    {
        _mi.setPose(pose);
    }

    /**
     * @brief pick a point on the mesh given a input camera observation
     * @param output the output measured point
     * @param intrinsic the camera intrinsics to use for ray computation
     * @param imageCoords the camera observation we want to use to estimate its 'depth'
     * @return true if the ray intersects the mesh.
    */
    bool pickPoint(Vec3 & output, const camera::IntrinsicBase & intrinsic, const Vec2 & imageCoords) override
    {
        return _mi.pickPoint(output, intrinsic, imageCoords);
    }

    /**
     * @brief method to get coordinates and normals of a pixel of an image
     * @param point result point in some global coordinates frame
     * @param normal result normal in some global coordinates frame
     * @param intrinsic the camera intrinsic object
     * @param imageCoords the input image pixel coordinates in 2D.
     * @return false on error
    */
    bool pickPointAndNormal(Vec3 & point,
                                Vec3 & normal,
                                const camera::IntrinsicBase & intrinsic,
                                const Vec2 & imageCoords) override
    {
        return _mi.pickPointAndNormal(point, normal, intrinsic, imageCoords);
    }

    /**
     * @brief get a point and get its normal on the mesh given 3D ray
     * @param point the output measured point
     * @param normal the output measured normal
     * @param origin the ray origin
     * @param direction the ray direction
     * @return true if the ray intersects the mesh.
    */
    bool getPointAndNormal(Vec3 & point, Vec3 & normal, const Vec3 & origin, const Vec3 & direction) override
    {
        return _mi.getPointAndNormal(point, normal, origin, direction);
    }

private:
    mesh::MeshIntersection _mi;
};

}
}