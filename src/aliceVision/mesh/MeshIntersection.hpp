// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>

#include <geogram/mesh/mesh.h>
#include <geogram/mesh/mesh_AABB.h>

namespace aliceVision {
namespace mesh {

class MeshIntersection
{
public:
    /**
     * @brief initialize object. to be called before any other methodd
     * @param pathToModel path to obj file to use as mesh
    */
    bool initialize(const std::string & pathToModel);

    /**
     * @brief Update pose to use for peeking
     * @param pose transformation to use (in aliceVision standard form)
    */
    void setPose(const geometry::Pose3 & pose)
    {
        _pose = pose;
    }

    /**
     * @brief peek a point on the mesh given a input camera observation
     * @param output the output measured point
     * @param intrinsic the camera intrinsics to use for ray computation
     * @param imageCoords the camera observation we want to use to estimate its 'depth'
     * @return true if the ray intersect the mesh.
    */
    bool peekPoint(Vec3 & output, const camera::IntrinsicBase & intrinsic, const Vec2 & imageCoords);

    /**
     * @brief peek a point and get its normal on the mesh given a input camera observation
     * @param output the output measured normal
     * @param intrinsic the camera intrinsics to use for ray computation
     * @param imageCoords the camera observation we want to use to estimate its 'depth'
     * @return true if the ray intersect the mesh.
    */
    bool peekNormal(Vec3 & output, const camera::IntrinsicBase & intrinsic, const Vec2 & imageCoords);

private:
    GEO::Mesh _mesh;
    GEO::MeshFacetsAABB _aabb;
    geometry::Pose3 _pose;
};

}
}