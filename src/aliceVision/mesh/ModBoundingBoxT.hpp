// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <float.h>
#include <OpenMesh/Tools/Decimater/ModBaseT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <aliceVision/geometry/Intersection.hpp>
#include <Eigen/Dense>

namespace OpenMesh {
namespace Decimater {

template<class MeshT>
class ModBoundingBoxT : public ModBaseT<MeshT>
{
  public:
    // Defines the types Self, Handle, Base, Mesh, and CollapseInfo
    // and the memberfunction name()
    DECIMATING_MODULE(ModBoundingBoxT, MeshT, BoundingBox);

  public:
    /** Constructor
     *  \internal
     */
    ModBoundingBoxT(MeshT& _mesh)
      : Base(_mesh, true)
    {
        _bbMin.fill(std::numeric_limits<double>::lowest());
        _bbMax.fill(std::numeric_limits<double>::max());
    }

    /// Destructor
    virtual ~ModBoundingBoxT() {}

    void setBoundingBox(const Eigen::Vector3d& bbMin, const Eigen::Vector3d& bbMax)
    {
        _bbMin = bbMin;
        _bbMax = bbMax;
    }

    virtual float collapse_priority(const CollapseInfo& _ci)
    {
        Eigen::Vector3d pt;
        pt.x() = _ci.p0[0];
        pt.y() = _ci.p0[1];
        pt.z() = _ci.p0[2];

        if (!aliceVision::geometry::isPointInsideAABB(_bbMin, _bbMax, pt))
        {
            return Base::ILLEGAL_COLLAPSE;
        }

        return Base::LEGAL_COLLAPSE;
    }

  private:
    Eigen::Vector3d _bbMin;
    Eigen::Vector3d _bbMax;
};

}  // namespace Decimater
}  // namespace OpenMesh
