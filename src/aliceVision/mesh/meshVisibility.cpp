// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "meshVisibility.hpp"

#include <geogram/points/kd_tree.h>

namespace aliceVision {
namespace mesh {

int getNearestVertices(const Mesh& refMesh, const Mesh& mesh, StaticVector<int>& out_nearestVertex)
{
    std::cout << "getNearestVertices begin" << std::endl;
    out_nearestVertex.resize(mesh.pts->size(), -1);

    GEO::AdaptiveKdTree refMesh_kdTree(3);
    refMesh_kdTree.set_points(refMesh.pts->size(), refMesh.pts->front().m);

    #pragma omp parallel for
    for(int i = 0; i < mesh.pts->size(); ++i)
    {
        out_nearestVertex[i] = refMesh_kdTree.get_nearest_neighbor((*mesh.pts)[i].m);
    }
    std::cout << "getNearestVertices end" << std::endl;
    return 0;
}


void remapMeshVisibilities(
    const Mesh& refMesh, const PointsVisibility& refPtsVisibilities,
    const Mesh& mesh, PointsVisibility& out_ptsVisibilities)
{
    std::cout << "remapMeshVisibility begin" << std::endl;

    GEO::AdaptiveKdTree refMesh_kdTree(3);
    refMesh_kdTree.set_points(refMesh.pts->size(), refMesh.pts->front().m);

    out_ptsVisibilities.resize(mesh.pts->size());

    #pragma omp parallel for
    for(int i = 0; i < mesh.pts->size(); ++i)
    {
        PointVisibility* pOut = new StaticVector<int>();
        out_ptsVisibilities[i] = pOut; // give ownership

        int iRef = refMesh_kdTree.get_nearest_neighbor((*mesh.pts)[i].m);
        if(iRef == -1)
            continue;
        PointVisibility* pRef = refPtsVisibilities[iRef];
        if(pRef == nullptr)
            continue;

        *pOut = *pRef;
    }

    std::cout << "remapMeshVisibility end" << std::endl;
}

} // namespace mesh
} // namespace aliceVision
