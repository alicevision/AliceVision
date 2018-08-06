// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "meshVisibility.hpp"
#include "geoMesh.hpp"

#include <aliceVision/system/Logger.hpp>

#include <geogram/basic/permutation.h>
#include <geogram/basic/attributes.h>
#include <geogram/points/kd_tree.h>
#include <geogram/mesh/mesh_AABB.h>
#include <geogram/mesh/mesh_reorder.h>


namespace aliceVision {
namespace mesh {

int getNearestVertices(const Mesh& refMesh, const Mesh& mesh, StaticVector<int>& out_nearestVertex)
{
    ALICEVISION_LOG_DEBUG("getNearestVertices start.");
    out_nearestVertex.resize(mesh.pts->size(), -1);

    GEO::AdaptiveKdTree refMesh_kdTree(3);
    refMesh_kdTree.set_points(refMesh.pts->size(), refMesh.pts->front().m);

    #pragma omp parallel for
    for(int i = 0; i < mesh.pts->size(); ++i)
    {
        out_nearestVertex[i] = refMesh_kdTree.get_nearest_neighbor((*mesh.pts)[i].m);
    }
    ALICEVISION_LOG_DEBUG("getNearestVertices done.");
    return 0;
}


void remapMeshVisibilities_pullVerticesVisibility(
    const Mesh& refMesh, const PointsVisibility& refPtsVisibilities,
    const Mesh& mesh, PointsVisibility& out_ptsVisibilities)
{
    ALICEVISION_LOG_DEBUG("remapMeshVisibility based on closest vertex start.");

    GEO::AdaptiveKdTree refMesh_kdTree(3);
    refMesh_kdTree.set_points(refMesh.pts->size(), refMesh.pts->front().m);

    out_ptsVisibilities.resize(mesh.pts->size(), nullptr);

    #pragma omp parallel for
    for(int i = 0; i < mesh.pts->size(); ++i)
    {
        PointVisibility* pOut = out_ptsVisibilities[i];
        if(pOut == nullptr)
        {
            pOut = new StaticVector<int>();
            out_ptsVisibilities[i] = pOut; // give ownership
        }

        int iRef = refMesh_kdTree.get_nearest_neighbor((*mesh.pts)[i].m);
        if(iRef == -1)
            continue;
        PointVisibility* pRef = refPtsVisibilities[iRef];
        if(pRef == nullptr)
            continue;

        *pOut = *pRef;
    }

    ALICEVISION_LOG_DEBUG("remapMeshVisibility done.");
}


double mesh_facet_edges_length(const GEO::Mesh &M, GEO::index_t f)
{
    const GEO::vec3& p0 = M.vertices.point(M.facets.vertex(f, 0));
    const GEO::vec3& p1 = M.vertices.point(M.facets.vertex(f, 1));
    const GEO::vec3& p2 = M.vertices.point(M.facets.vertex(f, 2));
    return (p1 - p0).length() + (p2 - p1).length() + (p0 - p2).length();
}

void remapMeshVisibilities_pushVerticesVisibilityToTriangles(
    const Mesh& refMesh, const PointsVisibility& refPtsVisibilities,
    const Mesh& mesh, PointsVisibility& out_ptsVisibilities)
{
    ALICEVISION_LOG_INFO("remapMeshVisibility based on triangles start.");

    GEO::initialize();
    GEO::Mesh meshG;
    toGeoMesh(mesh, meshG);

    // MeshFacetAABB will reorder the mesh, so we need to keep indices
    GEO::Attribute<GEO::index_t> reorderedVerticesAttr(meshG.vertices.attributes(), "reorder");
    for(int i = 0; i < meshG.vertices.nb(); ++i)
        reorderedVerticesAttr[i] = i;

    GEO::MeshFacetsAABB meshAABB(meshG);  // warning: mesh_reorder called inside

    GEO::vector<GEO::index_t> reorderedVertices = reorderedVerticesAttr.get_vector();

    if (out_ptsVisibilities.size() != mesh.pts->size())
    {
        out_ptsVisibilities.resize(mesh.pts->size(), nullptr);
        for (int vi = 0; vi < mesh.pts->size(); ++vi)
        {
            if(out_ptsVisibilities[vi] == nullptr)
                out_ptsVisibilities[vi] = new StaticVector<int>(); // create and give ownership
        }
    }

    #pragma omp parallel for
    for (int rvi = 0; rvi < refMesh.pts->size(); ++rvi)
    {
        PointVisibility* rpVis = refPtsVisibilities[rvi];
        if (rpVis == nullptr)
            continue;

        const GEO::vec3 rp((*refMesh.pts)[rvi].m);
        GEO::vec3 nearestPoint;
        double dist2 = 0.0;
        GEO::index_t f = meshAABB.nearest_facet(rp, nearestPoint, dist2);
        if(f == GEO::NO_FACET)
            continue;

        double avgEdgeLength = mesh_facet_edges_length(meshG, f) / 3.0;
        // if average edge length is larger than the distance between the output mesh
        // and the closest point in the reference mesh.
        if(std::sqrt(dist2) > avgEdgeLength)
            continue;

        #pragma omp critical
        {
            for (int i = 0; i < 3; ++i)
            {
                GEO::index_t v = meshG.facets.vertex(f, i);
                if (v == GEO::NO_VERTEX)
                    continue;
                PointVisibility* pOut = out_ptsVisibilities[reorderedVertices[v]];
                
                for(int j = 0; j < rpVis->size(); ++j)
                    pOut->push_back_distinct((*rpVis)[j]);
            }
        }
    }

    ALICEVISION_LOG_INFO("remapMeshVisibility done.");
}

} // namespace mesh
} // namespace aliceVision
