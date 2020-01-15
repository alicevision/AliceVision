#pragma once

#include <aliceVision/mesh/Mesh.hpp>

#include <geogram/mesh/mesh.h>


namespace aliceVision {
namespace mesh {

/**
* @brief Create a Geogram GEO::Mesh from an aliceVision::Mesh
*
* @note only initialize vertices and facets
* @param[in] the source aliceVision mesh
* @param[out] the destination GEO::Mesh
*/
inline void toGeoMesh(const Mesh& src, GEO::Mesh& dst)
{
    GEO::vector<double> vertices;
    vertices.reserve(src.pts.size() * 3);
    GEO::vector<GEO::index_t> facets;
    facets.reserve(src.tris.size() * 3);

    for (unsigned int i = 0; i < src.pts.size(); ++i)
    {
        const auto& point = src.pts[i];
        vertices.insert(vertices.end(), std::begin(point.m), std::end(point.m));
    }

    for (unsigned int i = 0; i < src.tris.size(); ++i)
    {
        const auto& tri = src.tris[i];
        facets.insert(facets.end(), std::begin(tri.v), std::end(tri.v));
    }

    dst.facets.assign_triangle_mesh(3, vertices, facets, true);
    dst.facets.connect();

    assert(src.pts.size() == dst.vertices.nb());
    assert(src.tris.size() == dst.facets.nb());
}

}
}
