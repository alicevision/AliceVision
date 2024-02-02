// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>
#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_3d.h>
#include <geogram/mesh/mesh.h>
#include <geogram/basic/geometry_nd.h>
#include <aliceVision/mvsData/geometry.hpp>

#include <aliceVision/fuseCut/Intersections.hpp>

namespace aliceVision {
namespace fuseCut {

class GraphFiller
{
public:
    
    struct IntersectionHistory
    {
        size_t steps = 0;
        Point3d cam;
        Point3d originPt;
        Point3d dirVect;

        std::vector<GeometryIntersection> geometries;
        std::vector<Point3d> intersectPts;
        std::vector<Point3d> vecToCam;
        std::vector<float> distToCam;
        std::vector<float> angleToCam;
        IntersectionHistory(const Point3d& c, const Point3d& oPt, const Point3d& diV)
          : cam{c},
            originPt{oPt},
            dirVect{diV}
        {}

        void append(const GeometryIntersection& geom, const Point3d& intersectPt);
    };

    /**
     * @brief  Used for debug purposes to store count about geometries intersected during fillGraph and forceTedges.
     */
    struct GeometriesCount
    {
        size_t facets = 0;
        size_t vertices = 0;
        size_t edges = 0;

        GeometriesCount& operator+=(const GeometriesCount& gc)
        {
            edges += gc.edges;
            vertices += gc.vertices;
            facets += gc.facets;
            return *this;
        }
        GeometriesCount& operator/=(const size_t v)
        {
            edges /= v;
            vertices /= v;
            facets /= v;
            return *this;
        }
    };

public:
    GraphFiller(mvsUtils::MultiViewParams& mp) : _mp(mp)
    {
    }

    void createGraphCut(const Point3d hexah[8], const StaticVector<int>& cams);

private:
    void voteFullEmptyScore(const StaticVector<int>& cams);
    void addToInfiniteSw(float sW);
    void forceTedgesByGradientIJCV(float nPixelSizeBehind);
    void initCells();
    
    
private:
     GeometryIntersection intersectNextGeom(const GeometryIntersection& inGeometry,
                                           const Point3d& originPt,
                                           const Point3d& dirVect,
                                           Point3d& intersectPt,
                                           const double epsilonFactor,
                                           const Point3d& lastIntersectPt) const;

    GeometryIntersection rayIntersectTriangle(const Point3d& originPt,
                                              const Point3d& DirVec,
                                              const Facet& facet,
                                              Point3d& intersectPt,
                                              const double epsilonFactor,
                                              bool& ambiguous,
                                              const Point3d* lastIntersectPt = nullptr) const;


    inline const std::vector<CellIndex>& getNeighboringCellsByVertexIndex(VertexIndex vi) const { return _tetrahedralization.getNeighboringCellsPerVertex().at(vi); }

    std::vector<CellIndex> getNeighboringCellsByEdge(const Edge& e) const
    {
        const std::vector<CellIndex>& v0ci = getNeighboringCellsByVertexIndex(e.v0);
        const std::vector<CellIndex>& v1ci = getNeighboringCellsByVertexIndex(e.v1);

        std::vector<CellIndex> neighboringCells;
        std::set_intersection(v0ci.begin(), v0ci.end(), v1ci.begin(), v1ci.end(), std::back_inserter(neighboringCells));

        return neighboringCells;
    }

    std::vector<CellIndex> getNeighboringCellsByGeometry(const GeometryIntersection& g) const;

    inline VertexIndex getVertexIndex(const Facet& f, int i) const
    {
        return _tetrahedralization.cell_vertex(f.cellIndex, ((f.localVertexIndex + i + 1) % 4));
    }

    inline Facet mirrorFacet(const Facet& f) const
    {
        const std::array<VertexIndex, 3> facetVertices = {getVertexIndex(f, 0), getVertexIndex(f, 1), getVertexIndex(f, 2)};

        Facet out;
        out.cellIndex = _tetrahedralization.cell_adjacent(f.cellIndex, f.localVertexIndex);
        if (out.cellIndex != GEO::NO_CELL)
        {
            // Search for the vertex in adjacent cell which doesn't exist in input facet.
            for (int k = 0; k < 4; ++k)
            {
                CellIndex out_vi = _tetrahedralization.cell_vertex(out.cellIndex, k);
                if (std::find(facetVertices.begin(), facetVertices.end(), out_vi) == facetVertices.end())
                {
                    out.localVertexIndex = k;
                    return out;
                }
            }
        }
        return out;
    }

    std::vector<CellIndex> getNeighboringCellsByFacet(const Facet& f) const;

    void fillGraph(double nPixelSizeBehind, bool labatutWeights, bool fillOut, float distFcnHeight, float fullWeight);

    void fillGraphPartPtRc(int& out_nstepsFront,
                           int& out_nstepsBehind,
                           GeometriesCount& outFrontCount,
                           GeometriesCount& outBehindCount,
                           int vertexIndex,
                           int cam,
                           float weight,
                           float fullWeight,
                           double nPixelSizeBehind,
                           bool fillOut,
                           float distFcnHeight);
    float distFcn(float maxDist, float dist, float distFcnHeight) const;

public:
    mvsUtils::MultiViewParams& _mp;
    std::vector<GC_cellInfo> _cellsAttr;
    Tetrahedralization _tetrahedralization;
    std::vector<Point3d> _verticesCoords;
    std::vector<GC_vertexInfo> _verticesAttr;
};

}  // namespace fuseCut
}  // namespace aliceVision
