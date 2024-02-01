// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>
#include <aliceVision/mesh/Mesh.hpp>

#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_3d.h>
#include <geogram/mesh/mesh.h>
#include <geogram/basic/geometry_nd.h>


namespace aliceVision {

namespace sfmData {
class SfMData;
}

namespace fuseCut {

class GCOutput
{
public:
    using VertexIndex = GEO::index_t;
    using CellIndex = GEO::index_t;

    struct Facet
    {
        CellIndex cellIndex = GEO::NO_CELL;
        /// local opposite vertex index
        VertexIndex localVertexIndex = GEO::NO_VERTEX;

        Facet() {}
        Facet(CellIndex ci, VertexIndex lvi)
          : cellIndex(ci),
            localVertexIndex(lvi)
        {}

        bool operator==(const Facet& f) const { return cellIndex == f.cellIndex && localVertexIndex == f.localVertexIndex; }
    };

    struct Edge
    {
        VertexIndex v0 = GEO::NO_VERTEX;
        VertexIndex v1 = GEO::NO_VERTEX;

        Edge() = default;
        Edge(VertexIndex v0_, VertexIndex v1_)
          : v0{v0_},
            v1{v1_}
        {}

        bool operator==(const Edge& e) const { return v0 == e.v0 && v1 == e.v1; }
        bool isSameUndirectionalEdge(const Edge& e) const { return (v0 == e.v0 && v1 == e.v1) || (v0 == e.v1 && v1 == e.v0); }
    };

public:
    GCOutput(mvsUtils::MultiViewParams& mp) : _mp(mp)
    {
    }

    void segmentFullOrFree(bool full, StaticVector<int>& out_fullSegsColor, int& out_nsegments);
    void invertFullStatusForSmallLabels();
    void createPtsCams(StaticVector<StaticVector<int>>& out_ptsCams);
    mesh::Mesh* createMesh(int maxNbConnectedHelperPoints);
    int computeIsOnSurface(std::vector<bool>& vertexIsOnSurface) const;
    void filterLargeHelperPoints(std::vector<bool>& out_reliableVertices, const std::vector<bool>& vertexIsOnSurface, int maxSegSize);
    int removeBubbles();
    int removeDust(int minSegSize);
    void leaveLargestFullSegmentOnly();
    void cellsStatusFilteringBySolidAngleRatio(int nbSolidAngleFilteringIterations, double minSolidAngleRatio);

    void graphCutPostProcessing(const Point3d hexah[8], const std::string& folderName);

    std::vector<GC_vertexInfo> _verticesAttr;
    GEO::Delaunay_var _tetrahedralization;
    std::vector<bool> _cellIsFull;
    std::vector<Point3d> _verticesCoords;
    std::vector<int> _camsVertexes;
    std::vector<std::vector<CellIndex>> _neighboringCellsPerVertex;

private:
    inline const std::vector<CellIndex>& getNeighboringCellsByVertexIndex(VertexIndex vi) const { return _neighboringCellsPerVertex.at(vi); }
    inline std::size_t getNbVertices() const { return _verticesAttr.size(); }

    inline bool isInvalidOrInfiniteCell(CellIndex ci) const
    {
        return ci == GEO::NO_CELL || isInfiniteCell(ci);
        // return ci < 0 || ci > getNbVertices();
    }
    
    inline VertexIndex getOppositeVertexIndex(const Facet& f) const { return _tetrahedralization->cell_vertex(f.cellIndex, f.localVertexIndex); }

    inline bool isInfiniteCell(CellIndex ci) const
    {
        return _tetrahedralization->cell_is_infinite(ci);
        // return ci < 0 || ci > getNbVertices();
    }

    inline VertexIndex getVertexIndex(const Facet& f, int i) const
    {
        return _tetrahedralization->cell_vertex(f.cellIndex, ((f.localVertexIndex + i + 1) % 4));
    }

    

    inline Facet mirrorFacet(const Facet& f) const
    {
        const std::array<VertexIndex, 3> facetVertices = {getVertexIndex(f, 0), getVertexIndex(f, 1), getVertexIndex(f, 2)};

        Facet out;
        out.cellIndex = _tetrahedralization->cell_adjacent(f.cellIndex, f.localVertexIndex);
        if (out.cellIndex != GEO::NO_CELL)
        {
            // Search for the vertex in adjacent cell which doesn't exist in input facet.
            for (int k = 0; k < 4; ++k)
            {
                CellIndex out_vi = _tetrahedralization->cell_vertex(out.cellIndex, k);
                if (std::find(facetVertices.begin(), facetVertices.end(), out_vi) == facetVertices.end())
                {
                    out.localVertexIndex = k;
                    return out;
                }
            }
        }
        return out;
    }

    

    mvsUtils::MultiViewParams& _mp;
};


}  // namespace fuseCut
}  // namespace aliceVision
