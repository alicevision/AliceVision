// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "Tetrahedralization.hpp"
#include <aliceVision/mvsData/geometry.hpp>
#include <aliceVision/fuseCut/Intersections.hpp>
#include <aliceVision/fuseCut/delaunayGraphCutTypes.hpp>

namespace aliceVision {
namespace fuseCut {

class GraphCut
{
public:
    void maxflow();

public:
    double getFaceWeight(const Facet& f1) const;
    Point3d cellCircumScribedSphereCentre(CellIndex ci) const;
    inline double conj(double val) const { return val; }

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

    inline VertexIndex getVertexIndex(const Facet& f, int i) const
    {
        return _tetrahedralization.cell_vertex(f.cellIndex, ((f.localVertexIndex + i + 1) % 4));
    }

public:
    std::vector<bool> _cellIsFull;
    std::vector<GC_cellInfo> _cellsAttr;
    Tetrahedralization _tetrahedralization;
    std::vector<Point3d> _verticesCoords;
    std::vector<GC_vertexInfo> _verticesAttr;
};

}
}
