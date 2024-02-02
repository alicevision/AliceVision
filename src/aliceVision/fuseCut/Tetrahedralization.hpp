// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <geogram/basic/numeric.h>
#include <geogram/mesh/mesh.h>
#include <aliceVision/mvsData/Point3d.hpp>

namespace aliceVision {
namespace fuseCut {

using VertexIndex = GEO::index_t;
using CellIndex = GEO::index_t;

class Tetrahedralization
{
public:
    struct Cell
    {
        VertexIndex indices[4];
        CellIndex adjacent[4];
    };

public:
    Tetrahedralization();

    void buildFromVertices(const std::vector<Point3d> & vertices);

    //Find local index  for given global index 
    VertexIndex index(CellIndex ci, VertexIndex vi) const
    {
        const Cell & c = _mesh[ci];
        for (VertexIndex cvi = 0; cvi < 4; cvi++)
        {
            if (c.indices[cvi] == vi)
            {
                return cvi;
            }
        }

        return GEO::NO_VERTEX;
    }

    VertexIndex cell_vertex(CellIndex ci, VertexIndex vi) const
    {
        return _mesh[ci].indices[vi];
    }

    CellIndex cell_adjacent(CellIndex ci, VertexIndex vi) const
    {
        return _mesh[ci].adjacent[vi];
    }

    inline bool isInfiniteCell(CellIndex ci) const
    {
        for (int i = 0; i < 4; i++)
        {
            if (_mesh[ci].indices[i] == -1)
            {
                return true;
            }
        }
        
        return false;
    }

    inline bool isInvalidOrInfiniteCell(CellIndex ci) const
    {
        return ci == GEO::NO_CELL || isInfiniteCell(ci);
    }

    size_t nb_cells() const
    {
        return _mesh.size();
    }

    const std::vector<std::vector<CellIndex>> & getNeighboringCellsPerVertex() const
    {
        return _neighboringCellsPerVertex;
    }

private:
    void updateVertexToCellsCache(const std::vector<Point3d> & vertices);

private:
    std::vector<Cell> _mesh;
    std::vector<std::vector<CellIndex>> _neighboringCellsPerVertex;
};

}
}