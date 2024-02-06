// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <geogram/basic/numeric.h>
#include <geogram/mesh/mesh.h>
#include <aliceVision/mvsData/Point3d.hpp>
#include <aliceVision/fuseCut/Octree.hpp>

#include <Eigen/Dense>
#include <array>
#include <random>

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

    void buildFromTetrahedralization(const Tetrahedralization & other, const std::vector<Point3d> & vertices, const Node & node);

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

    size_t getGlobalCellIndex(size_t localIndex) const
    {
        if (_localToGlobal.size() == 0)
        {
            return localIndex;
        }

        return _localToGlobal.at(localIndex);
    }

    double orient3d(const std::array<Eigen::Vector3d, 4> & points)
    {
        Eigen::Matrix3d M;
        for (int i = 0; i < 3; i++)
        {
            M(i, 0) = points[i + 1].x() - points[0].x();
            M(i, 1) = points[i + 1].y() - points[0].y();
            M(i, 2) = points[i + 1].z() - points[0].z();
        }

        double delta = M.determinant();
        if (delta < 0.0) return -1.0;
        if (delta > 0.0) return 1.0;
        return 0.0;
    }

    bool isInside(const CellIndex & ci, const std::vector<Point3d> & vertices, const Eigen::Vector3d & point);
    
    bool locate(const std::vector<Point3d> & vertices, const Eigen::Vector3d & p, CellIndex & foundCell);

private:
    void updateVertexToCellsCache(size_t verticesCount);

private:
    std::vector<Cell> _mesh;
    std::vector<std::vector<CellIndex>> _neighboringCellsPerVertex;
    std::map<CellIndex, CellIndex> _localToGlobal;
};

}
}