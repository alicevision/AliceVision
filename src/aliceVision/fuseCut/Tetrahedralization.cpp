// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Tetrahedralization.hpp"

#include <geogram/delaunay/delaunay.h>
#include <geogram/delaunay/delaunay_3d.h>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace fuseCut {

Tetrahedralization::Tetrahedralization(const std::vector<Point3d> & vertices)
: _vertices(vertices)
{
    //Use geogram to build tetrahedrons
    GEO::initialize();
    GEO::Delaunay_var tetrahedralization = GEO::Delaunay::create(3, "BDEL");
    tetrahedralization->set_stores_neighbors(true);
    tetrahedralization->set_vertices(_vertices.size(), _vertices.front().m);

    //Copy information
    _mesh.clear();
    _neighboringCellsPerVertex.clear();
    _mesh.resize(tetrahedralization->nb_cells());
    for (CellIndex ci = 0; ci < tetrahedralization->nb_cells(); ci++)
    {
        Cell & c = _mesh[ci];
        c.indices[0] = tetrahedralization->cell_vertex(ci, 0);
        c.indices[1] = tetrahedralization->cell_vertex(ci, 1);
        c.indices[2] = tetrahedralization->cell_vertex(ci, 2);
        c.indices[3] = tetrahedralization->cell_vertex(ci, 3);

        c.adjacent[0] = tetrahedralization->cell_adjacent(ci, 0);
        c.adjacent[1] = tetrahedralization->cell_adjacent(ci, 1);
        c.adjacent[2] = tetrahedralization->cell_adjacent(ci, 2);
        c.adjacent[3] = tetrahedralization->cell_adjacent(ci, 3);
    }

    //Remove geogram data
    tetrahedralization.reset();

    updateVertexToCellsCache(_vertices.size());
}

Tetrahedralization::Tetrahedralization(const GEO::Delaunay_var & tetrahedralization, const std::vector<Point3d> & vertices)
: _vertices(vertices)
{
    //Copy information
    _mesh.clear();
    _neighboringCellsPerVertex.clear();
    _mesh.resize(tetrahedralization->nb_cells());

    for (CellIndex ci = 0; ci < tetrahedralization->nb_cells(); ci++)
    {
        Cell & c = _mesh[ci];
        c.indices[0] = tetrahedralization->cell_vertex(ci, 0);
        c.indices[1] = tetrahedralization->cell_vertex(ci, 1);
        c.indices[2] = tetrahedralization->cell_vertex(ci, 2);
        c.indices[3] = tetrahedralization->cell_vertex(ci, 3);

        c.adjacent[0] = tetrahedralization->cell_adjacent(ci, 0);
        c.adjacent[1] = tetrahedralization->cell_adjacent(ci, 1);
        c.adjacent[2] = tetrahedralization->cell_adjacent(ci, 2);
        c.adjacent[3] = tetrahedralization->cell_adjacent(ci, 3);
    }

    updateVertexToCellsCache(_vertices.size());
}

void Tetrahedralization::updateVertexToCellsCache(const size_t verticesCount)
{
    _neighboringCellsPerVertex.clear();

    std::map<VertexIndex, std::set<CellIndex>> neighboringCellsPerVertexTmp;

    for (CellIndex ci = 0; ci < nb_cells(); ++ci)
    {
        for (VertexIndex k = 0; k < 4; ++k)
        {
            const VertexIndex vi = cell_vertex(ci, k);

            if (vi == GEO::NO_VERTEX || vi >= verticesCount)
            {
                continue;
            }

            neighboringCellsPerVertexTmp[vi].insert(ci);
        }
    }
    
    _neighboringCellsPerVertex.resize(verticesCount);
    for (const auto& it : neighboringCellsPerVertexTmp)
    {
        const std::set<CellIndex>& input = it.second;
        std::vector<CellIndex>& output = _neighboringCellsPerVertex[it.first];
        output.assign(input.begin(), input.end());
    }
}

void Tetrahedralization::get_neighbors(const VertexIndex & vi, GEO::vector<VertexIndex> & adjVertices) const
{
    adjVertices.clear();

    
    adjVertices.clear();
    if (vi >= _mesh.size())
    {
        return;
    }
    
    //Create unique set of vertices
    std::set<VertexIndex> vertices;
    const auto & cells = _neighboringCellsPerVertex[vi];
    for (const auto & cellId : cells)
    {
        const auto & cell = _mesh[cellId];

        vertices.insert(cell.indices[0]);
        vertices.insert(cell.indices[1]);
        vertices.insert(cell.indices[2]);
        vertices.insert(cell.indices[3]);
    }

    //Copy set to vector
    for (const auto & item : vertices)
    {
        if (item == vi)
        {
            continue;
        }
        
        adjVertices.push_back(item);
    }
}


bool Tetrahedralization::locate(const Eigen::Vector3d & p, CellIndex & foundCell) const
{
    std::mt19937 gen;
    std::uniform_int_distribution<std::size_t> distMesh(0, _mesh.size() - 1);
    std::uniform_int_distribution<std::size_t> dist4(0, 3);

    CellIndex previousCell = GEO::NO_CELL;
    CellIndex currentCell = CellIndex(distMesh(gen));

    std::array<Eigen::Vector3d, 4> points;
    int orientations[] = {-1, -1, -1, -1};

    bool found = false;
    int count = 0;

    while (!found)
    {
        count++;
        
        const Cell & c = _mesh[currentCell];

        //Build cell list of points
        for (int i = 0; i < 4; i++)
        {
            const Point3d & pt = _vertices[c.indices[i]];   
            points[i].x() = pt.x;
            points[i].y() = pt.y;
            points[i].z() = pt.z;
        }

        bool hasChange = false;

        //Loop over cell faces
        std::size_t vstart = dist4(gen);
        for (int idx = 0; idx < 4; idx++)
        {
            const size_t index = (vstart + idx) % 4;

            const CellIndex & adjacent = c.adjacent[index];
            //Do not move to next cell if it's an invalid cell (obviously)
            if (adjacent == GEO::NO_CELL)
            {
                continue;
            }
            
            if (adjacent == previousCell)
            {
                //Previous orientation was negative,
                // we are looping, so this one is positive
                orientations[index] = 1;
                continue;
            }

            Eigen::Vector3d bck = points[index];
            points[index] = p;
            orientations[index] = orient3d(points);
            points[index] = bck;


            //We found a face with negative sign, go to next cell
            if (orientations[index] == -1)
            {
                previousCell = currentCell;
                currentCell = adjacent;
                hasChange = true;
                break;
            }
        }
        
        foundCell = currentCell;
        
        //If there is no change in the previous loop, 
        //It means we have found something
        if (!hasChange)
        {
            break;
        }

        //Too much errand is weird
        if (count > 100000)
        {
            ALICEVISION_LOG_ERROR("something went wrong");
            return false;
        }
    }

    return true;
}

bool Tetrahedralization::isInside(const CellIndex & ci, const Eigen::Vector3d & point) const
{
    std::array<Eigen::Vector3d, 4> pts;

    if (ci < 0 || ci > nb_cells())
    {
        return false;
    }

    const auto & cell = _mesh[ci];

    for (int i = 0; i < 4; i++)
    {
        Point3d pt = _vertices[cell.indices[i]];
        pts[i].x() = pt.x;
        pts[i].y() = pt.y;
        pts[i].z() = pt.z;
    }

    std::array<Eigen::Vector3d, 4> test;

    for (int i = 0; i < 4; i++)
    {
        test = pts;
        test[i] = point;
        if (orient3d(test) < 0) 
        {
            return false;
        }
    }

    return true;
}

}
}