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

Tetrahedralization::Tetrahedralization()
{
}

void Tetrahedralization::buildFromVertices(const std::vector<Point3d> & vertices)
{
    //Use geogram to build tetrahedrons
    GEO::initialize();
    GEO::Delaunay_var tetrahedralization = GEO::Delaunay::create(3, "BDEL");
    tetrahedralization->set_stores_neighbors(true);
    tetrahedralization->set_vertices(vertices.size(), vertices.front().m);

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

    updateVertexToCellsCache(vertices.size());
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

void Tetrahedralization::buildFromTetrahedralization(const Tetrahedralization & other, const std::vector<Point3d> & vertices,  const Node & node)
{
    _mesh.clear();
    _neighboringCellsPerVertex.clear();

    std::map<CellIndex, CellIndex> globalToLocal;
    std::vector<bool> verticesToKeep(vertices.size(), false);

    //Keep only vertices which are inside the bounds
    for (int idVertex = 0; idVertex < vertices.size(); idVertex++)
    {
        const auto & p = vertices[idVertex];

        Eigen::Vector3d ep;
        ep.x() = p.x;
        ep.y() = p.y;
        ep.z() = p.z;

        if (node.isInside(ep))
        {
            verticesToKeep[idVertex] = true;
        }
    }
    
    //Keep only cells with at least one vertice valid
    for (int idCell = 0; idCell < other._mesh.size(); idCell++)
    {
        const auto & c = other._mesh[idCell];

        bool valid = false;
        for (int i = 0; i < 4; i++)
        {   
            VertexIndex v = c.indices[i];

            if (v != GEO::NO_VERTEX)
            {
                if (!verticesToKeep[v])
                {
                    valid = true;
                    break;
                }
            }
        }

        globalToLocal[idCell] = (valid)?_mesh.size():GEO::NO_CELL;
        _mesh.push_back(c); 
    }


    //Update adjacency
    for (auto & c : _mesh)
    {
        for (int i = 0; i < 4; i++)
        {
            CellIndex & adjacent = c.adjacent[i];
            if (adjacent == GEO::NO_CELL)
            {
                continue;
            }
            
            adjacent = globalToLocal[adjacent];
        }
    }

    for (const auto & p : globalToLocal)
    {
        if (p.second != GEO::NO_CELL)
        {
            _localToGlobal[p.second] = p.first;
        }
    }

    updateVertexToCellsCache(vertices.size());
}

bool Tetrahedralization::locate(const std::vector<Point3d> & vertices, const Eigen::Vector3d & p, CellIndex & foundCell)
{
    std::mt19937 gen;
    std::uniform_int_distribution<std::size_t> distMesh(0, _mesh.size() - 1);
    std::uniform_int_distribution<std::size_t> dist4(0, 3);

    CellIndex previousCell = GEO::NO_CELL;
    CellIndex currentCell = CellIndex(distMesh(gen));

    std::array<Eigen::Vector3d, 4> points;
    int orientations[] = {-1, -1, -1, -1};

    int iter = 0;
    bool found = false;
    int count = 0;

    while (!found)
    {
        count++;
        
        const Cell & c = _mesh[currentCell];

        for (int i = 0; i < 4; i++)
        {
            const Point3d & pt = vertices[c.indices[i]];
            
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
            if (adjacent == GEO::NO_CELL)
            {
                return false;
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
        
        //We are not able to move ...
        if (!hasChange)
        {
            break;
        }

        foundCell = currentCell;
        if (count > 10000)
        {
            ALICEVISION_LOG_ERROR("something went wrong");
            return false;
        }
    }

    return true;
}

bool Tetrahedralization::isInside(const CellIndex & ci, const std::vector<Point3d> & vertices, const Eigen::Vector3d & point)
{
    std::array<Eigen::Vector3d, 4> pts;

    for (int i = 0; i < 4; i++)
    {
        Point3d pt = vertices[_mesh[ci].indices[0]];
        pts[i].x() = pt.x;
        pts[i].y() = pt.y;
        pts[i].z() = pt.z;
    }

    std::array<Eigen::Vector3d, 4> test;

    test = pts;
    test[0] = point;
    if (orient3d(test) < 0) return false;

    test = pts;
    test[1] = point;
    if (orient3d(test) < 0) return false;

    test = pts;
    test[2] = point;
    if (orient3d(test) < 0) return false;

    test = pts;
    test[3] = point;
    if (orient3d(test) < 0) return false;

    return true;
}

}
}