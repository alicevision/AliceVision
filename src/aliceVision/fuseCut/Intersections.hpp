// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/fuseCut/Tetrahedralization.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision {
namespace fuseCut {

enum class EGeometryType
{
    Vertex,
    Edge,
    Facet,
    None
};

struct Facet
{
    // Associated cell
    CellIndex cellIndex = GEO::NO_CELL;
    
    /// local opposite vertex index
    VertexIndex localVertexIndex = GEO::NO_VERTEX;


    Facet() 
    {

    }

    Facet(CellIndex ci, VertexIndex lvi)
        : cellIndex(ci),
        localVertexIndex(lvi)
    {

    }

    bool operator==(const Facet& f) const 
    { 
        return cellIndex == f.cellIndex && localVertexIndex == f.localVertexIndex; 
    }

    VertexIndex getIndex(int i) const
    {
        return ((localVertexIndex + i + 1) % 4);
    }
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

    bool operator==(const Edge& e) const 
    { 
        return v0 == e.v0 && v1 == e.v1; 
    }

    bool isSameUndirectionalEdge(const Edge& e) const 
    { 
        return (v0 == e.v0 && v1 == e.v1) || (v0 == e.v1 && v1 == e.v0); 
    }
};

struct GeometryIntersection
{
    EGeometryType type = EGeometryType::None;

    union
    {
        Facet facet;
        VertexIndex vertexIndex;
        Edge edge;
    };

    GeometryIntersection() 
    {

    }

    explicit GeometryIntersection(const Facet& f)
        : facet{f},
        type{EGeometryType::Facet}
    {

    }
    
    explicit GeometryIntersection(const VertexIndex& v)
        : vertexIndex{v},
        type{EGeometryType::Vertex}
    {

    }

    explicit GeometryIntersection(const Edge& e)
        : edge{e},
        type{EGeometryType::Edge}
    {

    }

    bool operator==(const GeometryIntersection& g) const
    {
        if (type != g.type)
        {
            return false;
        }

        switch (type)
        {
            case EGeometryType::Vertex:
                return vertexIndex == g.vertexIndex;
            case EGeometryType::Edge:
                return edge == g.edge;
            case EGeometryType::Facet:
                return facet == g.facet;
            case EGeometryType::None:
                break;
        }
        
        return true;
    }

    bool operator!=(const GeometryIntersection& g) const { return !(*this == g); }
};

std::ostream& operator<<(std::ostream& stream, const EGeometryType type);
std::ostream& operator<<(std::ostream& stream, const Facet& facet);
std::ostream& operator<<(std::ostream& stream, const Edge& edge);
std::ostream& operator<<(std::ostream& stream, const GeometryIntersection& intersection);

class TetrahedronsRayMarching
{
public:
    TetrahedronsRayMarching(const Tetrahedralization & tetra,
                            const VertexIndex & originId,
                            const VertexIndex & destinationId,
                            const bool away);
                            
    GeometryIntersection intersectNextGeom();

    void updateIntersection(const GeometryIntersection & intersection)
    {
        _intersection = intersection;
    }

    GeometryIntersection getIntersection() const
    {
        return _intersection;
    }

    GeometryIntersection getPreviousIntersection() const
    {
        return _previousIntersection;
    }

    const Eigen::Vector3d & getIntersectionPoint() const
    {
        return _intersectionPoint;
    }

    const Eigen::Vector3d & getPreviousIntersectionPoint() const
    {
        return _previousIntersectionPoint;
    }

private:
    GeometryIntersection intersectNextGeomFacet();

    GeometryIntersection intersectNextGeomEdge();

    GeometryIntersection intersectNextGeomVertex();

    Eigen::Vector2d getLineTriangleIntersectBarycCoords(Eigen::Vector3d & P, 
                                                        const Eigen::Vector3d & A,  
                                                        const Eigen::Vector3d & B, 
                                                        const Eigen::Vector3d & C) const;

    inline const std::vector<CellIndex>& getNeighboringCellsByVertexIndex(VertexIndex vi) const { return _tetrahedralization.getNeighboringCellsPerVertex().at(vi); }

    GeometryIntersection rayIntersectTriangle(const Facet& facet,
                                            const Eigen::Vector3d & lastIntersectPt,
                                            Eigen::Vector3d & intersectPt,
                                            bool& ambiguous) const;

    std::vector<CellIndex> getNeighboringCellsByEdge(const Edge& e) const
    {
        const std::vector<CellIndex> & v0ci = _tetrahedralization.getNeighboringCellsPerVertex().at(e.v0);
        const std::vector<CellIndex> & v1ci = _tetrahedralization.getNeighboringCellsPerVertex().at(e.v1);

        std::vector<CellIndex> neighboringCells;
        std::set_intersection(v0ci.begin(), v0ci.end(), v1ci.begin(), v1ci.end(), std::back_inserter(neighboringCells));

        return neighboringCells;
    }

    VertexIndex getVertexIndex(const Facet& f, int i) const;
    Facet mirrorFacet(const Facet& f) const;

private:
    const Tetrahedralization & _tetrahedralization;
    const double _epsilonFactor = 1e-4;

    Eigen::Vector3d _origin;
    Eigen::Vector3d _direction;
    GeometryIntersection _intersection;
    GeometryIntersection _previousIntersection;
    Eigen::Vector3d _intersectionPoint;
    Eigen::Vector3d _previousIntersectionPoint;

    size_t _facetCount;
    size_t _vertexCount;
    size_t _edgeCount;
};

}
}
