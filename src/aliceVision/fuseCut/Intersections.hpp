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

    GeometryIntersection rayIntersectTriangle(const Facet& facet,
                                            const Eigen::Vector3d & lastIntersectPt,
                                            Eigen::Vector3d & intersectPt,
                                            bool& ambiguous) const;    

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
