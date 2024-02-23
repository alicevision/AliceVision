// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Intersections.hpp"
#include <aliceVision/mvsData/geometry.hpp>

namespace aliceVision {
namespace fuseCut {

TetrahedronsRayMarching::TetrahedronsRayMarching(const Tetrahedralization & tetra,
                                                const VertexIndex & originId,
                                                const VertexIndex & destinationId,
                                                const bool away)
    :
    _tetrahedralization(tetra),
    _epsilonFactor(1e-4),
    _intersection(originId),
    _facetCount(0),
    _vertexCount(0),
    _edgeCount(0)
{
    const auto & vertices = tetra.getVertices();

    const auto & ptOrigin = vertices[originId];
    _origin.x() = ptOrigin.x;
    _origin.y() = ptOrigin.y;
    _origin.z() = ptOrigin.z;

    _intersectionPoint = _origin;
    _previousIntersectionPoint = _origin;

    const auto & ptDest = vertices[destinationId];
    Eigen::Vector3d dest;
    dest.x() = ptDest.x;
    dest.y() = ptDest.y;
    dest.z() = ptDest.z;

    _direction = (dest - _origin).normalized();
    if (away)
    {
        _direction = -_direction;
    }
}

GeometryIntersection TetrahedronsRayMarching::intersectNextGeom()
{
    

    _previousIntersection = _intersection;
    _previousIntersectionPoint = _intersectionPoint;

    switch (_previousIntersection.type)
    {
    case EGeometryType::Facet:
        _intersection = intersectNextGeomFacet();
        _facetCount++;
        break;
    case EGeometryType::Edge:
        _intersection = intersectNextGeomEdge();
        _edgeCount++;
        break;
    case EGeometryType::Vertex:
        _intersection = intersectNextGeomVertex();
        _vertexCount++;
        break;
    default:
        return GeometryIntersection();
    }

    if (_facetCount > 10000 && _previousIntersection.type == EGeometryType::Facet)
    {
        return GeometryIntersection();
    }

    if (_vertexCount > 1000 && _previousIntersection.type == EGeometryType::Vertex)
    {
        return GeometryIntersection();
    }

    if (_edgeCount > 1000 && _previousIntersection.type == EGeometryType::Edge)
    {
        return GeometryIntersection();
    }

    //Make sure we are moving in the right direction !
    if ((_intersectionPoint - _origin).norm() <= (_previousIntersectionPoint - _origin).norm())
    {
        return GeometryIntersection();
    }

    //We intersected a facet.
    //We directly move to the opposite side
    if (_intersection.type == EGeometryType::Facet)
    {
        Facet f = _tetrahedralization.mirrorFacet(_intersection.facet);
        if (_tetrahedralization.isInvalidOrInfiniteCell(f.cellIndex))
        {
            return GeometryIntersection();
        }

        _previousIntersection = _intersection;
        _intersection = GeometryIntersection(f);
    }

    return _intersection;
}

GeometryIntersection TetrahedronsRayMarching::intersectNextGeomFacet()
{
    GeometryIntersection bestMatch;
    Eigen::Vector3d bestMatchIntersectPt;

    
    const CellIndex tetrahedronIndex = _intersection.facet.cellIndex;

    // Test all facets of the tetrahedron using i as localVertexIndex to define next intersectionFacet
    for (int i = 0; i < 4; ++i)
    {
        // Because we can't intersect with incoming facet (same localVertexIndex)
        if (i == _intersection.facet.localVertexIndex)
        {
            continue;
        }

        const Facet intersectionFacet(tetrahedronIndex, i);
        bool ambiguous = false;

        const GeometryIntersection result = rayIntersectTriangle(intersectionFacet, _previousIntersectionPoint, _intersectionPoint, ambiguous);
        if (result.type != EGeometryType::None)
        {
            if (!ambiguous)
            {
                return result;
            }

            // Retrieve the best intersected point (farthest from origin point)
            if (bestMatch.type == EGeometryType::None || (_origin - _intersectionPoint).size() > (_origin - bestMatchIntersectPt).size())
            {
                bestMatchIntersectPt = _intersectionPoint;
                bestMatch = result;
            }
        }
    }
        
    _intersectionPoint = bestMatchIntersectPt;
    return bestMatch;
}

GeometryIntersection TetrahedronsRayMarching::intersectNextGeomEdge()
{
    GeometryIntersection bestMatch;
    Eigen::Vector3d bestMatchIntersectPt;

   
    GeometryIntersection result;

    for (CellIndex adjCellIndex : _tetrahedralization.getNeighboringCellsByEdge(_intersection.edge))
    {
        if (_tetrahedralization.isInvalidOrInfiniteCell(adjCellIndex))
            continue;
        // Local vertices indices
        const VertexIndex lvi0 = _tetrahedralization.index(adjCellIndex, _intersection.edge.v0);
        const VertexIndex lvi1 = _tetrahedralization.index(adjCellIndex, _intersection.edge.v1);

        // The two facets that do not touch this edge need to be tested
        const std::array<Facet, 2> opositeFacets{{{adjCellIndex, lvi0}, {adjCellIndex, lvi1}}};

        for (const Facet& facet : opositeFacets)
        {
            bool ambiguous = false;
            const GeometryIntersection result = rayIntersectTriangle(facet, _previousIntersectionPoint, _intersectionPoint, ambiguous);

            if (result.type == EGeometryType::Edge)
            {
                if (result.edge.isSameUndirectionalEdge(_intersection.edge))
                {
                    continue;
                }
            }
            if (result.type != EGeometryType::None)
            {
                if (!ambiguous)
                {
                    return result;
                }

                // Retrieve the best intersected point (farthest from origin point)
                if (bestMatch.type == EGeometryType::None || (_origin - _intersectionPoint).size() > (_origin - bestMatchIntersectPt).size())
                {
                    bestMatchIntersectPt = _intersectionPoint;
                    bestMatch = result;
                }
            }
        }
    }
        
    _intersectionPoint = bestMatchIntersectPt;
    return bestMatch;
}

GeometryIntersection TetrahedronsRayMarching::intersectNextGeomVertex()
{
    GeometryIntersection bestMatch;
    Eigen::Vector3d bestMatchIntersectPt;


   for (CellIndex adjCellIndex : _tetrahedralization.getNeighboringCellsByVertexIndex(_intersection.vertexIndex))
    {
        if (_tetrahedralization.isInvalidOrInfiniteCell(adjCellIndex))
            continue;

        // Get local vertex index
        const VertexIndex localVertexIndex = _tetrahedralization.index(adjCellIndex, _intersection.vertexIndex);

        // Define the facet to intersect
        const Facet facet(adjCellIndex, localVertexIndex);
        bool ambiguous = false;

        const GeometryIntersection result = rayIntersectTriangle(facet, _previousIntersectionPoint, _intersectionPoint, ambiguous);

        if (result.type != EGeometryType::None)
        {
            if (!ambiguous)
            {
                return result;
            }

            // Retrieve the best intersected point (farthest from origin point)
            if (bestMatch.type == EGeometryType::None || (_origin - _intersectionPoint).size() > (_origin - bestMatchIntersectPt).size())
            {
                bestMatchIntersectPt = _intersectionPoint;
                bestMatch = result;
            }
        }
    }

    _intersectionPoint = bestMatchIntersectPt;
    return bestMatch;
}

Eigen::Vector2d TetrahedronsRayMarching::getLineTriangleIntersectBarycCoords(Eigen::Vector3d & P,
                                                                            const Eigen::Vector3d & A, 
                                                                            const Eigen::Vector3d & B, 
                                                                            const Eigen::Vector3d & C) const
{
    const double A_x = A.x();
    const double A_y = A.y();
    const double A_z = A.z();
    const double linePoint_x = _origin.x();
    const double linePoint_y = _origin.y();
    const double linePoint_z = _origin.z();
    const double lineVect_x = _direction.x();
    const double lineVect_y = _direction.y();
    const double lineVect_z = _direction.z();
    const double v0_x = C.x() - A_x;
    const double v0_y = C.y() - A_y;
    const double v0_z = C.z() - A_z;
    const double v1_x = B.x() - A_x;
    const double v1_y = B.y() - A_y;
    const double v1_z = B.z() - A_z;
    const double _n_x = v0_y * v1_z - v0_z * v1_y;
    const double _n_y = v0_z * v1_x - v0_x * v1_z;
    const double _n_z = v0_x * v1_y - v0_y * v1_x;
    const double k = ((A_x * _n_x + A_y * _n_y + A_z * _n_z) - (_n_x * linePoint_x + _n_y * linePoint_y + _n_z * linePoint_z)) / (_n_x * lineVect_x + _n_y * lineVect_y + _n_z * lineVect_z);
    const double P_x = linePoint_x + lineVect_x * k;
    const double P_y = linePoint_y + lineVect_y * k;
    const double P_z = linePoint_z + lineVect_z * k;

    // Compute vectors
    const double v2_x = P_x - A_x;
    const double v2_y = P_y - A_y;
    const double v2_z = P_z - A_z;

    // Compute dot products
    const double dot00 = (v0_x * v0_x + v0_y * v0_y + v0_z * v0_z);
    const double dot01 = (v0_x * v1_x + v0_y * v1_y + v0_z * v1_z);
    const double dot02 = (v0_x * v2_x + v0_y * v2_y + v0_z * v2_z);
    const double dot11 = (v1_x * v1_x + v1_y * v1_y + v1_z * v1_z);
    const double dot12 = (v1_x * v2_x + v1_y * v2_y + v1_z * v2_z);

    // Compute barycentric coordinates
    const double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
    const double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    const double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    P.x() = P_x;
    P.y() = P_y;
    P.z() = P_z;

    Eigen::Vector2d ret;
    ret.x() = u;
    ret.y() = v;

    return ret;
}

GeometryIntersection TetrahedronsRayMarching::rayIntersectTriangle(
                                                        const Facet& facet,
                                                        const Eigen::Vector3d & lastIntersectionPoint,
                                                        Eigen::Vector3d & intersectPt,
                                                        bool& ambiguous) const
{
    ambiguous = false;

    const VertexIndex AvertexIndex = _tetrahedralization.cell_vertex(facet.cellIndex, facet.getIndex(0));
    const VertexIndex BvertexIndex = _tetrahedralization.cell_vertex(facet.cellIndex, facet.getIndex(1));
    const VertexIndex CvertexIndex = _tetrahedralization.cell_vertex(facet.cellIndex, facet.getIndex(2));

    const auto & vertices = _tetrahedralization.getVertices();

    const Point3d & A = vertices[AvertexIndex];
    const Point3d & B = vertices[BvertexIndex];
    const Point3d & C = vertices[CvertexIndex];

    Eigen::Vector3d eA;
    eA.x() = A.x;
    eA.y() = A.y;
    eA.z() = A.z;

    Eigen::Vector3d eB;
    eB.x() = B.x;
    eB.y() = B.y;
    eB.z() = B.z;

    Eigen::Vector3d eC;
    eC.x() = C.x;
    eC.y() = C.y;
    eC.z() = C.z;

    const double ABSize = (A - B).size();
    const double BCSize = (B - C).size();
    const double ACSize = (A - C).size();

    const double marginEpsilon = std::min({ABSize, BCSize, ACSize}) * _epsilonFactor;
    const double ambiguityEpsilon = (ABSize + BCSize + ACSize) / 3.0 * 1.0e-2;

    

    Eigen::Vector3d tempIntersectPt;
    const Eigen::Vector2d triangleUv = getLineTriangleIntersectBarycCoords(tempIntersectPt, eA, eB, eC);

    if (!std::isnormal(tempIntersectPt.x()) 
        || !std::isnormal(tempIntersectPt.y()) 
        || !std::isnormal(tempIntersectPt.z()))
    {
        return GeometryIntersection();
    }

    const double u = triangleUv.x();  // A to C
    const double v = triangleUv.y();  // A to B

    // If we find invalid uv coordinate
    if (!std::isfinite(u) || !std::isfinite(v))
    {
        return GeometryIntersection();
    }

    // Ouside the triangle with marginEpsilon margin
    if (u < -marginEpsilon || v < -marginEpsilon || (u + v) > (1.0 + marginEpsilon))
    {
        return GeometryIntersection();
    }

    // In case intersectPt is provided, check if intersectPt is in front of lastIntersectionPt
    // in the DirVec direction to ensure that we are moving forward in the right direction
    const Eigen::Vector3d diff = tempIntersectPt - lastIntersectionPoint;
    const double dotValue = _direction.dot(diff.normalized());
    
    if (dotValue < marginEpsilon || diff.size() < 100 * std::numeric_limits<double>::min())
    {
        return GeometryIntersection();
    }

    if (diff.size() < ambiguityEpsilon)
    {
        ambiguous = true;
    }

    // Change intersection point only if tempIntersectionPt is in the right direction (mean we intersect something)
    intersectPt = tempIntersectPt;

    if (v < marginEpsilon)  // along A C edge
    {
        if (u < marginEpsilon)
        {
            intersectPt = eA;
            return GeometryIntersection(AvertexIndex);  // vertex A
        }
        if (u > 1.0 - marginEpsilon)
        {
            intersectPt = eC;
            return GeometryIntersection(CvertexIndex);  // vertex C
        }

        return GeometryIntersection(Edge(AvertexIndex, CvertexIndex));  // edge AC
    }

    if (u < marginEpsilon)  // along A B edge
    {
        if (v > 1.0 - marginEpsilon)
        {
            intersectPt = eB;
            return GeometryIntersection(BvertexIndex);  // vertex B
        }

        return GeometryIntersection(Edge(AvertexIndex, BvertexIndex));  // edge AB
    }

    if (u + v > 1.0 - marginEpsilon)
    {
        return GeometryIntersection(Edge(BvertexIndex, CvertexIndex));  // edge BC
    }

    return GeometryIntersection(facet);
}

std::ostream& operator<<(std::ostream& stream, const EGeometryType type)
{
    switch (type)
    {
        case EGeometryType::Facet:
            stream << "Facet";
            break;
        case EGeometryType::Vertex:
            stream << "Vertex";
            break;
        case EGeometryType::Edge:
            stream << "Edge";
            break;
        case EGeometryType::None:
            stream << "None";
            break;
    }
    return stream;
}

std::ostream& operator<<(std::ostream& stream, const GeometryIntersection& intersection)
{
    stream << intersection.type << ": ";
    switch (intersection.type)
    {
        case EGeometryType::Facet:
            stream << intersection.facet;
            break;
        case EGeometryType::Vertex:
            stream << intersection.vertexIndex;
            break;
        case EGeometryType::Edge:
            stream << intersection.edge;
            break;
        case EGeometryType::None:
            stream << "None";
            break;
    }
    return stream;
}

}
}
