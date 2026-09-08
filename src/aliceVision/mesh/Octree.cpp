// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/mesh/Octree.hpp>

namespace aliceVision {
namespace mesh {

struct GridCoordHash
{
    std::size_t operator()(const Vec3u & coord) const noexcept
    {
        std::size_t seed = 0;

        for (int axis = 0; axis < 3; ++axis)
        {
            const std::size_t value = std::hash<unsigned>{}(coord[axis]);
            seed ^= value + 0x9e3779b9u + (seed << 6) + (seed >> 2);
        }

        return seed;
    }
};

struct GridCoordEqual
{
    bool operator()(const GridCoord& lhs, const GridCoord& rhs) const noexcept
    {
        return lhs[0] == rhs[0] && lhs[1] == rhs[1] && lhs[2] == rhs[2];
    }
};

template<typename TValue>
using GridCoordMap = std::unordered_map<GridCoord, TValue, GridCoordHash, GridCoordEqual>;

uint32_t OctreeNode::getLargestAbsoluteDepth()
{
    uint32_t maxDepth = 0;

    // Collect all the leafs
    std::stack<OctreeNode*> stack;
    stack.push(this);

    while (!stack.empty())
    {
        OctreeNode * cur = stack.top();
        stack.pop();

        if (!cur->_children[0])
        {
            // Update maxDepth
            maxDepth = std::max(maxDepth, cur->_depth);
        }
        else 
        {
            for (auto & child : cur->_children)
            {
                stack.push(child.get());
            }
        }
    }

    return maxDepth;
}

void OctreeNode::computeBounds()
{
    if (_rawData.positions.empty())
    {
        _bMin = Vec3f::Zero();
        _bMax = Vec3f::Zero();
        return;
    }

    _bMin = _rawData.positions[0];
    _bMax = _rawData.positions[0];
    for (const Vec3f& v : _rawData.positions)
    {
        _bMin = _bMin.cwiseMin(v);
        _bMax = _bMax.cwiseMax(v);
    }
}

void OctreeNode::buildDown(size_t maxTriangles, double minSize, size_t maxLevel)
{
    if (_depth == maxLevel)
    {
        return;
    }

    if (_rawData.positions.size() < maxTriangles)
    {
        return;
    }

    if ((_bMax - _bMin).minCoeff() < minSize)
    {
        return;
    }

    if (!subdivide())
    {
        return;
    }

    //Build children recursively
    if (maxLevel == _depth)
    {
        return;
    }
        
    for (const auto & child : _children)
    {
        child->buildDown(maxTriangles, minSize, maxLevel);
    }
}

bool OctreeNode::subdivide()
{
    if (!isLeaf())
    {
        return false;
    }

    if (!hasFaces())
    {
        return false;
    }

    const Vec3f center = (_bMin + _bMax) * 0.5f;

    // Allocate the 8 children
    for (int i = 0; i < 8; ++i)
    {
        _children[i] = std::make_unique<OctreeNode>();
        _children[i]->setDepth(_depth + 1);
    }

    // Per-child remapping: global vertex index -> local index within child
    std::array<std::unordered_map<unsigned, unsigned>, 8> indexRemap;


    // Loop over all faces and move them to the 
    // Correct children according to the centroid position
    for (unsigned f = 0; f < _rawData.getFacesCount(); ++f)
    {
        const unsigned * const face = &_rawData.indices[f * 3];

        // Assign face to the octant of its centroid
        const Vec3f centroid = (_rawData.positions[face[0]] + _rawData.positions[face[1]] + _rawData.positions[face[2]]) / 3.0f;

        // Encore octant to int
        const int octant = ((centroid[0] >= center[0]) ? 1 : 0) | ((centroid[1] >= center[1]) ? 2 : 0) | ((centroid[2] >= center[2]) ? 4 : 0);

        OctreeNode& child = *_children[octant];
        unsigned localFace[3];

        for (int k = 0; k < 3; ++k)
        {
            const unsigned globalIdx = face[k];
            const unsigned nextLocalIdx = static_cast<unsigned>(child.getVertices().size());

            auto [it, inserted] = indexRemap[octant].emplace(globalIdx, nextLocalIdx);

            if (inserted)
            {
                const Vec3f& vertex = _rawData.positions[globalIdx];
                const Vec3f& normal = _rawData.normals[globalIdx];
                child.getVertices().push_back(vertex);
                child.getNormals().push_back(normal);
            }

            // May be a previous value if inserted is false;
            const unsigned localIdx = it->second;
            localFace[k] = localIdx;
        }

        child.getIndices().push_back(localFace[0]);
        child.getIndices().push_back(localFace[1]);
        child.getIndices().push_back(localFace[2]);
    }

    // Set each child's bounding box from the parent bounds and center.
    // Bit 0 = x, bit 1 = y, bit 2 = z: 0 → [bMin, center], 1 → [center, bMax].
    for (int octant = 0; octant < 8; ++octant)
    {
        Vec3f childMin, childMax;
        for (int axis = 0; axis < 3; ++axis)
        {
            const bool upper = (octant >> axis) & 1;
            childMin[axis] = upper ? center[axis] : _bMin[axis];
            childMax[axis] = upper ? _bMax[axis]  : center[axis];
            _children[octant]->_position[axis] = _position[axis] * 2 + ((upper)? 1 : 0);
        }

        _children[octant]->setBMin(childMin);
        _children[octant]->setBMax(childMax);
    }

    // Don't keep data to avoid redundancy
    _rawData.clear();

    return true;
}

std::vector<OctreeNode*> OctreeNode::getLeaves()
{
    std::vector<OctreeNode*> leaves;

    std::stack<OctreeNode*> stack;
    stack.push(this);

    while (!stack.empty())
    {
        OctreeNode * cur = stack.top();
        stack.pop();

        if (cur->isLeaf())
        {
            leaves.push_back(cur);
        }
        else 
        {
            for (auto & child : cur->_children)
            {
                stack.push(child.get());
            }
        }
    }

    return leaves;
}

std::vector<const OctreeNode*> OctreeNode::getLeaves() const
{
    std::vector<const OctreeNode*> leaves;

    std::stack<const OctreeNode*> stack;
    stack.push(this);

    while (!stack.empty())
    {
        const OctreeNode* cur = stack.top();
        stack.pop();

        if (cur->isLeaf())
        {
            leaves.push_back(cur);
        }
        else
        {
            for (const auto& child : cur->_children)
            {
                stack.push(child.get());
            }
        }
    }

    return leaves;
}

}
}