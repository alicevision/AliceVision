// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mesh/IndexedMeshStreams.hpp>

#include <memory>

namespace aliceVision {
namespace mesh {

using GridCoord = Vec3u;

class OctreeNode
{
public:
    using uptr = std::unique_ptr<OctreeNode>;
    using ptr = OctreeNode*;

public:

    /**
     * @brief Returns the mesh vertices stored in this node.
     * @return Read-only reference to the vertex list.
     */
    const std::vector<Vec3f>& getVertices() const
    {
        return _rawData.positions;
    }

    /**
     * @brief Returns a mutable reference to the mesh vertices stored in this node.
     * @return Mutable reference to the vertex list.
     */
    std::vector<Vec3f>& getVertices()
    {
        return _rawData.positions;
    }

    /**
     * @brief Sets the mesh vertices for this node.
     * @param vertices New vertex list.
     */
    void setVertices(const std::vector<Vec3f>& vertices)
    {
        _rawData.positions = vertices;
    }

    /**
     * @brief Returns the per-vertex normals stored in this node.
     * @return Read-only reference to the normal list.
     */
    const std::vector<Vec3f>& getNormals() const
    {
        return _rawData.normals;
    }

    /**
     * @brief Returns a mutable reference to the per-vertex normals stored in this node.
     * @return Mutable reference to the normal list.
     */
    std::vector<Vec3f>& getNormals()
    {
        return _rawData.normals;
    }

    /**
     * @brief Sets the per-vertex normals for this node.
     * @param normals New normal list.
     */
    void setNormals(const std::vector<Vec3f>& normals)
    {
        _rawData.normals = normals;
    }

    /**
     * @brief Returns the triangular faces (index triplets) stored in this node.
     * @return Read-only reference to the indices list.
     */
    const std::vector<unsigned>& getIndices() const
    {
        return _rawData.indices;
    }

    /**
     * @brief Returns a mutable reference to the triangular faces stored in this node.
     * @return Mutable reference to the indices list.
     */
    std::vector<unsigned>& getIndices()
    {
        return _rawData.indices;
    }

    /**
     * @brief Sets the triangular faces for this node.
     * @param indices New indices list.
     */
    void setIndices(const std::vector<unsigned>& indices)
    {
        _rawData.indices = indices;
    }

    /**
     * @brief Returns the minimum corner of the node's axis-aligned bounding box.
     * @return Read-only reference to the minimum bound.
     */
    const Vec3f& getBMin() const
    {
        return _bMin;
    }

    /**
     * @brief Sets the minimum corner of the node's axis-aligned bounding box.
     * @param bMin New minimum bound.
     */
    void setBMin(const Vec3f& bMin)
    {
        _bMin = bMin;
    }

    /**
     * @brief Returns the maximum corner of the node's axis-aligned bounding box.
     * @return Read-only reference to the maximum bound.
     */
    const Vec3f& getBMax() const
    {
        return _bMax;
    }

    /**
     * @brief Sets the maximum corner of the node's axis-aligned bounding box.
     * @param bMax New maximum bound.
     */
    void setBMax(const Vec3f& bMax)
    {
        _bMax = bMax;
    }

    /**
     * @brief Returns the depth of this node in the octree (root = 0).
     * @return Depth level of this node.
     */
    uint32_t getDepth() const
    {
        return _depth;
    }

    /**
     * @brief Sets the depth of this node in the octree.
     * @param depth Depth level to assign (root = 0).
     */
    void setDepth(uint32_t depth)
    {
        _depth = depth;
    }

    /**
     * @brief Indicates whether this node has no children.
     * @return True if the node is a leaf node, false otherwise.
     */
    bool isLeaf() const 
    {
        return (!_children[0]);
    }

    /**
     * @brief Indicates whether this node stores any faces.
     * @return True if the node contains at least one face, false otherwise.
     */
    bool hasFaces() const
    {
        return !_rawData.indices.empty();
    }

    const std::array<uptr, 8>& getChildren() const
    {
        return _children;
    }

    /**
     * @brief Walk over the children and find the one with the largest depth.
     * @return the largest absolute depth (not relative to this).
     */
    uint32_t getLargestAbsoluteDepth();

    /**
     * @brief Computes the axis-aligned bounding box from @c _vertices
     *        and stores the result in @c _bMin and @c _bMax.
     *
     * If the vertex list is empty, both bounds are set to zero.
     */
    void computeBounds();
   
    void buildDown(size_t maxTriangles, double minSize, size_t maxLevel = std::numeric_limits<size_t>::max());

    bool subdivide();

    std::vector<OctreeNode*> getLeaves();

    std::vector<const OctreeNode*> getLeaves() const;

    /*void balance()
    {
        while (balanceOnce())
        {
        }
    }*/

private:
    /*bool balanceOnce()
    {
        std::vector<OctreeNode*> leaves = getLeaves();
        const uint32_t maxDepth = getLargestAbsoluteDepth();

        std::vector<GridCoordMap<OctreeNode*>> leavesByDepth(maxDepth + 1);

        for (OctreeNode* leaf : leaves)
        {
            leavesByDepth[leaf->_depth].emplace(leaf->_position, leaf);
        }

        std::unordered_set<OctreeNode*> leavesToSplit;

        // Loop over all leaf nodes
        for (OctreeNode* leaf : leaves)
        {
            const GridCoord leafMin = leaf->getScaledPosition(maxDepth);
            const unsigned leafSpan = getSpanAtDepth(maxDepth, leaf->_depth);
            const GridCoord leafMax = leafMin.array() + leafSpan;

            for (int axis = 0; axis < 3; ++axis)
            {
                //For each axis, get the two other axis
                const int axisA = (axis + 1) % 3;
                const int axisB = (axis + 2) % 3;

                for (int side = 0; side < 2; ++side)
                {
                    for (uint32_t depth = 0; depth + 1 < leaf->_depth; ++depth)
                    {
                        const unsigned neighborSpan = getSpanAtDepth(maxDepth, depth);

                        // A coarser neighbor can only exist if this leaf face lies on
                        // a boundary of the coarser grid cell. If the face is inside a
                        // coarser cell, that depth cannot contain a face-adjacent leaf.
                        if (side == 0)
                        {
                            if (leafMin[axis] == 0 || (leafMin[axis] % neighborSpan) != 0)
                            {
                                continue;
                            }
                        }
                        else
                        {
                            if ((leafMax[axis] % neighborSpan) != 0)
                            {
                                continue;
                            }
                        }

                        // Build the only coarse-grid coordinate that can share this face.
                        GridCoord neighborPos = GridCoord::Zero();
                        neighborPos[axis] = (side == 0) ? (leafMin[axis] / neighborSpan) - 1 : (leafMax[axis] / neighborSpan);
                        neighborPos[axisA] = leafMin[axisA] / neighborSpan;
                        neighborPos[axisB] = leafMin[axisB] / neighborSpan;

                        auto it = leavesByDepth[depth].find(neighborPos);
                        if (it == leavesByDepth[depth].end())
                        {
                            continue;
                        }

                        OctreeNode* neighbor = it->second;

                        if (!neighbor->hasFaces())
                        {
                            continue;
                        }

                        leavesToSplit.insert(neighbor);
                    }
                }
            }
        }

        bool split = false;

        ALICEVISION_LOG_ERROR(leavesToSplit.size());
        for (OctreeNode* leaf : leavesToSplit)
        {
            split = leaf->subdivide() || split;
        }

        return split;
    }*/

    static unsigned getSpanAtDepth(uint32_t targetDepth, uint32_t depth)
    {
        return 1u << (targetDepth - depth);
    }

    GridCoord getScaledPosition(uint32_t targetDepth) const
    {
        return _position * getSpanAtDepth(targetDepth, _depth);
    }

    mesh::IndexedMeshStreams _rawData;
    uint32_t _depth = 0;
    Vec3f _bMin = Vec3f::Zero();
    Vec3f _bMax = Vec3f::Zero();
    GridCoord _position = Vec3u::Zero();

    std::array<uptr, 8> _children;
};

}
}