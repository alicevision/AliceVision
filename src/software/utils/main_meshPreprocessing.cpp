// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <boost/program_options.hpp>


#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <unordered_map>
#include <unordered_set>
#include <cstdint>
#include <stack>
#include <queue>
#include <limits>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;

using GridCoord = Eigen::Vector<unsigned, 3>;

struct GridCoordHash
{
    std::size_t operator()(const GridCoord& coord) const noexcept
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



class OctreeNode
{
public:
    using uptr = std::unique_ptr<OctreeNode>;

public:

    /**
     * @brief Returns the mesh vertices stored in this node.
     * @return Read-only reference to the vertex list.
     */
    const std::vector<Vec3f>& getVertices() const
    {
        return _vertices;
    }

    /**
     * @brief Returns a mutable reference to the mesh vertices stored in this node.
     * @return Mutable reference to the vertex list.
     */
    std::vector<Vec3f>& getVertices()
    {
        return _vertices;
    }

    /**
     * @brief Sets the mesh vertices for this node.
     * @param vertices New vertex list.
     */
    void setVertices(const std::vector<Vec3f>& vertices)
    {
        _vertices = vertices;
    }

    /**
     * @brief Returns the per-vertex normals stored in this node.
     * @return Read-only reference to the normal list.
     */
    const std::vector<Vec3f>& getNormals() const
    {
        return _normals;
    }

    /**
     * @brief Returns a mutable reference to the per-vertex normals stored in this node.
     * @return Mutable reference to the normal list.
     */
    std::vector<Vec3f>& getNormals()
    {
        return _normals;
    }

    /**
     * @brief Sets the per-vertex normals for this node.
     * @param normals New normal list.
     */
    void setNormals(const std::vector<Vec3f>& normals)
    {
        _normals = normals;
    }

    /**
     * @brief Returns the triangular faces (index triplets) stored in this node.
     * @return Read-only reference to the face list.
     */
    const std::vector<Eigen::Vector<unsigned, 3>>& getFaces() const
    {
        return _faces;
    }

    /**
     * @brief Returns a mutable reference to the triangular faces stored in this node.
     * @return Mutable reference to the face list.
     */
    std::vector<Eigen::Vector<unsigned, 3>>& getFaces()
    {
        return _faces;
    }

    /**
     * @brief Sets the triangular faces for this node.
     * @param faces New face list.
     */
    void setFaces(const std::vector<Eigen::Vector<unsigned, 3>>& faces)
    {
        _faces = faces;
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

    bool isLeaf() const 
    {
        return (!_children[0]);
    }

    bool hasFaces() const
    {
        return !_faces.empty();
    }

    /**
     * @brief Computes the axis-aligned bounding box from @c _vertices
     *        and stores the result in @c _bMin and @c _bMax.
     *
     * If the vertex list is empty, both bounds are set to zero.
     */
    void computeBounds()
    {
        if (_vertices.empty())
        {
            _bMin = Vec3f::Zero();
            _bMax = Vec3f::Zero();
            return;
        }

        _bMin = _vertices[0];
        _bMax = _vertices[0];
        for (const Vec3f& v : _vertices)
        {
            _bMin = _bMin.cwiseMin(v);
            _bMax = _bMax.cwiseMax(v);
        }
    }

    void buildDown(size_t maxTriangles, size_t maxLevel = std::numeric_limits<size_t>::max())
    {
        if (_depth == maxLevel)
        {
            return;
        }

        if (_vertices.size() < maxTriangles)
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
            child->buildDown(maxTriangles, maxLevel);
        }
    }

    bool subdivide()
    {
        if (!isLeaf())
        {
            return false;
        }

        if (_faces.empty())
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
        for (unsigned f = 0; f < _faces.size(); ++f)
        {
            const Eigen::Vector<unsigned, 3> & face = _faces[f];

            // Assign face to the octant of its centroid
            const Vec3f centroid = (_vertices[face[0]] + _vertices[face[1]] + _vertices[face[2]]) / 3.0f;

            // Encore octant to int
            const int octant = ((centroid[0] >= center[0]) ? 1 : 0) | ((centroid[1] >= center[1]) ? 2 : 0) | ((centroid[2] >= center[2]) ? 4 : 0);

            OctreeNode& child = *_children[octant];
            Eigen::Vector<unsigned, 3> localFace;

            for (int k = 0; k < 3; ++k)
            {
                const unsigned globalIdx = face[k];
                const unsigned nextLocalIdx = static_cast<unsigned>(child.getVertices().size());

                auto [it, inserted] = indexRemap[octant].emplace(globalIdx, nextLocalIdx);

                if (inserted)
                {
                    const Vec3f& vertex = _vertices[globalIdx];
                    const Vec3f& normal = _normals[globalIdx];
                    child.getVertices().push_back(vertex);
                    child.getNormals().push_back(normal);
                }

                // May be a previous value if inserted is false;
                const unsigned localIdx = it->second;
                localFace[k] = localIdx;
            }

            child.getFaces().push_back(localFace);
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
        _vertices.clear();
        _normals.clear();
        _faces.clear();

        return true;
    }


    uint32_t getRemainingDepth()
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

    std::vector<OctreeNode*> getLeaves()
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

    void balance()
    {
        while (balanceOnce())
        {
        }
    }

private:
    bool balanceOnce()
    {
        std::vector<OctreeNode*> leaves = getLeaves();
        const uint32_t maxDepth = getRemainingDepth();

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
    }

    static unsigned getSpanAtDepth(uint32_t targetDepth, uint32_t depth)
    {
        return 1u << (targetDepth - depth);
    }

    GridCoord getScaledPosition(uint32_t targetDepth) const
    {
        return _position * getSpanAtDepth(targetDepth, _depth);
    }

    std::vector<Vec3f> _vertices;
    std::vector<Vec3f> _normals;
    std::vector<Eigen::Vector<unsigned, 3>> _faces;

    std::array<uptr, 8> _children;

    Vec3f _bMin = Vec3f::Zero();
    Vec3f _bMax = Vec3f::Zero();
    Eigen::Vector<unsigned, 3> _position = Eigen::Vector<unsigned, 3>::Zero();
    
    uint32_t _depth = 0;
};

bool importScene(const std::string & path, std::vector<Vec3f> & vertices, std::vector<Vec3f> & normals, std::vector<Eigen::Vector<unsigned, 3>> & faces)
{
    vertices.clear();
    normals.clear();
    faces.clear();

    Assimp::Importer importer;

    const aiScene *scene = importer.ReadFile(
        path,
        aiProcess_Triangulate           |  // triangles only
        aiProcess_GenSmoothNormals      |  // generate normals if missing
        aiProcess_JoinIdenticalVertices |  // deduplicate vertices
        aiProcess_FlipUVs               |  // Qt / Vulkan UV convention
        aiProcess_PreTransformVertices  |  // bake node transforms
        aiProcess_ValidateDataStructure
    );

    if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode) 
    {
        ALICEVISION_LOG_ERROR("Assimp : failed to load : " << importer.GetErrorString());
        return false;
    }

    if (scene->mNumMeshes != 1) 
    {
        ALICEVISION_LOG_ERROR("Assimp : This application only supports 1 mesh per file");
        return false;
    }

    const aiMesh *mesh = scene->mMeshes[0];
    
    if (!mesh || mesh->mNumVertices == 0 || mesh->mNumFaces == 0)
    {
        ALICEVISION_LOG_ERROR("Invalid mesh.");
        return false;
    }

    vertices.resize(mesh->mNumVertices);
    normals.resize(mesh->mNumVertices);
    faces.resize(mesh->mNumFaces);
    
    // Vertices
    for (unsigned int v = 0; v < mesh->mNumVertices; ++v) {
        
        Vec3f & vertex = vertices[v];
        Vec3f & normal = normals[v];

        vertex[0] = mesh->mVertices[v].x;
        vertex[1] = mesh->mVertices[v].y;
        vertex[2] = mesh->mVertices[v].z;
        normal[0] = mesh->mNormals[v].x;
        normal[1] = mesh->mNormals[v].y;
        normal[2] = mesh->mNormals[v].z;
    }

    // Indices
    for (unsigned int f = 0; f < mesh->mNumFaces; ++f) 
    {
        const aiFace &face = mesh->mFaces[f];
        // aiProcess_Triangulate guarantees 3 indices per face

        Eigen::Vector<unsigned, 3> & oface = faces[f];

        oface[0] = face.mIndices[0];
        oface[1] = face.mIndices[1];
        oface[2] = face.mIndices[2];
    }

    return true;
}

int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string inputFilename;
    std::string outputFilename;

    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&inputFilename)->required(), "Input mesh.")
        ("output,o", po::value<std::string>(&outputFilename)->required(), "Output mesh.");


    CmdLine cmdline("Preprocessing mesh for visualization.\n"
                    "AliceVision meshPreprocessing");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    OctreeNode octree;

    ALICEVISION_LOG_INFO("Importing mesh.");
    if (!importScene(inputFilename, octree.getVertices(), octree.getNormals(), octree.getFaces()))
    {
        ALICEVISION_LOG_ERROR("Failed loading scene.");
        return EXIT_FAILURE;
    }

    ALICEVISION_LOG_INFO("Computing bounds.");
    octree.computeBounds();
    
    size_t maxTriangles = 100;

    ALICEVISION_LOG_INFO("Splitting.");
    octree.buildDown(maxTriangles);

    ALICEVISION_LOG_INFO("Balancing.");
    octree.balance();

    ALICEVISION_LOG_INFO("Balancing.");
    octree.balance();
    
    ALICEVISION_LOG_INFO("Balancing.");
    octree.balance();

    return EXIT_SUCCESS;
}
