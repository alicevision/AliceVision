// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/numeric/numeric.hpp>
#include <vector>

namespace aliceVision {
namespace mesh {

struct IndexedMeshStreams
{
    std::vector<Vec3f> positions;
    std::vector<Vec3f> normals;
    std::vector<unsigned int> indices;

    /**
     * @brief Returns the number of triangular faces in the mesh.
     *
     * Each face is represented by three consecutive entries in @c indices,
     * so the face count equals <tt>indices.size() / 3</tt>.
     *
     * @return Number of triangles.
     */
    std::size_t getFacesCount() const
    {
        return indices.size() / 3;
    }

    /**
     * @brief Clears all streams, releasing their memory.
     *
     * After this call @c positions, @c normals, and @c indices are all empty.
     */
    void clear()
    {
        positions.clear();
        normals.clear();
        indices.clear();
    }
};

}
}