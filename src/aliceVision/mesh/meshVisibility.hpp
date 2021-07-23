// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mesh/Mesh.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>

namespace aliceVision {
namespace mesh {

/**
 * @brief Retrieve the nearest neighbor vertex in @p refMesh for each vertex in @p mesh.
 * @param[in] refMesh input reference mesh
 * @param[in] mesh input target mesh
 * @param[out] out_nearestVertex index of the nearest vertex in @p refMesh for each vertex in @p mesh
 * @return the nearest vertex in @p refMesh for each vertex in @p mesh
 */
void getNearestVertices(const Mesh& refMesh, const Mesh& mesh, StaticVector<int>& out_nearestVertex);

/**
 * @brief Transfer the visibility per vertex from one mesh to another.
 * For each vertex of the @p mesh, we search the nearest neighbor vertex in the @p refMesh and copy its visibility information.
 * @note The visibility information is a list of camera IDs seeing the vertex.
 *
 * @param[in] refMesh input reference mesh
 * @param[in] mesh input target mesh
 */
void remapMeshVisibilities_pullVerticesVisibility(const Mesh& refMesh, Mesh &mesh);

/**
* @brief Transfer the visibility per vertex from one mesh to another.
* For each vertex of the @p refMesh, we search the closest triangle in the @p mesh and copy its visibility information to each vertex of the triangle.
* @note The visibility information is a list of camera IDs seeing the vertex.
*
* @param[in] refMesh input reference mesh
* @param[in] mesh input target mesh
*/
void remapMeshVisibilities_pushVerticesVisibilityToTriangles(const Mesh& refMesh, Mesh& mesh);

void remapMeshVisibilities_meshItself(const mvsUtils::MultiViewParams& mp, Mesh& mesh);

} // namespace mesh
} // namespace aliceVision
