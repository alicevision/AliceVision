// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "mv_mesh.hpp"
#include <aliceVision/structures/mv_staticVector.hpp>

namespace mesh {

using PointVisibility = staticVector<int>;
using PointsVisibility = staticVector<PointVisibility*>;

/**
 * @brief Retrieve the nearest neighbor vertex in @p refMesh for each vertex in @p mesh.
 * @param[in] refMesh input reference mesh
 * @param[in] mesh input target mesh
 * @param[out] out_nearestVertex index of the nearest vertex in @p refMesh for each vertex in @p mesh
 * @return the nearest vertex in @p refMesh for each vertex in @p mesh
 */
int getNearestVertices(const mv_mesh& refMesh, const mv_mesh& mesh, staticVector<int>& out_nearestVertex);

/**
 * @brief Transfer the visibility per vertex from one mesh to another.
 * For each vertex of the @p mesh, we search the nearest neighbor vertex in the @p refMesh and copy its visibility information.
 * @note The visibility information is a list of camera IDs which are seeing the vertex.
 *
 * @param[in] refMesh input reference mesh
 * @param[in] refPtsVisibilities visibility array per vertex of @p refMesh
 * @param[in] mesh input target mesh
 * @param[out] out_ptsVisibilities visibility array per vertex of @p mesh
 */
void remapMeshVisibilities(
    const mv_mesh& refMesh, const PointsVisibility& refPtsVisibilities,
    const mv_mesh& mesh, PointsVisibility& out_ptsVisibilities);

}
