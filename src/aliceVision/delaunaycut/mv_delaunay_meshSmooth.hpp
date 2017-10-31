#pragma once

#include <aliceVision/structures/mv_multiview_params.hpp>
#include <aliceVision/structures/mv_staticVector.hpp>
#include <aliceVision/mesh/mv_mesh.hpp>

class point3d;
class mv_delaunay_GC;
class mv_prematch_cams;

void meshPostProcessing(mv_mesh*& inout_mesh, staticVector<staticVector<int>*>*& inout_ptsCams, staticVector<int>& usedCams,
                      multiviewParams& mp, mv_prematch_cams& pc,
                      const std::string& resultFolderName,
                      staticVector<point3d>* hexahsToExcludeFromResultingMesh, point3d* hexah);
