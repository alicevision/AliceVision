#pragma once

#include "structures/mv_multiview_params.h"
#include "structures/mv_staticVector.h"
#include "mesh/mv_mesh.h"

class point3d;
class mv_delaunay_GC;
class mv_prematch_cams;

void meshPostProcessing(mv_mesh*& inout_mesh, staticVector<staticVector<int>*>*& inout_ptsCams, staticVector<int>& usedCams,
                      multiviewParams& mp, mv_prematch_cams& pc,
                      const std::string& resultFolderName,
                      staticVector<point3d>* hexahsToExcludeFromResultingMesh, point3d* hexah);
