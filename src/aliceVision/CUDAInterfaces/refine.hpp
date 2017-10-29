#pragma once

#include <aliceVision/mesh/mv_mesh.hpp>
#include <aliceVision/output3D/mv_output3D.hpp>

void filterPtsCamsByMinimalPixelSize(mv_mesh* me, staticVector<staticVector<int>*>* ptsCams, multiviewParams* mp);
