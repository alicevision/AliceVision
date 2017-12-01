// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "rply.h"

#include <aliceVision/mesh/mv_mesh.hpp>
//#include <aliceVision/mesh/mv_mesh_scene_point.hpp>

int mv_saveply(std::string plyFileName, mv_mesh* me, bool diffuse = false);
int mv_saveply(std::string plyFileName, mv_mesh* me, staticVector<rgb>* triColors, bool diffuse = false);
// int mv_saveply(std::string plyFileName, mv_mesh_scene_point *me);

staticVector<rgb>* getTrisColorsRgb(mv_mesh* me, staticVector<rgb>* ptsColors);
staticVector<rgb>* getPtsColorsRgb(mv_mesh* me, staticVector<rgb>* triColors);
