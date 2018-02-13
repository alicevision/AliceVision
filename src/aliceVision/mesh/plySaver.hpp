// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mesh/rply.h>
#include <aliceVision/mesh/Mesh.hpp>

int savePLY(std::string plyFileName, Mesh* me, bool diffuse = false);
int savePLY(std::string plyFileName, Mesh* me, staticVector<rgb>* triColors, bool diffuse = false);

staticVector<rgb>* getTrisColorsRgb(Mesh* me, staticVector<rgb>* ptsColors);
staticVector<rgb>* getPtsColorsRgb(Mesh* me, staticVector<rgb>* triColors);
