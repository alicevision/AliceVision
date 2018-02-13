// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Rgb.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/mesh/rply.h>
#include <aliceVision/mesh/Mesh.hpp>

int savePLY(std::string plyFileName, Mesh* me, bool diffuse = false);
int savePLY(std::string plyFileName, Mesh* me, StaticVector<rgb>* triColors, bool diffuse = false);

StaticVector<rgb>* getTrisColorsRgb(Mesh* me, StaticVector<rgb>* ptsColors);
StaticVector<rgb>* getPtsColorsRgb(Mesh* me, StaticVector<rgb>* triColors);
