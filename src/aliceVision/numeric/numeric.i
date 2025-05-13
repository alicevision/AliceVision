// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="pyalicevision") numeric

%include <aliceVision/global.i>
%include <aliceVision/numeric/numeric.hpp>

%include <aliceVision/numeric/eigen.i>
%eigen_typemaps(Vec2)

double getX(const Vec2 & vec);
double getY(const Vec2 & vec);

%{
#include <aliceVision/numeric/numeric.hpp>
using namespace aliceVision;

double getX(const Vec2 & vec)
{
    return vec(0);
}

double getY(const Vec2 & vec)
{
    return vec(1);
}

%}

