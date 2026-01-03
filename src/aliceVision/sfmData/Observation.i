// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <aliceVision/numeric/eigen.i>
%eigen_typemaps(Vec2)

//Ignore the non const version, as it create problems
//with eigen wrapper. Force the use of const version
%ignore aliceVision::sfmData::Observation::getCoordinates();

%include <aliceVision/sfmData/Observation.hpp>

%{
#include <aliceVision/sfmData/Observation.hpp>
%}

// TODO: find a way to template flat_map
// %template(Observations) stl::flat_map<uint32_t, aliceVision::sfmData::Observation>;