// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%rename(SO2_expm) aliceVision::SO2::expm;
%rename(SO3_expm) aliceVision::SO3::expm;
%rename(SE3_expm) aliceVision::SE3::expm;


%include <aliceVision/geometry/lie.hpp>

%{
#include <aliceVision/geometry/lie.hpp>
%}