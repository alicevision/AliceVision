// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <std_shared_ptr.i>
%shared_ptr(aliceVision::camera::Undistortion)
%shared_ptr(aliceVision::camera::UndistortionRadialK3)
%shared_ptr(aliceVision::camera::Undistortion3DEAnamorphic4)
%shared_ptr(aliceVision::camera::Undistortion3DEClassicLD)
%shared_ptr(aliceVision::camera::Undistortion3DERadial4)

%include <aliceVision/camera/Undistortion.hpp>

%{
#include <aliceVision/camera/Undistortion.hpp>
%}
