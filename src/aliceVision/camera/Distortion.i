// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <std_shared_ptr.i>
%shared_ptr(aliceVision::camera::Distortion)
%shared_ptr(aliceVision::camera::DistortionBrown)
%shared_ptr(aliceVision::camera::DistortionFisheye)
%shared_ptr(aliceVision::camera::DistortionFisheye1)
%shared_ptr(aliceVision::camera::DistortionRadialK1)
%shared_ptr(aliceVision::camera::DistortionRadialK3)
%shared_ptr(aliceVision::camera::DistortionRadialK3PT)

%include <aliceVision/camera/Distortion.hpp>

%{
#include <aliceVision/camera/Distortion.hpp>
%}
