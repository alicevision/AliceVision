// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <std_shared_ptr.i>
// EInitMode is used by IntrinsicBase: wrap it in every module that includes this file (e.g. sfmData),
// otherwise getInitializationMode() returns an opaque pointer.
%include <aliceVision/camera/IntrinsicInitMode.i>

// The whole intrinsic hierarchy is declared here, before any intrinsic class or factory function is parsed.
%shared_ptr(aliceVision::camera::IntrinsicBase)
%shared_ptr(aliceVision::camera::IntrinsicScaleOffset)
%shared_ptr(aliceVision::camera::IntrinsicScaleOffsetDisto)
%shared_ptr(aliceVision::camera::Pinhole)
%shared_ptr(aliceVision::camera::Equidistant)
%shared_ptr(aliceVision::camera::Equirectangular)

%{
#include <aliceVision/camera/IntrinsicBase.hpp>
%}

%include <aliceVision/camera/IntrinsicBase.hpp>
