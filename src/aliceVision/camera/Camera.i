// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="pyalicevision") camera

%include <aliceVision/global.i>

%{
#include <aliceVision/camera/camera.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/cameraUndistortImage.hpp>
using namespace aliceVision;
using namespace aliceVision::camera;
%}

%include <aliceVision/camera/camera.hpp>
%include <aliceVision/camera/cameraCommon.hpp>
%include <aliceVision/camera/cameraUndistortImage.hpp>

%include <aliceVision/camera/Distortion3DE.i>
%include <aliceVision/camera/DistortionBrown.i>
%include <aliceVision/camera/DistortionFisheye.i>
%include <aliceVision/camera/DistortionFisheye1.i>
%include <aliceVision/camera/DistortionRadial.i>
%include <aliceVision/camera/Undistortion3DE.i>
%include <aliceVision/camera/UndistortionRadial.i>

%include <aliceVision/camera/IntrinsicInitMode.i>
%include <aliceVision/camera/Equidistant.i>
%include <aliceVision/camera/Pinhole.i>
