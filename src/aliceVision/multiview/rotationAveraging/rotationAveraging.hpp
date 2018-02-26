// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

//--
//-- Implementation related to rotation averaging.
// . Compute global rotation from a list of relative estimates.
// - L2 -> See [1]
// - L1 -> See [2]
//
//- [1] "Robust Multiview Reconstruction."
//- Author : Daniel Martinec.
//- Date : July 2, 2008.
//
//- [2] "Efficient and Robust Large-Scale Rotation Averaging"
//- Authors: Avishek Chatterjee and Venu Madhav Govindu
//- Date: December 2013.
//- Conference: ICCV.
//--

#include <aliceVision/multiview/rotationAveraging/common.hpp>
#include <aliceVision/multiview/rotationAveraging/l1.hpp>
#include <aliceVision/multiview/rotationAveraging/l2.hpp>
