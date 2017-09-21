// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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
