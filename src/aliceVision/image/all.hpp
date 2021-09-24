// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// Get rid of the specific MSVC compiler warnings.
#if defined(_MSC_VER) && !defined(_CRT_SECURE_NO_WARNINGS)
# define _CRT_SECURE_NO_WARNINGS
#endif

#include "aliceVision/image/Image.hpp"
#include "aliceVision/image/pixelTypes.hpp"
#include "aliceVision/image/convertion.hpp"
#include "aliceVision/image/drawing.hpp"
#include "aliceVision/image/filtering.hpp"
#include "aliceVision/image/resampling.hpp"
#include "aliceVision/image/diffusion.hpp"
#include "aliceVision/image/concat.hpp"
#include "aliceVision/image/io.hpp"
#include "aliceVision/image/convolutionBase.hpp"
#include "aliceVision/image/convolution.hpp"
#include "aliceVision/image/Sampler.hpp"
#include "aliceVision/image/convertionOpenCV.hpp"


