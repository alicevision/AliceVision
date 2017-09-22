// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

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


