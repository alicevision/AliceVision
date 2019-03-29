// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsData/Image.hpp>

namespace aliceVision {
namespace mesh {

class MultiBandBlending
{
public:

    ///Generate the pyramid of the differences of the successives gaussian convolutions of the input image
    void laplacianPyramid(std::vector<Image>& out_pyramidL, const Image& inImg, int camId, int nbBand, float sizeKernel);

    ///Generate the gaussian pyramid given the input image
    void laplacianDownscalePyramid(std::vector<Image>& out_pyramidL, const Image& inImg, int nbBand);


};

}
}
