// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MultiBandBlending.hpp"

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/imageIO/image.hpp>



namespace aliceVision {
namespace mesh {

void MultiBandBlending::laplacianPyramid(std::vector<Image>& out_pyramidL, const Image& inImg, int camId, int nbBand, float sizeKernel)
{
    assert(nbBand >= 1);

    const int width = inImg.width();
    const int height = inImg.height();

    Image base(inImg); // gaussian
    Image baseG(width, height); // gaussian
    out_pyramidL.resize(nbBand); // laplacian

    float s = sizeKernel / std::pow(2, nbBand-1);

    //Create Laplacian pyramid
    for(int b = 0; b < nbBand-1; ++b)
    {
        imageIO::convolveImage(width, height, base.data(), baseG.data(), "gaussian", s, s); //baseG = base * gaussianKernel
        Image::imageDiff(base, baseG, out_pyramidL.at(b));
        base.swap(baseG); //newBase = oldBaseG
        s *= 2;

        /*//TODO : REMOVE
        const std::string outPathG = std::string("/datas/rave/tmp/") + std::string("cam") + std::to_string(camId) + std::string("G") + std::to_string(b) +  ".exr";
        imageIO::writeImage(outPathG, width, height, baseG.data());
        const std::string outPathL = std::string("/datas/rave/tmp/") + std::string("cam") + std::to_string(camId) + std::string("L") + std::to_string(b) + ".exr";
        imageIO::writeImage(outPathL, width, height, out_pyramidL.at(b).data());*/

    }
    out_pyramidL.at(nbBand-1) = base;
}


}
}
