// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TileParams.hpp"

namespace aliceVision {
namespace mvsUtils {

void getTileList(const TileParams& tileParams, int imageWidth, int imageHeight, std::vector<ROI>& out_tileList)
{
    // compute nb of tiles per side
    const int nbTileSideX = 1 + int(std::ceil(float(imageWidth  - tileParams.width)  / float(tileParams.width  - tileParams.padding)));
    const int nbTileSideY = 1 + int(std::ceil(float(imageHeight - tileParams.height) / float(tileParams.height - tileParams.padding)));

    out_tileList.resize(nbTileSideX * nbTileSideY);

    // compute each tile 2d region of interest
    for(int i = 0; i < nbTileSideX; ++i)
    {
        const int startX = i * (tileParams.width - tileParams.padding);
        const int endX = std::min(startX + tileParams.width, imageWidth);

        for(int j = 0; j < nbTileSideY; ++j)
        {
            const int startY = j * (tileParams.height - tileParams.padding);
            const int endY = std::min(startY + tileParams.height, imageHeight);

            out_tileList.at(i * nbTileSideY + j) = ROI(startX, endX, startY, endY);
        }
    }
}

} // namespace mvsUtils
} // namespace aliceVision
