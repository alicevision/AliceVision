// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "TileParams.hpp"

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/numeric/numeric.hpp>

namespace aliceVision {
namespace mvsUtils {

void getTileRoiList(const TileParams& tileParams, int imageWidth, int imageHeight, int maxDownscale, std::vector<ROI>& out_tileRoiList)
{
    assert(maxDownscale > 0);
    assert(2 * tileParams.padding < tileParams.bufferWidth);
    assert(2 * tileParams.padding < tileParams.bufferHeight);

    // single tile case
    if (hasOnlyOneTile(tileParams, imageWidth, imageHeight))
    {
        out_tileRoiList.emplace_back(0, imageWidth, 0, imageHeight);
        return;
    }

    // compute maximum effective tile width and height: maximum size without padding
    const int maxEffectiveTileWidth = tileParams.bufferWidth - 2 * tileParams.padding;
    const int maxEffectiveTileHeight = tileParams.bufferHeight - 2 * tileParams.padding;

    // compute nb of tile buffers per side
    const int nbTileSideX = divideRoundUp(imageWidth, maxEffectiveTileWidth);
    const int nbTileSideY = divideRoundUp(imageHeight, maxEffectiveTileHeight);

    // allocate roi list
    out_tileRoiList.resize(nbTileSideX * nbTileSideY);

    // compute downscaled image width and height
    const int downscaledImageWidth = divideRoundUp(imageWidth, maxDownscale);
    const int downscaledImageHeight = divideRoundUp(imageHeight, maxDownscale);

    // compute effective tile width and height for the best tile layout at the maximum downscale
    const int effectiveTileWidth = divideRoundUp(downscaledImageWidth, nbTileSideX) * maxDownscale;
    const int effectiveTileHeight = divideRoundUp(downscaledImageHeight, nbTileSideY) * maxDownscale;

    // compute each tile ROI
    for (int i = 0; i < nbTileSideX; ++i)
    {
        const int beginX = i * effectiveTileWidth;
        const int endX = std::min((i + 1) * effectiveTileWidth + tileParams.padding, imageWidth);

        for (int j = 0; j < nbTileSideY; ++j)
        {
            const int beginY = j * effectiveTileHeight;
            const int endY = std::min((j + 1) * effectiveTileHeight + tileParams.padding, imageHeight);

            out_tileRoiList.at(i * nbTileSideY + j) = ROI(beginX, endX, beginY, endY);
        }
    }
}

void logTileRoiList(const TileParams& tileParams, int imageWidth, int imageHeight, int maxDownscale, const std::vector<ROI>& in_tileRoiList)
{
    // compute maximum effective tile width and height: maximum size without padding
    const int maxEffectiveTileWidth = tileParams.bufferWidth - 2 * tileParams.padding;
    const int maxEffectiveTileHeight = tileParams.bufferHeight - 2 * tileParams.padding;

    // compute nb of tile buffers per side
    const int nbTileSideX = divideRoundUp(imageWidth, maxEffectiveTileWidth);
    const int nbTileSideY = divideRoundUp(imageHeight, maxEffectiveTileHeight);

    // compute downscaled image width and height
    const int downscaledImageWidth = divideRoundUp(imageWidth, maxDownscale);
    const int downscaledImageHeight = divideRoundUp(imageHeight, maxDownscale);

    // compute effective tile width and height for the best tile layout at the maximum downscale
    const int effectiveTileWidth = divideRoundUp(downscaledImageWidth, nbTileSideX) * maxDownscale;
    const int effectiveTileHeight = divideRoundUp(downscaledImageHeight, nbTileSideY) * maxDownscale;

    std::ostringstream ostr;
    ostr << "Tiling information: " << std::endl
         << "\t- parameters: " << std::endl
         << "\t      - buffer width:  " << tileParams.bufferWidth << " px" << std::endl
         << "\t      - buffer height: " << tileParams.bufferHeight << " px" << std::endl
         << "\t      - padding: " << tileParams.padding << " px" << std::endl
         << "\t- maximum downscale:  " << maxDownscale << std::endl
         << "\t- maximum image width:  " << imageWidth << " px" << std::endl
         << "\t- maximum image height: " << imageHeight << " px" << std::endl;

    if (hasOnlyOneTile(tileParams, imageWidth, imageHeight))
    {
        ALICEVISION_LOG_INFO(ostr.str());
        ALICEVISION_LOG_INFO("Maximum image size is smaller than one tile, use only one tile.");
        return;
    }

    ostr << "\t- maximum effective tile width:  " << maxEffectiveTileWidth << " px" << std::endl
         << "\t- maximum effective tile height: " << maxEffectiveTileHeight << " px" << std::endl
         << "\t- # tiles on X-side: " << nbTileSideX << std::endl
         << "\t- # tiles on Y-side: " << nbTileSideY << std::endl
         << "\t- effective tile width:  " << effectiveTileWidth << " px" << std::endl
         << "\t- effective tile height: " << effectiveTileHeight << " px" << std::endl
         << "\t- tile list: " << std::endl;

    if (in_tileRoiList.empty())
        ostr << "\t   empty" << std::endl;

    for (size_t i = 0; i < in_tileRoiList.size(); ++i)
    {
        const ROI& roi = in_tileRoiList.at(i);

        ostr << "\t   - tile (" << (i + 1) << "/" << in_tileRoiList.size() << ") "
             << "size: " << roi.width() << "x" << roi.height() << " px, roi: [" << roi << "]" << std::endl;
    }

    ALICEVISION_LOG_INFO(ostr.str());
}

}  // namespace mvsUtils
}  // namespace aliceVision
