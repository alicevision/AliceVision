// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/structures/Color.hpp>
#include <aliceVision/structures/Point2d.hpp>
#include <aliceVision/structures/Rgb.hpp>
#include <aliceVision/structures/StaticVector.hpp>
#include <aliceVision/common/MultiViewParams.hpp>

namespace aliceVision {

class ImagesCache
{
public:
    const MultiViewParams* mp;

    int N_PRELOADED_IMAGES;
    Color** imgs;

    StaticVector<int>* camIdMapId;
    StaticVector<int>* mapIdCamId;
    StaticVector<long>* mapIdClock;
    std::vector<std::string> imagesNames;

    int bandType;
    bool transposed;

    ImagesCache(const MultiViewParams* _mp, int _bandType, bool _transposed = false);
    ImagesCache(const MultiViewParams* _mp, int _bandType, std::vector<std::string>& _imagesNames,
                    bool _transposed = false);
    void initIC(int _bandType, std::vector<std::string>& _imagesNames, bool _transposed);
    ~ImagesCache();

    int getPixelId(int x, int y, int imgid);
    void refreshData(int camId);
    Color getPixelValueInterpolated(const Point2d* pix, int camId);
    rgb getPixelValue(const Pixel& pix, int camId);
};

} // namespace aliceVision
