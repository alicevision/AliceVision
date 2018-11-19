// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/mvsData/Color.hpp>
#include <aliceVision/mvsData/Point2d.hpp>
#include <aliceVision/mvsData/Rgb.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>
#include <aliceVision/mvsUtils/MultiViewParams.hpp>

#include <future>
#include <mutex>

namespace aliceVision {
namespace mvsUtils {

class ImagesCache
{
public:
    const MultiViewParams* mp;

    ImagesCache(const ImagesCache&) = delete;

    int N_PRELOADED_IMAGES;
    Color** imgs;

    StaticVector<int>* camIdMapId;
    StaticVector<int>* mapIdCamId;
    StaticVector<long>* mapIdClock;
    std::vector<std::mutex> imagesMutexes;
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
    std::future<void> refreshData_async(int camId);

    Color getPixelValueInterpolated(const Point2d* pix, int camId);
    rgb getPixelValue(const Pixel& pix, int camId);
};

} // namespace mvsUtils
} // namespace aliceVision
