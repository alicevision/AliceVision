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
#include <aliceVision/mvsData/imageIO.hpp>
#include <aliceVision/mvsData/Image.hpp>

#include <future>
#include <mutex>

namespace aliceVision {
namespace mvsUtils {

class ImagesCache
{
public:
    const MultiViewParams* mp;

    typedef std::shared_ptr<Image> ImgSharedPtr;

private:
    ImagesCache(const ImagesCache&) = delete;

    int N_PRELOADED_IMAGES;
    std::vector<ImgSharedPtr> imgs;

    std::vector<int> camIdMapId;
    std::vector<int> mapIdCamId;
    StaticVector<long> mapIdClock;

    std::vector<std::mutex> imagesMutexes;
    std::vector<std::string> imagesNames;

    const int  bandType;
    const imageIO::EImageColorSpace _colorspace;

public:
    ImagesCache( const MultiViewParams* _mp, int _bandType, imageIO::EImageColorSpace colorspace);
    ImagesCache( const MultiViewParams* _mp, int _bandType, imageIO::EImageColorSpace colorspace, std::vector<std::string>& _imagesNames);
    void initIC( std::vector<std::string>& _imagesNames );
    void setCacheSize(int nbPreload);
    ~ImagesCache() = default;

    inline ImgSharedPtr getImg_sync( int camId )
    {
        refreshData_sync(camId);
        const int imageId = camIdMapId[camId];
        return imgs[imageId];
    }

    void refreshData(int camId);
    void refreshData_sync(int camId);

    std::future<void> refreshData_async(int camId);

    Color getPixelValueInterpolated(const Point2d* pix, int camId);
};

} // namespace mvsUtils
} // namespace aliceVision
