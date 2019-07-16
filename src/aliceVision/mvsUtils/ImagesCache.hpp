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

    enum class ECorrectEV
    {
        NO_CORRECTION,
        APPLY_CORRECTION
    };

    std::string ECorrectEV_enumToString(const ECorrectEV correctEV);

    typedef std::shared_ptr<Image> ImgSharedPtr;

private:
    ImagesCache(const ImagesCache&) = delete;

    const MultiViewParams* _mp;

    int _N_PRELOADED_IMAGES;
    std::vector<ImgSharedPtr> _imgs;

    std::vector<int> _camIdMapId;
    std::vector<int> _mapIdCamId;
    StaticVector<long> _mapIdClock;

    std::vector<std::mutex> _imagesMutexes;
    std::vector<std::string> _imagesNames;

    imageIO::EImageColorSpace _colorspace{imageIO::EImageColorSpace::AUTO};
    ECorrectEV _correctEV{ECorrectEV::NO_CORRECTION};

public:
    ImagesCache( const MultiViewParams* mp, imageIO::EImageColorSpace colorspace, ECorrectEV correctEV = ECorrectEV::NO_CORRECTION);
    ImagesCache( const MultiViewParams* mp, imageIO::EImageColorSpace colorspace, std::vector<std::string>& imagesNames, ECorrectEV correctEV = ECorrectEV::NO_CORRECTION);
    void initIC( std::vector<std::string>& imagesNames );
    void setCacheSize(int nbPreload);
    void setCorrectEV(const ECorrectEV correctEV) { _correctEV = correctEV; }
    ~ImagesCache() = default;

    inline ImgSharedPtr getImg_sync( int camId )
    {
        refreshData_sync(camId);
        const int imageId = _camIdMapId[camId];
        return _imgs[imageId];
    }

    void refreshData(int camId);
    void refreshData_sync(int camId);

    std::future<void> refreshData_async(int camId);

    Color getPixelValueInterpolated(const Point2d* pix, int camId);
};

} // namespace mvsUtils
} // namespace aliceVision
