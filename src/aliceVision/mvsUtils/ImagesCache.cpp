// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImagesCache.hpp"
#include <aliceVision/mvsUtils/common.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>

#include <future>

namespace aliceVision {
namespace mvsUtils {

std::string ECorrectEV_enumToString(const ECorrectEV correctEV)
{
    switch(correctEV)
    {
    case ECorrectEV::NO_CORRECTION:  return "no exposure correction";
    case ECorrectEV::APPLY_CORRECTION:   return "exposure correction";
    default: ;
    }
    throw std::out_of_range("No string defined for ECorrectEV: " + std::to_string(int(correctEV)));
}


template<typename Image>
ImagesCache<Image>::ImagesCache(const MultiViewParams& mp, imageIO::EImageColorSpace colorspace, ECorrectEV correctEV)
  : _mp(mp)
  , _colorspace(colorspace)
  , _correctEV(correctEV)
{
    std::vector<std::string> imagesNames;
    for(int rc = 0; rc < _mp.getNbCameras(); rc++)
    {
        imagesNames.push_back(_mp.getImagePath(rc));
    }
    initIC( imagesNames );
}

template<typename Image>
ImagesCache<Image>::ImagesCache(const MultiViewParams& mp, imageIO::EImageColorSpace colorspace, std::vector<std::string>& imagesNames
                        , ECorrectEV correctEV)
  : _mp(mp)
  , _colorspace(colorspace)
  , _correctEV(correctEV)
{
    initIC( imagesNames );
}

template<typename Image>
void ImagesCache<Image>::initIC( std::vector<std::string>& imagesNames )
{
    float oneimagemb = (sizeof(Color) * _mp.getMaxImageWidth() * _mp.getMaxImageHeight()) / 1024.f / 1024.f;
    float maxmbCPU = (float)_mp.userParams.get<int>("images_cache.maxmbCPU", 5000);
    int npreload = std::max((int)(maxmbCPU / oneimagemb), 5); // image cache has a minimum size of 5
    npreload = std::min(_mp.ncams, npreload);

    for(int rc = 0; rc < _mp.ncams; rc++)
    {
        _imagesNames.push_back(imagesNames[rc]);
    }

    _camIdMapId.resize( _mp.ncams, -1 );
    setCacheSize(npreload);

    {
        // Cannot resize the vector<mutex> directly, as mutex class is not move-constructible.
        // imagesMutexes.resize(mp->ncams); // cannot compile
        // So, we do the same with a new vector and swap.
        std::vector<std::mutex> imagesMutexesTmp(_mp.ncams);
        _imagesMutexes.swap(imagesMutexesTmp);
    }


}

template<typename Image>
void ImagesCache<Image>::setCacheSize(int nbPreload)
{
    _N_PRELOADED_IMAGES = nbPreload;
    _imgs.resize(_N_PRELOADED_IMAGES);
    _mapIdCamId.resize( _N_PRELOADED_IMAGES, -1 );
    _mapIdClock.resize( _N_PRELOADED_IMAGES, clock() );
}

template<typename Image>
void ImagesCache<Image>::refreshData(int camId)
{
    // printf("camId %i\n",camId);
    // test if the image is in the memory
    if(_camIdMapId[camId] == -1)
    {
        // remove the oldest one
        int mapId = _mapIdClock.minValId();
        int oldCamId = _mapIdCamId[mapId];
        if(oldCamId>=0)
            _camIdMapId[oldCamId] = -1;
            // TODO: oldCamId should be protected if already used

        // replace with new new
        _camIdMapId[camId] = mapId;
        _mapIdCamId[mapId] = camId;
        _mapIdClock[mapId] = clock();

        // reload data from files
        long t1 = clock();
        if (_imgs[mapId] == nullptr)
        {
            const int maxWidth = _mp.getMaxImageWidth();
            const int maxHeight = _mp.getMaxImageHeight();
            _imgs[mapId] = std::make_shared<Image>(maxWidth, maxHeight);
        }

        const std::string imagePath = _imagesNames.at(camId);
        loadImage(imagePath, _mp, camId, *(_imgs[mapId]), _colorspace, _correctEV);

        ALICEVISION_LOG_DEBUG("Add " << imagePath << " to image cache. " << formatElapsedTime(t1));
    }
    else
    {
      ALICEVISION_LOG_DEBUG("Reuse " << _imagesNames.at(camId) << " from image cache. ");
    }
}

template<typename Image>
void ImagesCache<Image>::refreshImage_sync(int camId)
{
  std::lock_guard<std::mutex> lock(_imagesMutexes[camId]);
  refreshData(camId);
}

template<typename Image>
void ImagesCache<Image>::refreshImage_async(int camId)
{
    _asyncObjects.emplace_back(std::async(std::launch::async, &ImagesCache<Image>::refreshImage_sync, this, camId));
}

template<typename Image>
void ImagesCache<Image>::refreshImages_sync(const std::vector<int>& camIds)
{
    for(int camId: camIds)
        refreshImage_sync(camId);
}

template<typename Image>
void ImagesCache<Image>::refreshImages_async(const std::vector<int>& camIds)
{
    _asyncObjects.emplace_back(std::async(std::launch::async, &ImagesCache<Image>::refreshImages_sync, this, camIds));
}

template class ImagesCache<ImageRGBf>;
template class ImagesCache<ImageRGBAf>;

} // namespace mvsUtils
} // namespace aliceVision
