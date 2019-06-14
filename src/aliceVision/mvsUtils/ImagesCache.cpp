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

ImagesCache::ImagesCache(const MultiViewParams* _mp, int _bandType, imageIO::EImageColorSpace colorspace)
  : mp(_mp)
  , bandType( _bandType )
  , _colorspace(colorspace)
{
    std::vector<std::string> _imagesNames;
    for(int rc = 0; rc < _mp->getNbCameras(); rc++)
    {
        _imagesNames.push_back(_mp->getImagePath(rc));
    }
    initIC( _imagesNames );
}

ImagesCache::ImagesCache(const MultiViewParams* _mp, int _bandType, imageIO::EImageColorSpace colorspace, std::vector<std::string>& _imagesNames)
  : mp(_mp)
  , bandType( _bandType )
  , _colorspace(colorspace)
{
    initIC( _imagesNames );
}

void ImagesCache::initIC( std::vector<std::string>& _imagesNames )
{
    float oneimagemb = (sizeof(Color) * mp->getMaxImageWidth() * mp->getMaxImageHeight()) / 1024.f / 1024.f;
    float maxmbCPU = (float)mp->userParams.get<int>("images_cache.maxmbCPU", 5000);
    int _npreload = std::max((int)(maxmbCPU / oneimagemb), 5); // image cache has a minimum size of 5
    _npreload = std::min(mp->ncams, _npreload);

    for(int rc = 0; rc < mp->ncams; rc++)
    {
        imagesNames.push_back(_imagesNames[rc]);
    }

    camIdMapId.resize( mp->ncams, -1 );
    setCacheSize(_npreload);

    {
        // Cannot resize the vector<mutex> directly, as mutex class is not move-constructible.
        // imagesMutexes.resize(mp->ncams); // cannot compile
        // So, we do the same with a new vector and swap.
        std::vector<std::mutex> imagesMutexesTmp(mp->ncams);
        imagesMutexes.swap(imagesMutexesTmp);
    }


}

void ImagesCache::setCacheSize(int nbPreload)
{
    N_PRELOADED_IMAGES = nbPreload;
    imgs.resize(N_PRELOADED_IMAGES);
    mapIdCamId.resize( N_PRELOADED_IMAGES, -1 );
    mapIdClock.resize( N_PRELOADED_IMAGES, clock() );
}

void ImagesCache::refreshData(int camId)
{
    // printf("camId %i\n",camId);
    // test if the image is in the memory
    if(camIdMapId[camId] == -1)
    {
        // remove the oldest one
        int mapId = mapIdClock.minValId();
        int oldCamId = mapIdCamId[mapId];
        if(oldCamId>=0)
            camIdMapId[oldCamId] = -1;
            // TODO: oldCamId should be protected if already used

        // replace with new new
        camIdMapId[camId] = mapId;
        mapIdCamId[mapId] = camId;
        mapIdClock[mapId] = clock();

        // reload data from files
        long t1 = clock();
        if (imgs[mapId] == nullptr)
        {
            const int maxWidth = mp->getMaxImageWidth();
            const int maxHeight = mp->getMaxImageHeight();
            imgs[mapId] = std::make_shared<Image>(maxWidth, maxHeight);
        }

        const std::string imagePath = imagesNames.at(camId);
        loadImage(imagePath, mp, camId, *(imgs[mapId]),  bandType, _colorspace);

        ALICEVISION_LOG_DEBUG("Add " << imagePath << " to image cache. " << formatElapsedTime(t1));
    }
    else
    {
      ALICEVISION_LOG_DEBUG("Reuse " << imagesNames.at(camId) << " from image cache. ");
    }
}
void ImagesCache::refreshData_sync(int camId)
{
  std::lock_guard<std::mutex> lock(imagesMutexes[camId]);
  refreshData(camId);
}

std::future<void> ImagesCache::refreshData_async(int camId)
{
    return std::async(&ImagesCache::refreshData_sync, this, camId);
}

Color ImagesCache::getPixelValueInterpolated(const Point2d* pix, int camId)
{
    // get the image index in the memory
    const int i = camIdMapId[camId];
    const ImgSharedPtr& img = imgs[i];
    
    const int xp = static_cast<int>(pix->x);
    const int yp = static_cast<int>(pix->y);

    // precision to 4 decimal places
    const float ui = pix->x - static_cast<float>(xp);
    const float vi = pix->y - static_cast<float>(yp);

    const Color lu = img->at( xp  , yp   );
    const Color ru = img->at( xp+1, yp   );
    const Color rd = img->at( xp+1, yp+1 );
    const Color ld = img->at( xp  , yp+1 );

    // bilinear interpolation of the pixel intensity value
    const Color u = lu + (ru - lu) * ui;
    const Color d = ld + (rd - ld) * ui;
    const Color out = u + (d - u) * vi;

    return out;
}


} // namespace mvsUtils
} // namespace aliceVision
