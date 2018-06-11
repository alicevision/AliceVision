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

int ImagesCache::getPixelId(int x, int y, int imgid)
{
    if(!transposed)
        return x * mp->getHeight(imgid) + y;
    return y * mp->getWidth(imgid) + x;
}

ImagesCache::ImagesCache(const MultiViewParams* _mp, int _bandType, bool _transposed)
  : mp(_mp)
{
    std::vector<std::string> _imagesNames;
    for(int rc = 0; rc < _mp->ncams; rc++)
    {
        _imagesNames.push_back(mv_getFileNamePrefix(_mp->mvDir, _mp, rc) + "." + _mp->getImageExtension());
    }
    initIC(_bandType, _imagesNames, _transposed);
}

ImagesCache::ImagesCache(const MultiViewParams* _mp, int _bandType, std::vector<std::string>& _imagesNames,
                                 bool _transposed)
  : mp(_mp)
{
    initIC(_bandType, _imagesNames, _transposed);
}

void ImagesCache::initIC(int _bandType, std::vector<std::string>& _imagesNames,
                             bool _transposed)
{
    float oneimagemb = (sizeof(Color) * mp->getMaxImageWidth() * mp->getMaxImageHeight()) / 1024.f / 1024.f;
    float maxmbCPU = (float)mp->_ini.get<int>("images_cache.maxmbCPU", 5000);
    int _npreload = std::max((int)(maxmbCPU / oneimagemb), mp->_ini.get<int>("grow.minNumOfConsistentCams", 10));
    N_PRELOADED_IMAGES = std::min(mp->ncams, _npreload);

    transposed = _transposed;
    bandType = _bandType;

    for(int rc = 0; rc < mp->ncams; rc++)
    {
        imagesNames.push_back(_imagesNames[rc]);
    }

    imgs = new Color*[N_PRELOADED_IMAGES];

    camIdMapId = new StaticVector<int>();
    camIdMapId->reserve(mp->ncams);
    mapIdCamId = new StaticVector<int>();
    mapIdCamId->reserve(N_PRELOADED_IMAGES);
    mapIdClock = new StaticVector<long>();
    mapIdClock->reserve(N_PRELOADED_IMAGES);

    for(int i = 0; i < mp->ncams; ++i)
    {
        camIdMapId->push_back(-1);
    }

    {
        // Cannot resize the vector<mutex> directly, as mutex class is not move-constructible.
        // imagesMutexes.resize(mp->ncams); // cannot compile
        // So, we do the same with a new vector and swap.
        std::vector<std::mutex> imagesMutexesTmp(mp->ncams);
        imagesMutexes.swap(imagesMutexesTmp);
    }

    for (int ni = 0; ni < N_PRELOADED_IMAGES; ++ni)
    {
        imgs[ni] = nullptr;

        (*camIdMapId)[ni] = -1;
        mapIdCamId->push_back(-1);
        mapIdClock->push_back(clock());
    }
}

ImagesCache::~ImagesCache()
{
    for(int ni = 0; ni < N_PRELOADED_IMAGES; ni++)
    {
        delete[] imgs[ni];
    }
    delete[] imgs;

    delete camIdMapId;
    delete mapIdCamId;
    delete mapIdClock;
}

void ImagesCache::refreshData(int camId)
{
    std::lock_guard<std::mutex> lock(imagesMutexes[camId]);

    // printf("camId %i\n",camId);
    // test if the image is in the memory
    if((*camIdMapId)[camId] == -1)
    {
        // remove the oldest one
        int mapId = mapIdClock->minValId();
        int oldCamId = (*mapIdCamId)[mapId];
        if(oldCamId>=0)
            (*camIdMapId)[oldCamId] = -1;

        // replace with new new
        (*camIdMapId)[camId] = mapId;
        (*mapIdCamId)[mapId] = camId;
        (*mapIdClock)[mapId] = clock();

        // reload data from files
        long t1 = clock();
        if (imgs[mapId] == nullptr)
        {
            const std::size_t maxSize = mp->getMaxImageWidth() * mp->getMaxImageHeight();
            imgs[mapId] = new Color[maxSize];
        }

        const std::string imagePath = imagesNames.at(camId);
        memcpyRGBImageFromFileToArr(camId, imgs[mapId], imagePath, mp, transposed, bandType);

        ALICEVISION_LOG_DEBUG("Add " << imagePath << " to image cache. " << formatElapsedTime(t1));
    }
}

std::future<void> ImagesCache::refreshData_async(int camId)
{
    return std::async(&ImagesCache::refreshData, this, camId);
}

Color ImagesCache::getPixelValueInterpolated(const Point2d* pix, int camId)
{
    refreshData(camId);

    // get the image index in the memory
    const int i = (*camIdMapId)[camId];
    const Color* img = imgs[i];
    
    const int xp = static_cast<int>(pix->x);
    const int yp = static_cast<int>(pix->y);

    // precision to 4 decimal places
    const float ui = pix->x - static_cast<float>(xp);
    const float vi = pix->y - static_cast<float>(yp);

    const Color lu = img[getPixelId(xp,     yp,     camId)];
    const Color ru = img[getPixelId(xp + 1, yp,     camId)];
    const Color rd = img[getPixelId(xp + 1, yp + 1, camId)];
    const Color ld = img[getPixelId(xp,     yp + 1, camId)];

    // bilinear interpolation of the pixel intensity value
    const Color u = lu + (ru - lu) * ui;
    const Color d = ld + (rd - ld) * ui;
    const Color out = u + (d - u) * vi;

    return out;
}

rgb ImagesCache::getPixelValue(const Pixel& pix, int camId)
{
    refreshData(camId);

    // get the image index in the memory
    const int imageId = (*camIdMapId)[camId];
    const Color* img = imgs[imageId];
    const Color floatRGB = img[getPixelId(pix.x, pix.y, camId)] * 255.0f;

    return rgb(static_cast<unsigned char>(floatRGB.r),
               static_cast<unsigned char>(floatRGB.g),
               static_cast<unsigned char>(floatRGB.b));
}

} // namespace mvsUtils
} // namespace aliceVision
