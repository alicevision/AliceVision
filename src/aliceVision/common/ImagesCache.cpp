// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImagesCache.hpp"
#include <aliceVision/common/common.hpp>
#include <aliceVision/common/fileIO.hpp>

int mv_images_cache::getPixelId(int x, int y, int imgid)
{
    if(!transposed)
        return x * mp->mip->getHeight(imgid) + y;
    return y * mp->mip->getWidth(imgid) + x;
}

mv_images_cache::mv_images_cache(const multiviewParams* _mp, int _bandType, bool _transposed)
  : mp(_mp)
{
    std::vector<std::string> _imagesNames;
    for(int rc = 0; rc < _mp->ncams; rc++)
    {
        _imagesNames.push_back(mv_getFileNamePrefix(_mp->mip, rc + 1) + "." + _mp->mip->imageExt);
    }
    initIC(_bandType, _imagesNames, _transposed);
}

mv_images_cache::mv_images_cache(const multiviewParams* _mp, int _bandType, std::vector<std::string>& _imagesNames,
                                 bool _transposed)
  : mp(_mp)
{
    initIC(_bandType, _imagesNames, _transposed);
}

void mv_images_cache::initIC(int _bandType, std::vector<std::string>& _imagesNames,
                             bool _transposed)
{
    float oneimagemb = 3.f * (((float)(mp->mip->getMaxImageWidth() * mp->mip->getMaxImageHeight()) / 1024.f) / 1024.f);
    float maxmbCPU = (float)mp->mip->_ini.get<int>("images_cache.maxmbCPU", 5000);
    int _npreload = std::max((int)(maxmbCPU / oneimagemb), mp->mip->_ini.get<int>("grow.minNumOfConsistentCams", 10));
    N_PRELOADED_IMAGES = std::min(mp->ncams, _npreload);

    transposed = _transposed;
    bandType = _bandType;

    if(_imagesNames.size() != mp->ncams)
    {
        printf("WARNING _imagesNames->size() %zu !=  mp->ncams %i \n", _imagesNames.size(), mp->ncams);
    }

    for(int rc = 0; rc < mp->ncams; rc++)
    {
        imagesNames.push_back(_imagesNames[rc]);
    }

    imgs = new Color*[N_PRELOADED_IMAGES];

    camIdMapId = new staticVector<int>(mp->ncams);
    mapIdCamId = new staticVector<int>(N_PRELOADED_IMAGES);
    mapIdClock = new staticVector<long>(N_PRELOADED_IMAGES);

    for(int i = 0; i < mp->ncams; i++)
    {
        camIdMapId->push_back(-1);
    }
    for (int ni = 0; ni < N_PRELOADED_IMAGES; ni++)
    {
        imgs[ni] = nullptr;

        (*camIdMapId)[ni] = -1;
        mapIdCamId->push_back(-1);
        mapIdClock->push_back(clock());
    }
}

mv_images_cache::~mv_images_cache()
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

void mv_images_cache::refreshData(int camId)
{
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
            size_t maxsize = mp->mip->getMaxImageWidth() * mp->mip->getMaxImageHeight();
            imgs[mapId] = new Color[maxsize];
        }

        std::string imagePath = imagesNames[camId];
        memcpyRGBImageFromFileToArr(camId, imgs[mapId], imagePath, mp->mip, transposed, 1, bandType);

        if(mp->verbose)
        {
            std::string basename = imagePath.substr(imagePath.find_last_of("/\\") + 1);
            printfElapsedTime(t1, "add "+ basename +" to image cache");
        }
    }
}

Color mv_images_cache::getPixelValueInterpolated(const point2d* pix, int camId)
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

rgb mv_images_cache::getPixelValue(const pixel& pix, int camId)
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
