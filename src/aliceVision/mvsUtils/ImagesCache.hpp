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
    class Img
    {
        int  _width;
        int  _height;
    public:
        Img( )
            : data(nullptr)
        { }
        Img( size_t sz )
            : data( new Color[sz] )
        { }

        ~Img( )
        {
            delete [] data;
        }

        inline void setWidth(  int w ) { _width  = w; }
        inline void setHeight( int h ) { _height = h; }

        inline int  getWidth()  const { return _width;  }
        inline int  getHeight() const { return _height; }

        inline Color& at( int x, int y )
        {
            return data[x * _height + y];
        }

        inline const Color& at( int x, int y ) const
        {
            return data[x * _height + y];
        }

        Color* data;
    };

    typedef std::shared_ptr<Img> ImgPtr;

public:
    const MultiViewParams* mp;

private:
    ImagesCache(const ImagesCache&) = delete;

    int N_PRELOADED_IMAGES;
    std::vector<ImgPtr> imgs;

    std::vector<int> camIdMapId;
    std::vector<int> mapIdCamId;
    StaticVector<long> mapIdClock;

    std::vector<std::mutex> imagesMutexes;
    std::vector<std::string> imagesNames;

    const int  bandType;
public:
    const bool transposed;

public:
    ImagesCache( const MultiViewParams* _mp, int _bandType,
                 bool _transposed = false);
    ImagesCache( const MultiViewParams* _mp, int _bandType, std::vector<std::string>& _imagesNames,
                 bool _transposed = false);
    void initIC( std::vector<std::string>& _imagesNames );
    ~ImagesCache();

    inline ImgPtr getImg_sync( int camId )
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
