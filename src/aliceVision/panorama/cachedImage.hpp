#pragma once

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/cache.hpp>
#include <aliceVision/panorama/boundingBox.hpp>
#include <aliceVision/system/Logger.hpp>

namespace aliceVision
{

template <class T>
class CachedImage
{
public:
    bool createImage(std::shared_ptr<image::TileCacheManager> manager, size_t width, size_t height)
    {

        _width = width;
        _height = height;
        _tileSize = manager->getTileWidth();

        _tilesArray.clear();

        int countHeight = int(ceil(double(height) / double(manager->getTileHeight())));
        int countWidth = int(ceil(double(width) / double(manager->getTileWidth())));

        _memoryWidth = countWidth * _tileSize;
        _memoryHeight = countHeight * _tileSize;

        for(int i = 0; i < countHeight; i++)
        {

            int tile_height = manager->getTileHeight();
            if(i == countHeight - 1)
            {
                tile_height = height - (i * tile_height);
            }

            std::vector<image::CachedTile::smart_pointer> row;

            for(int j = 0; j < countWidth; j++)
            {

                int tile_width = manager->getTileWidth();
                if(j == countWidth - 1)
                {
                    tile_width = width - (i * tile_width);
                }

                image::CachedTile::smart_pointer tile = manager->requireNewCachedTile<T>(tile_width, tile_height);
                if(tile == nullptr)
                {
                    return false;
                }

                row.push_back(tile);
            }

            _tilesArray.push_back(row);
        }

        return true;
    }

    bool writeImage(const std::string& path)
    {

        ALICEVISION_LOG_ERROR("incorrect template function");
        return false;
    }

    template <class UnaryFunction>
    bool perPixelOperation(UnaryFunction f)
    {

        for(int i = 0; i < _tilesArray.size(); i++)
        {

            std::vector<image::CachedTile::smart_pointer>& row = _tilesArray[i];

            for(int j = 0; j < _tilesArray[i].size(); j++)
            {

                image::CachedTile::smart_pointer ptr = row[j];
                if(!ptr)
                {
                    continue;
                }

                if(!ptr->acquire())
                {
                    continue;
                }

                T* data = (T*)ptr->getDataPointer();

                std::transform(data, data + ptr->getTileWidth() * ptr->getTileHeight(), data, f);
            }
        }

        return true;
    }

    bool assign(const aliceVision::image::Image<T>& input, const BoundingBox & inputBb, const BoundingBox & outputBb)
    {
        BoundingBox outputMemoryBb;
        outputMemoryBb.left = 0;
        outputMemoryBb.top = 0;
        outputMemoryBb.width = _width;
        outputMemoryBb.height = _height;

        if(!outputBb.isInside(outputMemoryBb))
        {
            return false;
        }

        BoundingBox inputMemoryBb;
        inputMemoryBb.left = 0;
        inputMemoryBb.top = 0;
        inputMemoryBb.width = input.Width();
        inputMemoryBb.height = input.Height();

        if(!inputBb.isInside(inputMemoryBb))
        {
            return false;
        }

        BoundingBox snapedBb = outputBb;
        snapedBb.snapToGrid(_tileSize);

        BoundingBox gridBb;
        gridBb.left = snapedBb.left / _tileSize;
        gridBb.top = snapedBb.top / _tileSize;
        gridBb.width = snapedBb.width / _tileSize;
        gridBb.height = snapedBb.height / _tileSize;

        for(int i = 0; i < gridBb.height; i++)
        {
            int ti = gridBb.top + i;
            int cy = ti * _tileSize;
            int sy = inputBb.top + i * _tileSize;

            int start_y = std::max(0, outputBb.top - cy);
            int end_y = std::min(_tileSize - 1, outputBb.getBottom() - cy);

            std::vector<image::CachedTile::smart_pointer>& row = _tilesArray[ti];

            for(int j = 0; j < gridBb.width; j++)
            {
                int tj = gridBb.left + j;
                int cx = tj * _tileSize;
                int sx = inputBb.left + j * _tileSize;

                int start_x = std::max(0, outputBb.left - cx);
                int end_x = std::min(_tileSize - 1, outputBb.getRight() - cx);

                image::CachedTile::smart_pointer ptr = row[tj];
                if(!ptr)
                {
                    continue;
                }

                if(!ptr->acquire())
                {
                    continue;
                }

                T* data = (T*)ptr->getDataPointer();

                for(int y = start_y; y <= end_y; y++)
                {
                    for(int x = start_x; x <= end_x; x++)
                    {
                        data[y * _tileSize + x] = input(sy + y, sx + x);
                    }
                }
            }
        }

        return true;
    }

    bool extract(aliceVision::image::Image<T>& output, const BoundingBox & outputBb, const BoundingBox& inputBb)
    {
        BoundingBox thisBb;
        thisBb.left = 0;
        thisBb.top = 0;
        thisBb.width = _width;
        thisBb.height = _height;

        if(!inputBb.isInside(thisBb))
        {
            return false;
        }

        BoundingBox outputMemoryBb;
        outputMemoryBb.left = 0;
        outputMemoryBb.top = 0;
        outputMemoryBb.width = output.Width();
        outputMemoryBb.height = output.Height();

        if(!outputBb.isInside(outputMemoryBb))
        {
            return false;
        }


        BoundingBox snapedBb = inputBb;
        snapedBb.snapToGrid(_tileSize);

        BoundingBox gridBb;
        gridBb.left = snapedBb.left / _tileSize;
        gridBb.top = snapedBb.top / _tileSize;
        gridBb.width = snapedBb.width / _tileSize;
        gridBb.height = snapedBb.height / _tileSize;

        for(int i = 0; i < gridBb.height; i++)
        {
            int ti = gridBb.top + i;
            int cy = ti * _tileSize;
            int sy = outputBb.top + i * _tileSize;

            int start_y = std::max(0, inputBb.top - cy);
            int end_y = std::min(_tileSize - 1, inputBb.getBottom() - cy);

            std::vector<image::CachedTile::smart_pointer>& row = _tilesArray[ti];

            for(int j = 0; j < gridBb.width; j++)
            {
                int tj = gridBb.left + j;
                int cx = tj * _tileSize;
                int sx = outputBb.left + j * _tileSize;

                int start_x = std::max(0, inputBb.left - cx);
                int end_x = std::min(_tileSize - 1, inputBb.getRight() - cx);

                image::CachedTile::smart_pointer ptr = row[tj];
                if(!ptr)
                {
                    continue;
                }

                if(!ptr->acquire())
                {
                    continue;
                }

                T* data = (T*)ptr->getDataPointer();

                for(int y = start_y; y <= end_y; y++)
                {
                    for(int x = start_x; x <= end_x; x++)
                    {
                        output(sy + y, sx + x) = data[y * _tileSize + x];
                    }
                }
            }
        }

        return true;
    }

    std::vector<std::vector<image::CachedTile::smart_pointer>>& getTiles() { return _tilesArray; }

    int getWidth() const { return _width; }

    int getHeight() const { return _height; }

private:
    int _width;
    int _height;
    int _memoryWidth;
    int _memoryHeight;
    int _tileSize;

    std::vector<std::vector<image::CachedTile::smart_pointer>> _tilesArray;
};

template <>
bool CachedImage<image::RGBAfColor>::writeImage(const std::string& path)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::ImageSpec spec(_memoryWidth, _memoryHeight, 4, oiio::TypeDesc::FLOAT);
    spec.tile_width = _tileSize;
    spec.tile_height = _tileSize;

    if(!out->open(path, spec))
    {
        return false;
    }
    
    for(int i = 0; i < _tilesArray.size(); i++)
    {

        std::vector<image::CachedTile::smart_pointer>& row = _tilesArray[i];

        for(int j = 0; j < row.size(); j++)
        {

            if(!row[j]->acquire())
            {
                return false;
            }

            unsigned char* ptr = row[j]->getDataPointer();

            out->write_tile(j * _tileSize, i * _tileSize, 0, oiio::TypeDesc::FLOAT, ptr);
        }
    }

    out->close();

    return true;
}

template <>
bool CachedImage<IndexT>::writeImage(const std::string& path)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::ImageSpec spec(_memoryWidth, _memoryHeight, 1, oiio::TypeDesc::UINT32);
    spec.tile_width = _tileSize;
    spec.tile_height = _tileSize;

    if(!out->open(path, spec))
    {
        return false;
    }
    
    for(int i = 0; i < _tilesArray.size(); i++)
    {

        std::vector<image::CachedTile::smart_pointer>& row = _tilesArray[i];

        for(int j = 0; j < row.size(); j++)
        {

            if(!row[j]->acquire())
            {
                return false;
            }

            unsigned char* ptr = row[j]->getDataPointer();

            out->write_tile(j * _tileSize, i * _tileSize, 0, oiio::TypeDesc::UINT32, ptr);
        }
    }

    out->close();

    return true;
}

} // namespace aliceVision