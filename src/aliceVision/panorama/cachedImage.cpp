#include "cachedImage.hpp"

namespace aliceVision
{

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
bool CachedImage<image::RGBfColor>::writeImage(const std::string& path)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::ImageSpec spec(_memoryWidth, _memoryHeight, 3, oiio::TypeDesc::FLOAT);
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

template <>
bool CachedImage<unsigned char>::writeImage(const std::string& path)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::ImageSpec spec(_memoryWidth, _memoryHeight, 1, oiio::TypeDesc::UINT8);
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

            out->write_tile(j * _tileSize, i * _tileSize, 0, oiio::TypeDesc::UINT8, ptr);
        }
    }

    out->close();

    return true;
}

} // namespace aliceVision