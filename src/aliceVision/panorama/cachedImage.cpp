#include "cachedImage.hpp"

namespace aliceVision
{

template <>
bool CachedImage<image::RGBAfColor>::writeImage(const std::string& path, const image::EStorageDataType &storageDataType)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::TypeDesc typeColor = oiio::TypeDesc::FLOAT;
    if (storageDataType == image::EStorageDataType::Half || storageDataType == image::EStorageDataType::HalfFinite) 
    {
        typeColor = oiio::TypeDesc::HALF;
    } 
		

    oiio::ImageSpec spec(_width, _height, 4, typeColor);
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
bool CachedImage<image::RGBfColor>::writeImage(const std::string& path, const image::EStorageDataType &storageDataType)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::TypeDesc typeColor = oiio::TypeDesc::FLOAT;
    if (storageDataType == image::EStorageDataType::Half || storageDataType == image::EStorageDataType::HalfFinite) 
    {
        typeColor = oiio::TypeDesc::HALF;
    } 

    oiio::ImageSpec spec(_width, _height, 3, typeColor);
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
bool CachedImage<IndexT>::writeImage(const std::string& path, const image::EStorageDataType &storageDataType)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::ImageSpec spec(_width, _height, 1, oiio::TypeDesc::UINT32);
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
bool CachedImage<float>::writeImage(const std::string& path, const image::EStorageDataType &storageDataType)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::ImageSpec spec(_width, _height, 1, oiio::TypeDesc::FLOAT);
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
bool CachedImage<unsigned char>::writeImage(const std::string& path, const image::EStorageDataType &storageDataType)
{

    std::unique_ptr<oiio::ImageOutput> out = oiio::ImageOutput::create(path);
    if(!out)
    {
        return false;
    }

    oiio::ImageSpec spec(_width, _height, 1, oiio::TypeDesc::UINT8);
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