// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Material.hpp"

namespace aliceVision {
namespace mesh {

void Material::addTexture(TextureType type, const std::string& textureName)
{
    // get extension and prefix
    std::size_t found = textureName.rfind('.');
    if (found == std::string::npos)
    {
        throw std::runtime_error("Texture file name has no extension!");
    }

    const std::string prefix = textureName.substr(0, found - 4);
    const std::string extension = textureName.substr(found + 1);

    switch (type)
    {
        case TextureType::BUMP:
            if (_bumpTextures.empty())
            {
                bumpPrefix = prefix;
                bumpType = image::EImageFileType_stringToEnum(extension);
            }
            else if (bumpPrefix != prefix || bumpType != image::EImageFileType_stringToEnum(extension))
            {
                throw std::runtime_error("All texture tiles must have the same prefix and extension!");
            }
            _bumpTextures.push_back(textureName);
            break;
        case TextureType::DIFFUSE:
            if (_diffuseTextures.empty())
            {
                diffusePrefix = prefix;
                diffuseType = image::EImageFileType_stringToEnum(extension);
            }
            else if (diffusePrefix != prefix || diffuseType != image::EImageFileType_stringToEnum(extension))
            {
                throw std::runtime_error("All texture tiles must have the same prefix and extension!");
            }
            _diffuseTextures.push_back(textureName);
            break;
        case TextureType::DISPLACEMENT:
            if (_displacementTextures.empty())
            {
                displacementPrefix = prefix;
                displacementType = image::EImageFileType_stringToEnum(extension);
            }
            else if (displacementPrefix != prefix || displacementType != image::EImageFileType_stringToEnum(extension))
            {
                throw std::runtime_error("All texture tiles must have the same prefix and extension!");
            }
            _displacementTextures.push_back(textureName);
            break;
        case TextureType::NORMAL:
            if (_normalTextures.empty())
            {
                normalPrefix = prefix;
                normalType = image::EImageFileType_stringToEnum(extension);
            }
            else if (normalPrefix != prefix || normalType != image::EImageFileType_stringToEnum(extension))
            {
                throw std::runtime_error("All texture tiles must have the same prefix and extension!");
            }
            _normalTextures.push_back(textureName);
            break;
        default:
            throw std::runtime_error("Unknown texture type!");
    }
}

const StaticVector<std::string>& Material::getTextures(TextureType type) const
{
    switch (type)
    {
        case TextureType::BUMP:
            return _bumpTextures;
        case TextureType::DIFFUSE:
            return _diffuseTextures;
        case TextureType::DISPLACEMENT:
            return _displacementTextures;
        case TextureType::NORMAL:
            return _normalTextures;
        default:
            throw std::runtime_error("Unknown texture type!");
    }
}

StaticVector<std::string> Material::getAllTextures() const
{
    StaticVector<std::string> textures;
    textures.resize(_bumpTextures.size() + _diffuseTextures.size() + _displacementTextures.size() + _normalTextures.size());

    auto last = std::copy(_bumpTextures.begin(), _bumpTextures.end(), textures.begin());
    last = std::copy(_diffuseTextures.begin(), _diffuseTextures.end(), last);
    last = std::copy(_displacementTextures.begin(), _displacementTextures.end(), last);
    std::copy(_normalTextures.begin(), _normalTextures.end(), last);

    return textures;
}

bool Material::hasTextures(TextureType type) const
{
    switch (type)
    {
        case TextureType::BUMP:
            return bumpType != image::EImageFileType::NONE;
        case TextureType::DIFFUSE:
            return diffuseType != image::EImageFileType::NONE;
        case TextureType::DISPLACEMENT:
            return displacementType != image::EImageFileType::NONE;
        case TextureType::NORMAL:
            return normalType != image::EImageFileType::NONE;
        default:
            throw std::runtime_error("Unknown texture type!");
    }
}

int Material::numAtlases() const
{
    const int num = _diffuseTextures.size();

    if ((!_displacementTextures.empty() && _displacementTextures.size() != num) || (!_normalTextures.empty() && _normalTextures.size() != num) ||
        (!_bumpTextures.empty() && _bumpTextures.size() != num))
    {
        throw std::runtime_error("All texture maps must have same number of atlases!");
    }

    return num;
}

void Material::clear()
{
    _diffuseTextures.clear();
    _normalTextures.clear();
    _bumpTextures.clear();
    _displacementTextures.clear();
}

std::string Material::textureName(TextureType type, int index) const
{
    std::string prefix;
    image::EImageFileType fileType;

    switch (type)
    {
        case TextureType::BUMP:
            prefix = bumpPrefix;
            fileType = bumpType;
            break;
        case TextureType::DIFFUSE:
            prefix = diffusePrefix;
            fileType = diffuseType;
            break;
        case TextureType::DISPLACEMENT:
            prefix = displacementPrefix;
            fileType = displacementType;
            break;
        case TextureType::NORMAL:
            prefix = normalPrefix;
            fileType = normalType;
            break;
        default:
            throw std::runtime_error("Unknown texture type!");
    }

    return prefix + textureId(index) + "." + image::EImageFileType_enumToString(fileType);
}

}  // namespace mesh
}  // namespace aliceVision
