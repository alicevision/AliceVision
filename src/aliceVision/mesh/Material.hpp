// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/io.hpp>
#include <aliceVision/mvsData/StaticVector.hpp>

namespace aliceVision {
namespace mesh {

/// Simple material type that supports only the data written out from Texturing pipeline
class Material
{
  public:
    struct Color
    {
        float r = 0.0;
        float g = 0.0;
        float b = 0.0;
    };

    enum class TextureType
    {
        BUMP,
        DIFFUSE,
        DISPLACEMENT,
        NORMAL
    };

    Color diffuse = {0.6, 0.6, 0.6};
    Color ambient = {0.6, 0.6, 0.6};
    Color specular = {0.0, 0.0, 0.0};
    float shininess = 0.0;

    std::string diffusePrefix = "texture_";
    image::EImageFileType diffuseType = image::EImageFileType::NONE;

    std::string displacementPrefix = "Displacement_";
    image::EImageFileType displacementType = image::EImageFileType::NONE;

    std::string normalPrefix = "Normal_";
    image::EImageFileType normalType = image::EImageFileType::NONE;

    std::string bumpPrefix = "Bump_";
    image::EImageFileType bumpType = image::EImageFileType::NONE;

    /// Add texture to material (assumes UDIM naming convention <prefix><udim>.<ext>)
    void addTexture(TextureType type, const std::string& textureName);

    /// Get textures by type
    const StaticVector<std::string>& getTextures(TextureType type) const;

    /// Get all textures used in the material
    StaticVector<std::string> getAllTextures() const;

    /// Check if material has textures of a given type
    bool hasTextures(TextureType type) const;

    /// Get number of UV atlases
    int numAtlases() const;

    /// Clear textures
    void clear();

    /// Get texture name for a given type
    std::string textureName(TextureType type, int index) const;

    /// Get texture ID as a string
    /// Passing a negative index will use the <UDIM> template tag instead of a concrete ID
    static std::string textureId(int index)
    {
        // UDIMs start with 1001 and is conventionally right before extension
        return index < 0 ? "<UDIM>" : std::to_string(1001 + index);
    }

  private:
    StaticVector<std::string> _diffuseTextures;
    StaticVector<std::string> _normalTextures;
    StaticVector<std::string> _bumpTextures;
    StaticVector<std::string> _displacementTextures;
};

}  // namespace mesh
}  // namespace aliceVision
