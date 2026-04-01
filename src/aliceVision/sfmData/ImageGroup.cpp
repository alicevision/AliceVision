// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "ImageGroup.hpp"
#include <aliceVision/sfmData/ImageSet.hpp>
#include <aliceVision/sfmData/ImageSequence.hpp>

namespace aliceVision {
namespace sfmData {

std::string ImageGroup::typeToString(Type type)
{
    switch (type)
    {
        case Type::ImageSet:
            return "ImageSet";
        case Type::ImageSequence:
            return "ImageSequence";
    }

    throw std::invalid_argument("Invalid ImageGroup::Type enum value");
}

ImageGroup::Type ImageGroup::stringToType(const std::string& typeStr)
{
    if (typeStr == "ImageSet")
    {
        return ImageGroup::Type::ImageSet;
    }

    if (typeStr == "ImageSequence")
    {
        return ImageGroup::Type::ImageSequence;
    }
    
    throw std::invalid_argument("Invalid ImageGroup type string: " + typeStr);
}

ImageGroup::sptr ImageGroup::create(const ImageGroup::Type & type)
{
    if (type == ImageGroup::Type::ImageSet)
    {
        return std::make_shared<ImageSet>();
    }
    else if (type == ImageGroup::Type::ImageSequence)
    {
        return std::make_shared<ImageSequence>();
    }

    throw std::invalid_argument("Invalid ImageGroup Type enum value in ImageGroup::create");
}

}  // namespace sfmData
}  // namespace aliceVision
