// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/ImageGroup.hpp>
#include <aliceVision/types.hpp>
#include <memory>
#include <set>

namespace aliceVision {
namespace sfmData {

/**
 * @brief ImageSequence class - a concrete implementation of ImageGroup
 * Represents a set of images with a sequence ordering
 */
class ImageSequence : public ImageGroup
{
public:
    using ptr = ImageSequence*;
    using sptr = std::shared_ptr<ImageSequence>;

public:
    /**
     * @brief Default constructor
     */
    ImageSequence() = default;

    /**
     * @brief Clone this and return pointer
     * @return a pointer to the newly created object
     */
    ImageGroup::ptr clone() const
    {
        return new ImageSequence(*this);
    }

    /**
     * @brief Equality operator
     * @param other The image group to compare with
     * @return true if image groups are equal, false otherwise
     */
    bool operator==(const ImageGroup& other) const
    {
        if (!ImageGroup::operator==(other))
        {
            return false;
        }

        return true;
    }

    /**
     * @brief Get the type
     * @return Type::ImageSet
     */
    Type getType() const override 
    {
        return Type::ImageSequence;
    }
};

}  // namespace sfmData
}  // namespace aliceVision
