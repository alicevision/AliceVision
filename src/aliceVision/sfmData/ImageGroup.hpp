// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/types.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <memory>
#include <string>
#include <stdexcept>

namespace aliceVision {
namespace sfmData {

class ImageGroup
{
public:
    using ptr = ImageGroup*;
    using sptr = std::shared_ptr<ImageGroup>;

public:
    enum class Type : int
    {
        ImageSet = 0,
        ImageSequence = 1
    };

    /**
     * @brief Convert Type enum to string
     * @param type The Type enum value to convert
     * @return String representation of the type
     */
    static std::string typeToString(Type type);

    /**
     * @brief Convert string to Type enum
     * @param typeStr The string to convert
     * @return The corresponding Type enum value
     * @throw std::invalid_argument if the string doesn't match any Type
     */
    static Type stringToType(const std::string& typeStr);

    
    static ImageGroup::sptr create(const Type & type);

public:
    /**
     * @brief Default constructor
     */
    ImageGroup() {}

    /**
     * @brief Virtual destructor to allow safe polymorphic deletion
     */
    virtual ~ImageGroup() = default;

    /**
    * @brief Clone this and return pointer
    * @return a pointer to the newly created object
     */
    virtual ImageGroup::ptr clone() const = 0;

    /**
     * @brief Equality operator
     * @param other The image group to compare with
     * @return true if image groups are equal, false otherwise
     */
    bool operator==(const ImageGroup& other) const
    {
        if (getType() != other.getType())
        {
            return false;
        }

        return true;
    }

    /**
     * @brief Inequality operator
     * @param other The image group to compare with
     * @return true if image groups are not equal, false otherwise
     */
    inline bool operator!=(const ImageGroup& other) const { return !(*this == other); }

    /**
    * @brief get the type
    * @return an element of the enum 'Type'
    */
    virtual Type getType() const = 0;

    /**
     * @brief Check if this group is a nodal camera group
     * @return true if this is a nodal camera group
     */
    bool isNodalCamera() const { return _isNodalCamera; }

    /**
     * @brief Set whether this group is a nodal camera group
     * @param isNodal true if this is a nodal camera group
     */
    void setIsNodalCamera(bool isNodal) { _isNodalCamera = isNodal; }

    /**
     * @brief Get the shared nodal center for all cameras in this group.
     *        Only meaningful when isNodalCamera() is true.
     * @return The shared center position
     */
    const Vec3& getCenter() const { return _center; }

    /**
     * @brief Set the shared nodal center for all cameras in this group.
     *        Only meaningful when isNodalCamera() is true.
     * @param center The shared center position
     */
    void setCenter(const Vec3& center) { _center = center; }

protected:
    bool _isNodalCamera = false;
    Vec3 _center = Vec3::Zero();
};

}  // namespace sfmData
}  // namespace aliceVision
