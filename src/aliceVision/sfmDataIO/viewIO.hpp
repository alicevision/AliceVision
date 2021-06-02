// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/sfmData/View.hpp>
#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/camera/IntrinsicBase.hpp>

#include <boost/filesystem.hpp>

#include <memory>

namespace aliceVision {
namespace sfmDataIO {

enum class EViewIdMethod
{
    METADATA,
    FILENAME
};

inline std::string EViewIdMethod_enumToString(EViewIdMethod viewIdMethod)
{
    switch(viewIdMethod)
    {
        case EViewIdMethod::METADATA: return "metadata";
        case EViewIdMethod::FILENAME: return "filename";
    }
    throw std::out_of_range("Invalid ViewIdMethod type Enum: " + std::to_string(int(viewIdMethod)));
}

inline EViewIdMethod EViewIdMethod_stringToEnum(const std::string& viewIdMethod)
{
    if(viewIdMethod == "metadata") return EViewIdMethod::METADATA;
    if(viewIdMethod == "filename") return EViewIdMethod::FILENAME;

    throw std::out_of_range("Invalid ViewIdMethod type string " + viewIdMethod);
}

inline std::ostream& operator<<(std::ostream& os, EViewIdMethod s)
{
    return os << EViewIdMethod_enumToString(s);
}

inline std::istream& operator>>(std::istream& in, EViewIdMethod& s)
{
    std::string token;
    in >> token;
    s = EViewIdMethod_stringToEnum(token);
    return in;
}

/**
 * @brief update an incomplete view (at least only the image path)
 * @param view The given incomplete view
 * @param[in] viewIdMethod ViewId generation method to use
 * @param[in] viewIdRegex Optional regex used when viewIdMethod is FILENAME
 */
void updateIncompleteView(sfmData::View& view, EViewIdMethod viewIdMethod = EViewIdMethod::METADATA, const std::string& viewIdRegex = "");

/**
 * @brief create an intrinsic for the given View
 * @param[in] view The given view
 * @param[in] mmFocalLength (-1 if unknown)
 * @param[in] sensorWidth (-1 if unknown)
 * @param[in] defaultFocalLengthPx (-1 if unknown)
 * @param[in] defaultFieldOfView (-1 if unknown)
 * @param[in] defaultIntrinsicType (unknown by default)
 * @param[in] defaultPPx 
 * @param[in] defaultPPy 
 * @param[in] allowedEintrinsics The intrinsics values that can be attributed
 * @return shared_ptr IntrinsicBase
 */
std::shared_ptr<camera::IntrinsicBase> getViewIntrinsic(
					const sfmData::View& view, double mmFocalLength = -1.0, double sensorWidth = -1,
					double defaultFocalLengthPx = -1, double defaultFieldOfView = -1,
					camera::EINTRINSIC defaultIntrinsicType = camera::EINTRINSIC::UNKNOWN,
					camera::EINTRINSIC allowedEintrinsics = camera::EINTRINSIC::VALID_CAMERA_MODEL,
					double defaultPPx = 0, double defaultPPy = 0);

/**
* @brief Allows you to retrieve the files paths corresponding to a view by searching through a list of folders.
*        Filename must be the same or equal to the viewId.
* @param[in] the view
* @param[in] the folder list
* @return the list of paths to the corresponding view if found in the folders, otherwise returns an empty list.
*/
std::vector<std::string> viewPathsFromFolders(const sfmData::View& view, const std::vector<std::string>& folders);


/*
* @brief Allows to detect if an image filename (stripped of its extension) contains a number.
*        This function can be used to detect if an image is part of an image sequence from the analysis of its filename.
*        Expected pattern: (optional prefix which ends with a non digit character)(a number)(optional suffix with at least one letter which starts with a separator ('.', '-', '_'))
*          Examples:
*          IMG01234
*          IMG_01234.cam
*          C4M0123-A
*          01234
* @param[in] imagePathStem the image filename stripped of its extension
* @param[out] number the detected number (or UndefinedIndexT)
* @param[out] prefix the detected prefix (or empty)
* @param[out] suffix the detected suffix (or empty)
* @return true if the image filename (stripped of its extension) contains a number
*/
bool extractNumberFromFileStem(const std::string& imagePathStem, IndexT& number, std::string& prefix, std::string& suffix);

} // namespace sfmDataIO
} // namespace aliceVision
