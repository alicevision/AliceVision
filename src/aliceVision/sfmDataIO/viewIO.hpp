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

/**
 * @brief update an incomplete view (at least only the image path)
 * @param view The given incomplete view
 */
void updateIncompleteView(sfmData::View& view);

/**
 * @brief create an intrinsic for the given View
 * @param[in] view The given view
 * @param[in] mmFocalLength (-1 if unknown)
 * @param[in] sensorWidth (-1 if unknown)
 * @param[in] defaultFocalLengthPx (-1 if unknown)
 * @param[in] defaultFieldOfView (-1 if unknown)
 * @param[in] defaultIntrinsicType (PINHOLE_CAMERA_START if unknown)
 * @param[in] defaultPPx (-1 if unknown)
 * @param[in] defaultPPy (-1 if unknown)
 * @return shared_ptr IntrinsicBase
 */
std::shared_ptr<camera::IntrinsicBase> getViewIntrinsic(const sfmData::View& view,
                                                double mmFocalLength = -1.0,
                                                double sensorWidth = -1,
                                                double defaultFocalLengthPx = -1,
                                                double defaultFieldOfView = -1,
                                                camera::EINTRINSIC defaultIntrinsicType = camera::CAMERA_END,
                                                double defaultPPx = -1,
                                                double defaultPPy = -1);

/**
    * @brief Allows you to retrieve the image file path corresponding to a view by searching through a list of folders.
    *        Filename must be the same or equal to the image uid.
    * @param[in] the view
    * @param[in] the folder list
    * @return the path to the corresponding view if found in the folders, otherwise returns an empty path ("").
    */
boost::filesystem::path viewPathFromFolders(const sfmData::View& view, const std::vector<std::string>& folders);

/**
    * @brief  Allows you to retrieve the image file path corresponding to a view by searching in a folder.
             Filename must be the same or equal to the image uid.
    * @param[in] the view
    * @param[in] the folder path
    * @return the path to the corresponding view if found in the folder, otherwise returns an empty path ("").
    */
boost::filesystem::path viewPathFromFolder(const sfmData::View& view, const std::string& folder);

} // namespace sfmDataIO
} // namespace aliceVision
