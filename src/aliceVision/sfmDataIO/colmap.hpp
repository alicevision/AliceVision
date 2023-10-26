// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/camera/cameraCommon.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/types.hpp>

#include <string>
#include <set>
#include <unordered_set>

namespace aliceVision {
namespace sfmDataIO {

using CompatibleList = std::unordered_set<IndexT>;

/**
 * @brief Struct containing the configuration parameters for the Colmap's scene structure
 */
struct ColmapConfig
{
    /**
     * @brief Initialize the configuration from the given base path.
     * @param[in] basePath The path where the directory structure and the various Colmap's file will be created.
     */
    ColmapConfig(const std::string& basePath);

    /// the base path
    std::string _basePath{};
    /// the directory containing the sparse reconstruction
    std::string _sparseDirectory{};
    /// the directory containing the dense reconstruction
    std::string _denseDirectory{};
    /// the directory containing the undistorted images
    std::string _imagesDirectory{};
    /// the full path for the cameras.txt file
    std::string _camerasTxtPath{};
    /// the full path for the images.txt file
    std::string _imagesTxtPath{};
    /// the full path for the points3d.txt file
    std::string _points3DPath{};
};

/**
 * @brief Return the set of intrinsic types that are compatible with those of Colmap.
 * @return the set of compatible intrinsics.
 */
const std::set<camera::EINTRINSIC>& colmapCompatibleIntrinsics();

/**
 * @brief Check whether a given intrinsic type is compatible with Colmap.
 * @param[in] intrinsicType the intrinsic to check.
 * @return \p true if the intrinsic is compatible.
 */
bool isColmapCompatible(camera::EINTRINSIC intrinsicType);

/**
 * @brief Given an SfMData scene returns the list of the IDs of the Colmap-compatible intrinsics.
 * @param[in] sfMData the sfm scene.
 * @return the set of IDs of the intrinsics that are compatible with Colmap.
 */
CompatibleList getColmapCompatibleIntrinsics(const sfmData::SfMData& sfMData);

/**
 * @brief Given an SfMData scene returns the list of the IDs of the views that have Colmap-compatible intrinsics.
 * @param[in] sfmData the sfm scene.
 * @return the set of IDs of the views that have intrinsics compatible with Colmap.
 */
CompatibleList getColmapCompatibleViews(const sfmData::SfMData& sfmData);

/**
 * @brief Convert the given intrinsic to the equivalent string to be placed in a cameras.txt file.
 * @param[in] intrinsicsID the id of the intrinsics.
 * @param[in] intrinsic the intrinsics.
 * @return a string format for the colmap equivalent intrinsics in the format CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[].
 * @throws std::runtime_error() if the intrinsic is not compatible with Colmap.
 */
std::string convertIntrinsicsToColmapString(const IndexT intrinsicsID, std::shared_ptr<camera::IntrinsicBase> intrinsic);

/**
 * @brief Given the sfmData it generates the equivalent cameras.txt file with the Colmap compatible cameras.
 * @param[in] sfmData the input sfmData.
 * @param[in] filename the filename where to save the Colmap's scene (usually a cameras.txt)
 */
void generateColmapCamerasTxtFile(const sfmData::SfMData& sfmData, const std::string& filename);

/// map< viewID, map<landmarkID, pt2d>>
using PerViewVisibility = std::map<IndexT, std::map<IndexT, Vec2>>;

/**
 * @brief Given an sfm scene and a selection of its views, it computes the visibility of the 3D points per each view:
 * for each view, it maps a 2d feature of the view to its relevant 3D point.
 * @param[in] sfmData the input scene.
 * @param[in] viewSelections a selection of view IDs.
 * @return the visibility of 3D points for each view.
 */
PerViewVisibility computePerViewVisibility(const sfmData::SfMData& sfmData, const CompatibleList& viewSelections);

/**
 * @brief Given an sfm scene and a selection of its views that are compatible with Colmap, it generates the images.txt
 * file for Colmap scene.
 * @param[in] sfmData  the input scene.
 * @param[in] viewSelections  a selection of view IDs that have compatible intrinsics with Colmap.
 * @param[in] filename the filename where to save the scene (usually a images.txt file)
 */
void generateColmapImagesTxtFile(const sfmData::SfMData& sfmData, const CompatibleList& viewSelections, const std::string& filename);

/**
 * @brief Given an sfm scene and a selection of its views that are compatible with Colmap, it copies the source images to
 * the given directory.
 * @param[in] sfmData the input scene.
 * @param[in] destinationFolder the folder where to copy the images.
 * @param[in] selection  a selection of view IDs that have compatible intrinsics with Colmap.
 */
void copyImagesFromSfmData(const sfmData::SfMData& sfmData, const std::string& destinationFolder, const CompatibleList selection);

/**
 * @brief Given an sfm scene and a selection of its views that are compatible with Colmap, it generates the points3d.txt
 * file for Colmap scene.
 * @param[in] sfmData the input scene.
 * @param[in] viewSelections a selection of view IDs that have compatible intrinsics with Colmap.
 * @param[in] filename the filename where to save the points (usually a points3d.txt file)
 */
void generateColmapPoints3DTxtFile(const sfmData::SfMData& sfmData, const CompatibleList& viewSelections, const std::string& filename);

/**
 * @brief Given an sfm scene and a selection of its views that are compatible with Colmap, it generates all the Colmap
 * files that represent the scene.
 * @param sfmData the input scene.
 * @param viewsSelection a selection of view IDs that have compatible intrinsics with Colmap.
 * @param colmapParams the configuration data for the Colmap scene.
 */
void generateColmapSceneFiles(const sfmData::SfMData& sfmData, const CompatibleList& viewsSelection, const ColmapConfig& colmapParams);

/**
 * @brief iven an sfm scene it generate the folder structure and all the files in Colmap format.
 * @param sfmData the input scene.
 * @param colmapBaseDir the base path where to generate the equivalent Colmap scene.
 * @param copyImages enable copying the source image into the Colmap scene folder. This can be set to false when the
 * sfmData scene contains images from a single directory: in this case copy can be skipped and the first undistort step
 * of Colmap's MVS process can be called directly on the source folder without copying the images.
 */
void convertToColmapScene(const sfmData::SfMData& sfmData, const std::string& colmapBaseDir, bool copyImages);

}  // namespace sfmDataIO
}  // namespace aliceVision