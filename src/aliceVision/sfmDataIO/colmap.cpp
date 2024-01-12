// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "colmap.hpp"
#include <aliceVision/geometry/Pose3.hpp>
#include <aliceVision/camera/cameraCommon.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace aliceVision {
namespace sfmDataIO {

ColmapConfig::ColmapConfig(const std::string& basePath)
  : _basePath(basePath)
{
    _sparseDirectory = (fs::path(_basePath) / fs::path("sparse/0")).string();
    _denseDirectory = (fs::path(_basePath) / fs::path("dense/0")).string();
    _imagesDirectory = (fs::path(_basePath) / fs::path("images/")).string();
    _camerasTxtPath = (fs::path(_sparseDirectory) / fs::path("cameras.txt")).string();
    _imagesTxtPath = (fs::path(_sparseDirectory) / fs::path("images.txt")).string();
    _points3DPath = (fs::path(_sparseDirectory) / fs::path("points3D.txt")).string();
}

const std::set<camera::EINTRINSIC>& colmapCompatibleIntrinsics()
{
    static const std::set<camera::EINTRINSIC> compatibleIntrinsics{camera::PINHOLE_CAMERA,
                                                                   camera::PINHOLE_CAMERA_RADIAL1,
                                                                   camera::PINHOLE_CAMERA_RADIAL3,
                                                                   camera::PINHOLE_CAMERA_BROWN,
                                                                   camera::PINHOLE_CAMERA_FISHEYE,
                                                                   camera::PINHOLE_CAMERA_FISHEYE1};
    return compatibleIntrinsics;
}

bool isColmapCompatible(camera::EINTRINSIC intrinsicType)
{
    const auto compatible = colmapCompatibleIntrinsics();
    return compatible.find(intrinsicType) != compatible.end();
}

CompatibleList getColmapCompatibleIntrinsics(const sfmData::SfMData& sfMData)
{
    CompatibleList compatible;
    for (const auto& iter : sfMData.getIntrinsics())
    {
        const auto& intrinsics = iter.second;
        const auto intrID = iter.first;
        const auto intrType = intrinsics->getType();
        if (isColmapCompatible(intrType))
        {
            compatible.insert(intrID);
        }
        else
        {
            ALICEVISION_LOG_INFO("The intrinsics " << intrID << " is of type " << intrType
                                                   << " which is not supported in Colmap and it will be removed");
        }
    }
    return compatible;
}

CompatibleList getColmapCompatibleViews(const sfmData::SfMData& sfmData)
{
    const auto compatibleIntrinsics = getColmapCompatibleIntrinsics(sfmData);

    CompatibleList compatibleViews;
    for (const auto& iter : sfmData.getViews())
    {
        const auto& view = iter.second;
        const auto viewID = iter.first;
        if (!sfmData.isPoseAndIntrinsicDefined(view.get()))
        {
            continue;
        }
        if (compatibleIntrinsics.count(view->getIntrinsicId()) > 0)
        {
            compatibleViews.insert(viewID);
        }
    }
    return compatibleViews;
}

std::string convertIntrinsicsToColmapString(const IndexT intrinsicsID, std::shared_ptr<camera::IntrinsicBase> intrinsic)
{
    std::stringstream intrString;

    camera::EINTRINSIC current_type = intrinsic->getType();
    switch (current_type)
    {
        case camera::PINHOLE_CAMERA:
            // PINHOLE_CAMERA corresponds to Colmap's PINHOLE
            // Parameters: f, cx, cy
            {
                std::shared_ptr<camera::Pinhole> pinhole_intrinsic = std::dynamic_pointer_cast<camera::Pinhole>(intrinsic);

                intrString << intrinsicsID << " "
                           << "PINHOLE"
                           << " " << pinhole_intrinsic->w() << " " << pinhole_intrinsic->h() << " " << pinhole_intrinsic->getFocalLengthPixX() << " "
                           << pinhole_intrinsic->getFocalLengthPixY() << " " << pinhole_intrinsic->getPrincipalPoint().x() << " "
                           << pinhole_intrinsic->getPrincipalPoint().y() << "\n";
            }
            break;
        case camera::PINHOLE_CAMERA_RADIAL1:
            // PINHOLE_CAMERA_RADIAL1 can only be modeled by Colmap's FULL_OPENCV setting the unused parameters to 0
            // Parameters: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
            {
                std::shared_ptr<camera::Pinhole> pinhole_intrinsic_radial = std::dynamic_pointer_cast<camera::Pinhole>(intrinsic);

                intrString << intrinsicsID << " "
                           << "FULL_OPENCV"
                           << " " << pinhole_intrinsic_radial->w() << " " << pinhole_intrinsic_radial->h() << " "
                           << pinhole_intrinsic_radial->getFocalLengthPixX() << " " << pinhole_intrinsic_radial->getFocalLengthPixY() << " "
                           << pinhole_intrinsic_radial->getPrincipalPoint().x() << " " << pinhole_intrinsic_radial->getPrincipalPoint().y() << " "
                           << pinhole_intrinsic_radial->getParams().at(4)
                           << " "
                           // k2, p1, p2, k3, k4, k5, k6
                           << "0.0 0.0 0.0 0.0 0.0 0.0 0.0"
                           << "\n";
            }
            break;
        case camera::PINHOLE_CAMERA_RADIAL3:
            // PINHOLE_CAMERA_RADIAL3 can only be modeled by Colmap's FULL_OPENCV setting the unused parameters to 0
            // Parameters: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
            {
                std::shared_ptr<camera::Pinhole> pinhole_intrinsic_radial = std::dynamic_pointer_cast<camera::Pinhole>(intrinsic);

                intrString << intrinsicsID << " "
                           << "FULL_OPENCV"
                           << " " << pinhole_intrinsic_radial->w() << " " << pinhole_intrinsic_radial->h() << " "
                           << pinhole_intrinsic_radial->getFocalLengthPixX() << " " << pinhole_intrinsic_radial->getFocalLengthPixY() << " "
                           << pinhole_intrinsic_radial->getPrincipalPoint().x() << " " << pinhole_intrinsic_radial->getPrincipalPoint().y() << " "
                           << pinhole_intrinsic_radial->getParams().at(4) << " " << pinhole_intrinsic_radial->getParams().at(5)
                           << " "
                           // tangential params p1 and p2
                           << 0.0 << " " << 0.0
                           << " "
                           // k3
                           << pinhole_intrinsic_radial->getParams().at(6)
                           << " "
                           // remaining radial params k4-k6
                           << 0.0 << " " << 0.0 << " " << 0.0 << "\n";
            }
            break;
        case camera::PINHOLE_CAMERA_BROWN:
            // PINHOLE_CAMERA_BROWN can only be modeled by Colmap's FULL_OPENCV setting the unused parameters to 0
            // Parameters: fx, fy, cx, cy, k1, k2, p1, p2, k3, k4, k5, k6
            {
                std::shared_ptr<camera::Pinhole> pinholeCameraBrownIntrinsics = std::dynamic_pointer_cast<camera::Pinhole>(intrinsic);

                intrString << intrinsicsID << " "
                           << "FULL_OPENCV"
                           << " " << pinholeCameraBrownIntrinsics->w() << " " << pinholeCameraBrownIntrinsics->h() << " "
                           << pinholeCameraBrownIntrinsics->getFocalLengthPixX() << " " << pinholeCameraBrownIntrinsics->getFocalLengthPixY() << " "
                           << pinholeCameraBrownIntrinsics->getPrincipalPoint().x() << " " << pinholeCameraBrownIntrinsics->getPrincipalPoint().y()
                           << " " << pinholeCameraBrownIntrinsics->getParams().at(4) << " " << pinholeCameraBrownIntrinsics->getParams().at(5)
                           << " "
                           // tangential params p1 and p2
                           << pinholeCameraBrownIntrinsics->getParams().at(7) << " " << pinholeCameraBrownIntrinsics->getParams().at(8)
                           << " "
                           // k3
                           << pinholeCameraBrownIntrinsics->getParams().at(6)
                           << " "
                           // remaining radial params k4-k6
                           << 0.0 << " " << 0.0 << " " << 0.0 << "\n";
            }
            break;
        case camera::PINHOLE_CAMERA_FISHEYE:
            // PINHOLE_CAMERA_FISHEYE is modeled by Colmap's OPENCV_FISHEYE
            // Parameters: fx, fy, cx, cy, k1, k2, k3, k4
            {
                std::shared_ptr<camera::Pinhole> pinholeIntrinsicFisheye = std::dynamic_pointer_cast<camera::Pinhole>(intrinsic);

                intrString << intrinsicsID << " "
                           << "OPENCV_FISHEYE"
                           << " " << pinholeIntrinsicFisheye->w() << " " << pinholeIntrinsicFisheye->h() << " "
                           << pinholeIntrinsicFisheye->getFocalLengthPixX() << " " << pinholeIntrinsicFisheye->getFocalLengthPixY() << " "
                           << pinholeIntrinsicFisheye->getPrincipalPoint().x() << " " << pinholeIntrinsicFisheye->getPrincipalPoint().y() << " "
                           << pinholeIntrinsicFisheye->getParams().at(4) << " " << pinholeIntrinsicFisheye->getParams().at(5) << " "
                           << pinholeIntrinsicFisheye->getParams().at(6) << " " << pinholeIntrinsicFisheye->getParams().at(7) << "\n";
            }
            break;
        case camera::PINHOLE_CAMERA_FISHEYE1:
            // PINHOLE_CAMERA_FISHEYE corresponds to Colmap's FOV
            // Parameters: fx, fy, cx, cy, k1, k2, k3, k4
            {
                std::shared_ptr<camera::Pinhole> pinholeIntrinsicFisheye = std::dynamic_pointer_cast<camera::Pinhole>(intrinsic);

                intrString << intrinsicsID << " "
                           << "FOV"
                           << " " << pinholeIntrinsicFisheye->w() << " " << pinholeIntrinsicFisheye->h() << " "
                           << pinholeIntrinsicFisheye->getFocalLengthPixX() << " " << pinholeIntrinsicFisheye->getFocalLengthPixY() << " "
                           << pinholeIntrinsicFisheye->getPrincipalPoint().x() << " " << pinholeIntrinsicFisheye->getPrincipalPoint().y() << " "
                           << pinholeIntrinsicFisheye->getParams().at(4) << "\n";
            }
            break;
        default:
            throw std::invalid_argument("The intrinsics " + EINTRINSIC_enumToString(current_type) + " for camera " + std::to_string(intrinsicsID) +
                                        " are not supported in Colmap");
    }
    return intrString.str();
}

void generateColmapCamerasTxtFile(const sfmData::SfMData& sfmData, const std::string& filename)
{
    // Adapted from Colmap Reconstruction::WriteCamerasText()
    // The cameras.txt file has the following header
    static const std::string camerasHeader = "# Camera list with one line of data per camera:\n"
                                             "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";

    std::ofstream outfile(filename);

    if (!outfile)
    {
        ALICEVISION_LOG_ERROR("Unable to create the cameras file " << filename);
        throw std::runtime_error("Unable to create the cameras file " + filename);
    }

    outfile << camerasHeader;
    outfile << "# Number of cameras: " << sfmData.getIntrinsics().size() << "\n";

    std::vector<std::string> camera_lines;

    for (const auto& iter : sfmData.getIntrinsics())
    {
        const IndexT intrID = iter.first;
        std::shared_ptr<camera::IntrinsicBase> intrinsic = iter.second;

        if (isColmapCompatible(intrinsic->getType()))
        {
            outfile << convertIntrinsicsToColmapString(intrID, intrinsic);
        }
    }
}

PerViewVisibility computePerViewVisibility(const sfmData::SfMData& sfmData, const CompatibleList& viewSelections)
{
    PerViewVisibility perCameraVisibility;

    for (const auto& land : sfmData.getLandmarks())
    {
        const IndexT landID = land.first;
        const auto& observations = land.second.getObservations();

        for (const auto& iter : observations)
        {
            const IndexT viewID = iter.first;
            const auto& obs = iter.second;

            if (viewSelections.find(viewID) != viewSelections.end())
            {
                // for the current viewID add the feature point and its associate 3D point's ID
                perCameraVisibility[viewID][landID] = obs.getCoordinates();
            }
        }
    }
    return perCameraVisibility;
}

void generateColmapImagesTxtFile(const sfmData::SfMData& sfmData, const CompatibleList& viewSelections, const std::string& filename)
{
    // adapted from Colmap's Reconstruction::WriteImagesText()
    // An images.txt file has the following format
    static const std::string imageHeader = "# Image list with two lines of data per image:\n"
                                           "#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
                                           "#   POINTS2D[] as (X, Y, POINT3D_ID)\n";

    std::ofstream outfile(filename);

    if (!outfile)
    {
        ALICEVISION_LOG_ERROR("Unable to create the image file " << filename);
        throw std::runtime_error("Unable to create the image file " + filename);
    }
    // Ensure no loss of precision by storing in text.
    outfile.precision(17);
    outfile << imageHeader;

    // colmap requires per-camera visibility rather per-landmark, so for each camera we need to create the relevant
    // visibility list
    const auto perViewVisibility = computePerViewVisibility(sfmData, viewSelections);

    // for each view to export add a line with the pose and the intrinsics ID and another with point visibility
    for (const auto& iter : sfmData.getViews())
    {
        const auto view = iter.second.get();
        const auto viewID = view->getViewId();

        if (viewSelections.find(viewID) == viewSelections.end())
        {
            continue;
        }

        // this is necessary if we copy the images in the image folder because the image path is absolute in AV
        const std::string imageFilename = fs::path(view->getImage().getImagePath()).filename().string();
        const IndexT intrID = view->getIntrinsicId();

        const auto pose = sfmData.getPose(*view).getTransform();
        const Mat3 rot = pose.rotation();
        const Vec3 tra = pose.translation();
        Eigen::Quaterniond quat(rot);

        //"#   IMAGE_ID, QW, QX, QY, QZ, TX, TY, TZ, CAMERA_ID, NAME\n"
        //"#   POINTS2D[] as (X, Y, POINT3D_ID)\n"
        outfile << viewID << " " << quat.w() << " " << quat.x() << " " << quat.y() << " " << quat.z() << " " << tra[0] << " " << tra[1] << " "
                << tra[2] << " " << intrID << " " << imageFilename << "\n";

        for (const auto& obs : perViewVisibility.at(viewID))
        {
            const auto& id = obs.first;
            const auto& pts = obs.second;
            outfile << pts.x() << " " << pts.y() << " " << id << " ";
        }
        outfile << "\n";
    }
}

void copyImagesFromSfmData(const sfmData::SfMData& sfmData, const std::string& destinationFolder, const CompatibleList selection)
{
    for (const auto viewId : selection)
    {
        const auto& view = sfmData.getView(viewId);
        const auto& from = fs::path(view.getImage().getImagePath());
        const auto& to = fs::path(destinationFolder) / from.filename();
        fs::copy_file(from, to);
    }
}

void generateColmapPoints3DTxtFile(const sfmData::SfMData& sfmData, const CompatibleList& viewSelections, const std::string& filename)
{
    // adapted from Colmap's Reconstruction::WritePoints3DText()
    // The points3D.txt file has the following header
    static const std::string points3dHeader = "# 3D point list with one line of data per point:\n"
                                              "#   POINT3D_ID, X, Y, Z, R, G, B, ERROR, TRACK[] as (IMAGE_ID, POINT2D_IDX)\n";

    std::ofstream outfile(filename);

    if (!outfile)
    {
        ALICEVISION_LOG_ERROR("Unable to create the 3D point file " << filename);
        throw std::runtime_error("Unable to create the 3D point file " + filename);
    }

    outfile << points3dHeader;
    // default reprojection error (not used)
    const double defaultError{-1.0};

    for (const auto& iter : sfmData.getLandmarks())
    {
        const IndexT id = iter.first;
        const Vec3 exportPoint = iter.second.X;
        const auto pointColor = iter.second.rgb;
        outfile << id << " " << exportPoint.x() << " " << exportPoint.y() << " " << exportPoint.z() << " " << static_cast<int>(pointColor.r()) << " "
                << static_cast<int>(pointColor.g()) << " " << static_cast<int>(pointColor.b()) << " ";

        outfile << defaultError;

        for (const auto& itObs : iter.second.getObservations())
        {
            const IndexT viewId = itObs.first;
            const IndexT featId = itObs.second.getFeatureId();

            if (viewSelections.find(viewId) != viewSelections.end())
            {
                outfile << " " << viewId << " " << featId;
            }
        }
        outfile << "\n";
    }
}

void generateColmapSceneFiles(const sfmData::SfMData& sfmData, const CompatibleList& viewsSelection, const ColmapConfig& colmapParams)
{
    generateColmapCamerasTxtFile(sfmData, colmapParams._camerasTxtPath);
    generateColmapImagesTxtFile(sfmData, viewsSelection, colmapParams._imagesTxtPath);
    generateColmapPoints3DTxtFile(sfmData, viewsSelection, colmapParams._points3DPath);
}

void create_directories(const std::string& directory)
{
    const bool ok = fs::create_directories(directory);
    if (!ok)
    {
        ALICEVISION_LOG_ERROR("Cannot create directory " << directory);
        throw std::runtime_error("Cannot create directory " + directory);
    }
}

void convertToColmapScene(const sfmData::SfMData& sfmData, const std::string& colmapBaseDir, bool copyImages)
{
    // retrieve the views that are compatible with Colmap and that can be exported
    const auto views2export = getColmapCompatibleViews(sfmData);
    if (views2export.empty())
    {
        ALICEVISION_LOG_WARNING("No camera in the scene is compatible with Colmap");
        return;
    }
    ALICEVISION_LOG_INFO("Found " << views2export.size() << "/" << sfmData.getValidViews().size()
                                  << " views to export "
                                     "that are compatible with Colmap");

    ColmapConfig colmapParams{colmapBaseDir};

    // Generate the folder structure for colmap
    ALICEVISION_LOG_INFO("Creating Colmap subfolders...");
    create_directories(colmapParams._sparseDirectory);
    create_directories(colmapParams._denseDirectory);
    create_directories(colmapParams._imagesDirectory);

    if (copyImages)
    {
        ALICEVISION_LOG_INFO("Copying source images...");
        copyImagesFromSfmData(sfmData, colmapParams._imagesDirectory, views2export);
    }

    ALICEVISION_LOG_INFO("Generating Colmap files...");
    generateColmapSceneFiles(sfmData, views2export, colmapParams);
}

}  // namespace sfmDataIO
}  // namespace aliceVision
