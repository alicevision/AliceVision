// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "middlebury.hpp"
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/image/io.hpp>

#include <boost/filesystem.hpp>

namespace aliceVision {
namespace sfmDataIO {

namespace bfs = boost::filesystem;

Mat3 extractMat3FromVec(const std::vector<std::string>& entries, std::size_t offset)
{
    // we are supposed to read 9 elements, so the offset must be coherent with the vector size
    if(offset + 9 > entries.size())
    {
        ALICEVISION_LOG_ERROR("The vector has " << entries.size()
                                                << " elements, tried to read out of bounds (offset: " << offset);
        throw std::out_of_range("Trying to read out of the bounds of the vector");
    }
    Mat3 rotation;
    const auto lastIdx{offset + 8};
    for(std::size_t i = offset; i <= lastIdx; ++i)
    {
        const double val = std::stod(entries[i]);
        const auto row = (i - offset) / 3;
        const auto col = (i - offset) % 3;
        rotation(row, col) = val;
    }
    return rotation;
}

void parseMiddleburyCamera(const std::string& line, std::string& imageName, Mat3& matK, Mat3& rotation,
                           Vec3& translation)
{
    std::vector<std::string> entries{};

    std::stringstream sstream(line);
    std::string entry;
    const char spaceChar{' '};
    // tokenize the line extracting all the entries separated by a space
    while(std::getline(sstream, entry, spaceChar))
    {
        entries.push_back(entry);
        ALICEVISION_LOG_TRACE(entry);
    }
    if(entries.size() != 22)
    {
        ALICEVISION_LOG_ERROR("read " << entries.size() << " entries, expected " << 22 << ". Incorrect file format");
        throw std::runtime_error("Error while reading the camera parameters, incorrect number of entries");
    }
    ALICEVISION_LOG_DEBUG("read " << entries.size() << " entries");

    // first entry is the image name
    imageName = entries[0];
    // next 9 elements are the calibration entries
    matK = extractMat3FromVec(entries, 1);
    // next 9 elements are the pose rotation
    rotation = extractMat3FromVec(entries, 10);
    // next and last 3 elements are the translation
    translation(0) = std::stod(entries[19]);
    translation(1) = std::stod(entries[20]);
    translation(2) = std::stod(entries[21]);
}

sfmData::SfMData middleburySceneToSfmData(const std::string& filename, const std::string& basePath,
                                          bool uniqueIntrinsics, bool importPoses, bool lockIntrinsics, bool lockPoses)
{
    std::ifstream infile(filename);
    if(!infile.is_open())
    {
        ALICEVISION_LOG_ERROR("Unable to open " << filename);
        throw std::runtime_error("Unable to open " + filename);
    }

    sfmData::SfMData scene;

    std::string line;

    // the first one is the number of cameras
    std::getline(infile, line);
    const size_t numViews = std::stoul(line);
    ALICEVISION_LOG_INFO("Found " << numViews << " cameras to read");

    // init the ids, we use incremental ids
    IndexT intrinsicsId{0};
    IndexT viewId{0};
    IndexT poseId = importPoses ? 0 : UndefinedIndexT;

    // parse all the other lines
    while(std::getline(infile, line))
    {
        std::string imageName;
        Mat3 matK;
        Mat3 rotation;
        Vec3 translation;
        parseMiddleburyCamera(line, imageName, matK, rotation, translation);

        const auto imagePath = (bfs::path(basePath) / bfs::path(imageName)).string();
        int imageWidth{};
        int imageHeight{};
        image::readImageSize(imagePath, imageWidth, imageHeight);

        // if uniqueIntrinsics do it once, otherwise always
        if((uniqueIntrinsics && scene.intrinsics.empty()) || !uniqueIntrinsics)
        {
            ALICEVISION_LOG_DEBUG("matK " << matK);
            // add the intrinsics
            scene.intrinsics.insert({intrinsicsId, std::make_shared<camera::Pinhole>(imageWidth, imageHeight, matK)});
            if(lockIntrinsics)
            {
                scene.intrinsics[intrinsicsId]->lock();
            }
        }
        ALICEVISION_LOG_DEBUG("rotation " << rotation);
        ALICEVISION_LOG_DEBUG("translation " << translation);

        if(importPoses)
        {
            // add the pose entry
            const auto pose = geometry::poseFromRT(rotation, translation);
            scene.getPoses().insert({poseId, sfmData::CameraPose(pose, lockPoses)});
        }

        // add view
        scene.getViews().insert({viewId, std::make_shared<sfmData::View>(imagePath, viewId, intrinsicsId, poseId,
                                                                         imageWidth, imageHeight)});

        // update the intrinsics id only if not unique
        if(!uniqueIntrinsics)
        {
            ++intrinsicsId;
        }
        ++viewId;
        if(importPoses)
        {
            ++poseId;
        }
    }
    // just a safe guard
    if(scene.getViews().size() != numViews)
    {
        ALICEVISION_LOG_ERROR("Read " << scene.getViews().size() << " views, expected " << numViews);
        throw std::runtime_error("Unexpected number of cameras read");
    }
    ALICEVISION_LOG_INFO("Scene contains: " << scene.getIntrinsics().size() << " intrinsics, "
                                            << scene.getViews().size() << " views, " << scene.getPoses().size()
                                            << " poses");

    return scene;
}

}
}