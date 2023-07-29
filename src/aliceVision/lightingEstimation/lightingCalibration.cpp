// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "lightingCalibration.hpp"
#include "lightingEstimation.hpp"

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/convolution.hpp>
#include <aliceVision/photometricStereo/photometricStereo.hpp>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <math.h>

namespace fs = boost::filesystem;
namespace bpt = boost::property_tree;

namespace aliceVision {
namespace lightingEstimation {


void lightCalibration(const sfmData::SfMData& sfmData, const std::string& inputJSON, const std::string& outputPath,
                      const std::string& method, const bool saveAsModel)
{
    std::vector<std::string> imageList;
    std::vector<std::array<float, 3>> allSpheresParams;
    std::vector<float> focals;

    std::string inputJSONFullName = inputJSON + "/detection.json";

    // Main tree
    bpt::ptree fileTree;
    // Read the json file and initialize the tree
    bpt::read_json(inputJSONFullName, fileTree);

    std::map<std::string, sfmData::View> viewMap;
    for (auto& viewIt: sfmData.getViews())
    {
        std::map<std::string, std::string> currentMetadata = sfmData.getView(viewIt.first).getMetadata();
        viewMap[currentMetadata.at("Exif:DateTimeDigitized")] = sfmData.getView(viewIt.first);
    }

    for (const auto& [currentTime, currentView] : viewMap)
    {
        ALICEVISION_LOG_INFO("View Id: " << currentView.getViewId());
        const fs::path imagePath = fs::path(currentView.getImagePath());

        if (!boost::algorithm::icontains(imagePath.stem().string(), "ambiant"))
        {
            std::string sphereName = std::to_string(currentView.getViewId());
            auto sphereExists = (fileTree.get_child_optional(sphereName)).is_initialized();
            if (sphereExists)
            {
                ALICEVISION_LOG_INFO ("  - " << imagePath.string ());
                imageList.push_back (imagePath.string ());

                std::array<float, 3> currentSphereParams;

                for (auto& currentSphere: fileTree.get_child(sphereName))
                {
                    currentSphereParams[0] = currentSphere.second.get_child("").get("x", 0.0);
                    currentSphereParams[1] = currentSphere.second.get_child("").get("y", 0.0);
                    currentSphereParams[2] = currentSphere.second.get_child("").get("r", 0.0);
                }

                allSpheresParams.push_back(currentSphereParams);

                IndexT intrinsicId = currentView.getIntrinsicId();
                focals.push_back(sfmData.getIntrinsics().at(intrinsicId)->getParams().at(0));
            }
            else
            {
                ALICEVISION_LOG_WARNING ("No detected sphere found for '" << imagePath << "'.");
            }
        }
    }

    Eigen::MatrixXf lightMat(imageList.size(), 3);
    std::vector<float> intList;

    for (size_t i = 0; i < imageList.size(); ++i)
    {
        std::string picturePath = imageList.at(i);
        std::array<float,3> sphereParam = allSpheresParams.at(i);
        float focal = focals.at(i);

        Eigen::Vector3f lightingDirection;
        lightCalibrationOneImage(picturePath, sphereParam, focal, method, lightingDirection);
        lightMat.row(i) = lightingDirection;
        intList.push_back(lightingDirection.norm());
    }

    // Write in JSON file
    writeJSON(outputPath, sfmData, imageList, lightMat, intList, saveAsModel);
}

void lightCalibrationOneImage(const std::string& picturePath, const std::array<float, 3>& sphereParam,
                              const float focal, const std::string& method, Eigen::Vector3f& lightingDirection)
{
    // Read picture :
    image::Image<float> imageFloat;
    image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);

    // If method = brightest point :
    if (!method.compare("brightestPoint"))
    {
        // Detect brightest point :
        Eigen::Vector2f brigthestPoint;
        detectBrightestPoint(sphereParam, imageFloat, brigthestPoint);

        Eigen::Vector3f normalBrightestPoint;
        getNormalOnSphere(brigthestPoint(0), brigthestPoint(1), sphereParam, normalBrightestPoint);

        // Observation direction :
        Eigen::Vector3f observationRay;

        // orthographic approximation :
        observationRay(0) = 0.0;
        observationRay(1) = 0.0;
        observationRay(2) = -1.0;

        // Evaluate lighting direction :
        lightingDirection = 2 * normalBrightestPoint.dot(observationRay) * normalBrightestPoint - observationRay;
        lightingDirection = lightingDirection / lightingDirection.norm();
    }
    // If method = HS :
    else if (!method.compare("whiteSphere"))
    {
        // Evaluate light direction and intensity by pseudo-inverse
        const int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows() / 2);
        const int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols() / 2);

        const float radius = sphereParam[2];

        image::Image<float> patch;
        patch = imageFloat.block(minISphere, minJSphere, 2 * radius, 2 * radius);

        const int nbPixelsPatch = 4 * radius * radius;
        Eigen::VectorXf imSphere(nbPixelsPatch);
        Eigen::MatrixXf normalSphere(nbPixelsPatch, 3);

        int currentIndex = 0;

        for (int j = 0; j < patch.cols(); ++j)
        {
            for (int i = 0; i < patch.rows(); ++i)
            {
                float distanceToCenter = (i - radius) * (i - radius) + (j - radius) * (j - radius);
                if ((distanceToCenter < (radius * radius - 0.05 * radius) ) && (patch(i, j) > 0.3) && (patch(i, j) < 0.8))
                {
                    // imSphere = normalSphere.s
                    imSphere(currentIndex) = patch(i, j);

                    normalSphere(currentIndex, 0) = (float(j) - radius) / radius;
                    normalSphere(currentIndex, 1) = (float(i) - radius) / radius;
                    normalSphere(currentIndex, 2) =
                        -sqrt(1 - normalSphere(currentIndex, 0) * normalSphere(currentIndex, 0) -
                              normalSphere(currentIndex, 1) * normalSphere(currentIndex, 1));

                    ++currentIndex;
                }
            }
        }
        Eigen::MatrixXf normalSphereMasked(currentIndex, 3);
        normalSphereMasked = normalSphere.block(0, 0, currentIndex, 3);

        Eigen::VectorXf imSphereMasked(currentIndex);
        imSphereMasked = imSphere.head(currentIndex);
        lightingDirection = normalSphere.colPivHouseholderQr().solve(imSphere);
    }
}

void detectBrightestPoint(const std::array<float, 3>& sphereParam, const image::Image<float>& imageFloat,
                          Eigen::Vector2f& brigthestPoint)
{
    image::Image<float> patch;
    std::array<float, 2> patchOrigin;
    cutImage(imageFloat, sphereParam, patch, patchOrigin);

    image::Image<float> convolutedPatch1;
    image::Image<float> convolutedPatch2;

    // Create Kernel
    size_t kernelSize = round(sphereParam[2] / 20); // arbitrary
    Eigen::VectorXf kernel(2 * kernelSize + 1);
    createTriangleKernel(kernelSize, kernel);

    image::ImageVerticalConvolution(patch, kernel, convolutedPatch1);
    image::ImageHorizontalConvolution(convolutedPatch1, kernel, convolutedPatch2);

    Eigen::Index maxRow, maxCol;
    static_cast<void>(convolutedPatch2.maxCoeff(&maxRow, &maxCol));

    brigthestPoint(0) = maxCol + patchOrigin[0] - imageFloat.cols() / 2;
    brigthestPoint(1) = maxRow + patchOrigin[1] - imageFloat.rows() / 2;
}

void createTriangleKernel(const size_t kernelSize, Eigen::VectorXf& kernel)
{
    for (int i = 0; i < 2 * kernelSize + 1; ++i)
    {
        if (i > kernelSize)
        {
            kernel(i) = (1.0 + kernelSize - (i - kernelSize)) / kernelSize;
        }
        else
        {
            kernel(i) = (1.0 + i) / kernelSize;
        }
    }
}

void getNormalOnSphere(const float xPicture, const float yPicture, const std::array<float, 3>& sphereParam,
                       Eigen::Vector3f& currentNormal)
{
    currentNormal(0) = (xPicture - sphereParam[0]) / sphereParam[2];
    currentNormal(1) = (yPicture - sphereParam[1]) / sphereParam[2];
    currentNormal(2) = -sqrt(1 - currentNormal(0) * currentNormal(0) - currentNormal(1) * currentNormal(1));
}

void cutImage(const image::Image<float>& imageFloat, const std::array<float, 3>& sphereParam,
              image::Image<float>& patch, std::array<float, 2>& patchOrigin)
{
    const int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows()/2);
    const int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols()/2);

    patchOrigin[0] = minJSphere;
    patchOrigin[1] = minISphere;

    const int radius = round(sphereParam[2]);

    patch = imageFloat.block(minISphere, minJSphere, 2 * radius, 2 * radius);

    for (int i = 0; i < patch.rows(); ++i)
    {
        for (int j = 0; j < patch.cols(); ++j)
        {
            const float distanceToCenter = (i - patch.rows() / 2) * (i - patch.rows() / 2) +
                                           (j - patch.cols() / 2) * (j - patch.cols() / 2);
            if (distanceToCenter > radius * radius + 2)
            {
                patch(i, j) = 0;
            }
        }
    }
}

void writeJSON(const std::string& fileName, const sfmData::SfMData& sfmData, const std::vector<std::string>& imageList,
               const Eigen::MatrixXf& lightMat, const std::vector<float>& intList, const bool saveAsModel)
{
    bpt::ptree lightsTree;
    bpt::ptree fileTree;

    int imgCpt = 0;
    std::map<std::string, sfmData::View> viewMap;
    for (auto& viewIt: sfmData.getViews())
    {
        std::map<std::string, std::string> currentMetadata = sfmData.getView(viewIt.first).getMetadata();
        viewMap[currentMetadata.at("Exif:DateTimeDigitized")] = sfmData.getView(viewIt.first);
    }

    for (const auto& [currentTime, viewId]: viewMap)
    {
        const fs::path imagePath = fs::path(viewId.getImagePath());

        // The file may be in the input SfMData but may not have been calibrated: in that case, it is not in imageList
        const bool calibratedFile = (std::find(imageList.begin(), imageList.end(), viewId.getImagePath()) != imageList.end());

        // Only write images that were actually used for the lighting calibration, instead of all the input images
        if (!boost::algorithm::icontains(imagePath.stem().string(), "ambiant") && calibratedFile)
        {
            bpt::ptree lightTree;
            if (saveAsModel)
            {
                lightTree.put("lightId", imgCpt);
                lightTree.put("type", "directional");
            }
            else
            {
                //const IndexT index = viewId.getViewId
                lightTree.put("viewId", viewId.getViewId());
                lightTree.put("type", "directional");
            }

            // Light direction
            bpt::ptree directionNode;
            for (int i = 0; i < 3; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(lightMat(imgCpt, i));
                directionNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("direction", directionNode);

            // Light intensity
            bpt::ptree intensityNode;
            for (unsigned int i = 0; i < 3; ++i)
            {
                bpt::ptree cell;
                cell.put_value<float>(intList.at(imgCpt));
                intensityNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("intensity", intensityNode);
            imgCpt++;

            lightsTree.push_back(std::make_pair("", lightTree));
        }
        else
        {
            ALICEVISION_LOG_INFO("'" << imagePath << "' is in the input SfMData but has not been used for the lighting " <<
                                 "calibration or contains 'ambiant' in its filename.");
        }
    }

    fileTree.add_child("lights", lightsTree);
    bpt::write_json(fileName, fileTree);
}

}
}
