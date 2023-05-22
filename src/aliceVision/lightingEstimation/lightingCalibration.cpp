// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "lightingCalibration.hpp"

//#include "augmentedNormals.hpp"
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

void lightCalibration(const std::string& inputPath, const std::string& outputPath)
{
    std::vector<std::string> imageList;
    std::string pictureFolder = inputPath + "PS_Pictures/";
    // getPicturesNames(pictureFolder, imageList);  ???

    // std::array<float, 3> sphereParam;
    // image::Image<float> sphereMask;

    // detectSphere(imageList, sphereParam, sphereMask);
    // lightCalibration(imageList, sphereParam, outputPath);
}

void lightCalibration(const sfmData::SfMData& sfmData, const std::string& inputJSON, const std::string& outputPath, const std::string& method)
{

    std::vector<std::string> imageList;
    std::vector<std::array<float, 3>> allSpheresParams;
    std::vector<float> focals;

    std::string inputJSONFullName = inputJSON + "/detection.json";

    // main tree
    bpt::ptree fileTree;
    // read the json file and initialize the tree
    bpt::read_json(inputJSONFullName, fileTree);

    for(auto& viewIt: sfmData.getViews())
    {
        ALICEVISION_LOG_INFO("View Id: " << viewIt.first);
        const fs::path imagePath = fs::path(sfmData.getView(viewIt.first).getImagePath());

        if(!boost::algorithm::icontains(imagePath.stem().string(), "ambiant"))
        {
            ALICEVISION_LOG_INFO("  - " << imagePath.string());
            imageList.push_back(imagePath.string());

            std::string sphereName = std::to_string(viewIt.second->getViewId());
            std::array<float,3> currentSphereParams;

            for (auto& currentSphere : fileTree.get_child(sphereName))
            {
                currentSphereParams[0] = currentSphere.second.get_child("").get("x",0.0);
                currentSphereParams[1] = currentSphere.second.get_child("").get("y",0.0);
                currentSphereParams[2] = currentSphere.second.get_child("").get("r",0.0);
            }

            allSpheresParams.push_back(currentSphereParams);

            IndexT intrinsicId = viewIt.second->getIntrinsicId();
            focals.push_back(sfmData.getIntrinsics().at(intrinsicId)->getParams().at(0));
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

    // Write in JSON file :
    writeJSON(outputPath, sfmData, lightMat, intList);

}

void lightCalibration(const std::vector<std::string>& imageList, const std::vector<std::array<float, 3>>& allSpheresParams, const std::string& jsonName, const std::vector<float>& focals, const std::string& method)
{
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

    // Write in JSON file :
    writeJSON(jsonName, imageList, lightMat, intList);
}

void lightCalibrationOneImage(const std::string& picturePath, const std::array<float, 3>& sphereParam, const float focal, const std::string& method, Eigen::Vector3f& lightingDirection)
{

    // Read picture :
    image::Image<float> imageFloat;
    image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);

    // If method = brightest point :
    if(!method.compare("brightestPoint"))
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
        lightingDirection = lightingDirection/lightingDirection.norm();
    }
    // if method = HS :
    else if(!method.compare("whiteSphere"))
    {
        // Evaluate light direction and intensity by pseudo-inverse
        int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows()/2);
        int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols()/2);

        float radius = sphereParam[2];

        image::Image<float> patch;
        patch = imageFloat.block(minISphere, minJSphere, 2*radius, 2*radius);

        int nbPixelsPatch = 4*radius*radius;
        Eigen::VectorXf imSphere(nbPixelsPatch);
        Eigen::MatrixXf normalSphere(nbPixelsPatch,3);

        int currentIndex = 0;

        for (size_t j = 0; j < patch.cols(); ++j)
        {
            for (size_t i = 0; i < patch.rows(); ++i)
            {
                float distanceToCenter = (i - radius)*(i - radius) + (j - radius)*(j - radius);
                if((distanceToCenter < (radius*radius - 0.05*radius) ) && (patch(i,j) > 0.3) && (patch(i,j) < 0.8))
                {
                    // imSphere = normalSphere.s
                    imSphere(currentIndex) = patch(i,j);

                    normalSphere(currentIndex,0) = (float(j) - radius)/radius;
                    normalSphere(currentIndex,1) = (float(i) - radius)/radius;
                    normalSphere(currentIndex,2) = -sqrt(1 - normalSphere(currentIndex,0)*normalSphere(currentIndex,0) - normalSphere(currentIndex,1)*normalSphere(currentIndex,1));

                    ++currentIndex;
                }
            }
        }
        Eigen::MatrixXf normalSphereMasked(currentIndex,3);
        normalSphereMasked = normalSphere.block(0, 0, currentIndex,3);

        Eigen::VectorXf imSphereMasked(currentIndex);
        imSphereMasked = imSphere.head(currentIndex);
        lightingDirection = normalSphere.colPivHouseholderQr().solve(imSphere);
    }


}

void detectBrightestPoint(const std::array<float, 3>& sphereParam, const image::Image<float>& imageFloat, Eigen::Vector2f& brigthestPoint)
{
    image::Image<float> patch;
    std::array<float, 2> patchOrigin;
    cutImage(imageFloat, sphereParam, patch, patchOrigin);

    image::Image<float> convonlutedPatch1;
    image::Image<float> convonlutedPatch2;

    // Create Kernel :
    size_t kernelSize = round(sphereParam[2]/20); //arbitrary
    Eigen::VectorXf kernel(2*kernelSize+1);
    createTriangleKernel(kernelSize, kernel);

    image::ImageVerticalConvolution(patch, kernel, convonlutedPatch1);
    image::ImageHorizontalConvolution(convonlutedPatch1, kernel, convonlutedPatch2);

    Eigen::Index maxRow, maxCol;
    float max = convonlutedPatch2.maxCoeff(&maxRow, &maxCol);

    brigthestPoint(0) = maxCol + patchOrigin[0] - imageFloat.cols()/2;
    brigthestPoint(1) = maxRow + patchOrigin[1] - imageFloat.rows()/2;
}

void createTriangleKernel(const size_t kernelSize, Eigen::VectorXf& kernel)
{
    for(int i = 0; i < 2*kernelSize+1; ++i)
    {
        if(i > kernelSize)
        {
            kernel(i) = (1.0 + kernelSize - (i-kernelSize))/kernelSize;
        }
        else
        {
            kernel(i) = (1.0 + i)/kernelSize;
        }
    }
}

void getNormalOnSphere(const float x_picture, const float y_picture, const std::array<float, 3>& sphereParam, Eigen::Vector3f& currentNormal)
{
    currentNormal(0) = (x_picture - sphereParam[0])/sphereParam[2];
    currentNormal(1) = (y_picture - sphereParam[1])/sphereParam[2];
    currentNormal(2) = -sqrt(1 - currentNormal(0)*currentNormal(0) - currentNormal(1)*currentNormal(1));
}


void cutImage(const image::Image<float>& imageFloat, const std::array<float, 3>& sphereParam, image::Image<float>& patch, std::array<float, 2>& patchOrigin)
{
    int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows()/2);
    int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols()/2);

    patchOrigin[0] = minJSphere;
    patchOrigin[1] = minISphere;

    int radius = round(sphereParam[2]);

    patch = imageFloat.block(minISphere, minJSphere, 2*radius, 2*radius);

    for (size_t i = 0; i < patch.rows(); ++i)
    {
        for (size_t j = 0; j < patch.cols(); ++j)
        {
            float distanceToCenter = (i - patch.rows()/2)*(i - patch.rows()/2) + (j - patch.cols()/2)*(j - patch.cols()/2);
            if(distanceToCenter > radius*radius + 2)
            {
                patch(i,j) = 0;
            }
        }
    }
}

void writeJSON(const std::string& fileName, const std::vector<std::string>& imageList, const Eigen::MatrixXf& lightMat, const std::vector<float>& intList)
{
    // main tree
    bpt::ptree fileTree;
    bpt::ptree lights_node;

    int imgCpt = 0;

    for(auto& currentImPath: imageList)
    {
        // get lights name
        fs::path imagePathFS = fs::path(currentImPath);
        std::string lightName = imagePathFS.stem().string();

        bpt::ptree currentLight_node;
        currentLight_node.put("type", "directionnal");

        // Light direction
        bpt::ptree direction_node;
        for (int i = 0; i < 3; i++)
        {
            bpt::ptree cell;
            cell.put_value<float>(lightMat(imgCpt, i));
            direction_node.push_back(std::make_pair("", cell));
         }
         currentLight_node.add_child("direction", direction_node);

        // Light intensity
        bpt::ptree intensity_node;
        for (int i = 0; i < 3; i++)
        {
            bpt::ptree cell;
            cell.put_value<float>(intList.at(imgCpt));
            intensity_node.push_back(std::make_pair("", cell));
         }
         currentLight_node.add_child("intensity", intensity_node);
         imgCpt++;

        lights_node.add_child(lightName, currentLight_node);
    }

    fileTree.add_child("lights", lights_node);
    bpt::write_json(fileName, fileTree);
}

}
}
