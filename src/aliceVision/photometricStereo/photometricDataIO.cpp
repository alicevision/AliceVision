// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "photometricDataIO.hpp"

#include <aliceVision/image/io.hpp>
#include <aliceVision/utils/filesIO.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <boost/algorithm/string.hpp>

#include <iostream>
#include <sstream>
#include <filesystem>
#include <fstream>

namespace bpt = boost::property_tree;
namespace fs = std::filesystem;

namespace aliceVision {
namespace photometricStereo {

void loadLightIntensities(const std::string& intFileName, std::vector<std::array<float, 3>>& intList)
{
    std::stringstream stream;
    std::string line;
    float x, y, z;

    std::fstream intFile;
    intFile.open(intFileName, std::ios::in);

    if (!intFile.is_open())
    {
        ALICEVISION_LOG_ERROR("Unable to load intensities");
        ALICEVISION_THROW_ERROR("Cannot open '" << intFileName << "'!");
    }
    else
    {
        while (!intFile.eof())
        {
            std::getline(intFile, line);
            stream.clear();
            stream.str(line);

            stream >> x >> y >> z;
            std::array<float, 3> v = {x, y, z};
            intList.push_back(v);
        }
        intList.pop_back();
        intFile.close();
    }
}

void loadLightDirections(const std::string& dirFileName, const Eigen::MatrixXf& convertionMatrix, Eigen::MatrixXf& lightMat)
{
    std::stringstream stream;
    std::string line;
    float x, y, z;

    std::fstream dirFile;
    dirFile.open(dirFileName, std::ios::in);

    if (!dirFile.is_open())
    {
        ALICEVISION_LOG_ERROR("Unable to load lightings");
        ALICEVISION_THROW_ERROR("Cannot open '" + dirFileName + "'!");
    }
    else
    {
        int lineNumber = 0;

        while (!dirFile.eof())
        {
            getline(dirFile, line);
            stream.clear();
            stream.str(line);

            stream >> x >> y >> z;

            if (lineNumber < lightMat.rows())
            {
                lightMat(lineNumber, 0) = convertionMatrix(0, 0) * x + convertionMatrix(0, 1) * y + convertionMatrix(0, 2) * z;
                lightMat(lineNumber, 1) = convertionMatrix(1, 0) * x + convertionMatrix(1, 1) * y + convertionMatrix(1, 2) * z;
                lightMat(lineNumber, 2) = convertionMatrix(2, 0) * x + convertionMatrix(2, 1) * y + convertionMatrix(2, 2) * z;
                ++lineNumber;
            }
        }
        dirFile.close();
    }
}

void loadLightHS(const std::string& dirFileName, Eigen::MatrixXf& lightMat)
{
    std::stringstream stream;
    std::string line;
    float x, y, z, ambiant, nxny, nxnz, nynz, nx2ny2, nz2;

    std::fstream dirFile;
    dirFile.open(dirFileName, std::ios::in);

    if (!dirFile.is_open())
    {
        ALICEVISION_LOG_ERROR("Unable to load lightings");
        ALICEVISION_THROW_ERROR("Cannot open '" + dirFileName + "'!");
    }
    else
    {
        int lineNumber = 0;

        while (!dirFile.eof())
        {
            getline(dirFile, line);
            stream.clear();
            stream.str(line);

            stream >> x >> y >> z >> ambiant >> nxny >> nxnz >> nynz >> nx2ny2 >> nz2;
            if (lineNumber < lightMat.rows())
            {
                lightMat(lineNumber, 0) = x;
                lightMat(lineNumber, 1) = -y;
                lightMat(lineNumber, 2) = -z;
                lightMat(lineNumber, 3) = ambiant;
                lightMat(lineNumber, 4) = nxny;
                lightMat(lineNumber, 5) = nxnz;
                lightMat(lineNumber, 6) = nynz;
                lightMat(lineNumber, 7) = nx2ny2;
                lightMat(lineNumber, 8) = nz2;
                ++lineNumber;
            }
        }
        dirFile.close();
    }
}

void buildLightMatFromJSON(const std::string& fileName,
                           const std::vector<std::string>& imageList,
                           Eigen::MatrixXf& lightMat,
                           std::vector<std::array<float, 3>>& intList)
{
    // Main tree
    bpt::ptree fileTree;

    // Read the JSON file and initialize the tree
    bpt::read_json(fileName, fileTree);

    int lineNumber = 0;
    for (auto& currentImPath : imageList)
    {
        int cpt = 0;
        for (auto& lightsName : fileTree.get_child("lights"))
        {
            fs::path imagePathFS = fs::path(currentImPath);
            if (boost::algorithm::icontains(imagePathFS.stem().string(), lightsName.first))
            {
                std::array<float, 3> currentIntensities;
                for (auto& intensities : lightsName.second.get_child("intensity"))
                {
                    currentIntensities[cpt] = intensities.second.get_value<float>();
                    ++cpt;
                }
                intList.push_back(currentIntensities);

                cpt = 0;

                for (auto& direction : lightsName.second.get_child("direction"))
                {
                    lightMat(lineNumber, cpt) = direction.second.get_value<float>();
                    ++cpt;
                }
                ++lineNumber;
            }
        }
    }
}

void buildLightMatFromJSON(const std::string& fileName,
                           const std::vector<IndexT>& indices,
                           Eigen::MatrixXf& lightMat,
                           std::vector<std::array<float, 3>>& intList)
{
    // Main tree
    bpt::ptree fileTree;

    // Read the JSON file and initialize the tree
    bpt::read_json(fileName, fileTree);

    int lineNumber = 0;
    for (auto& currentIndex : indices)
    {
        int cpt = 0;
        for (auto& light : fileTree.get_child("lights"))
        {
            if (lineNumber == lightMat.rows())
                break;

            IndexT lightIndex = light.second.get<IndexT>("lightId", UndefinedIndexT);
            if (lightIndex != UndefinedIndexT)
            {
                std::array<float, 3> currentIntensities;
                for (auto& intensities : light.second.get_child("intensity"))
                {
                    currentIntensities[cpt] = intensities.second.get_value<float>();
                    ++cpt;
                }
                intList.push_back(currentIntensities);
                cpt = 0;
                for (auto& direction : light.second.get_child("direction"))
                {
                    if (cpt < 3)
                    {
                        lightMat(lineNumber, cpt) = direction.second.get_value<float>();
                    }
                    else if (cpt == 4)
                    {
                        // no support for SH lighting :
                        for (int pictureIndex = 0; pictureIndex < lightMat.rows(); ++pictureIndex)
                        {
                            lightMat.row(pictureIndex) = lightMat.row(pictureIndex) / lightMat.row(pictureIndex).norm();
                        }
                        ALICEVISION_LOG_INFO("SH will soon be available for use in PS. For now, lighting is reduced to directional");
                    }
                    ++cpt;
                }
                ++lineNumber;
            }
            else
            {
                IndexT lightIndex = light.second.get<IndexT>("viewId", UndefinedIndexT);
                if (lightIndex == currentIndex)
                {
                    std::array<float, 3> currentIntensities;
                    for (auto& intensities : light.second.get_child("intensity"))
                    {
                        currentIntensities[cpt] = intensities.second.get_value<float>();
                        ++cpt;
                    }
                    intList.push_back(currentIntensities);

                    cpt = 0;

                    for (auto& direction : light.second.get_child("direction"))
                    {
                        if (cpt < 3)
                        {
                            lightMat(lineNumber, cpt) = direction.second.get_value<float>();
                        }
                        else if (cpt == 4)
                        {
                            // no support for SH lighting :
                            for (int pictureIndex = 0; pictureIndex < lightMat.rows(); ++pictureIndex)
                            {
                                lightMat.row(pictureIndex) = lightMat.row(pictureIndex) / lightMat.row(pictureIndex).norm();
                            }
                            ALICEVISION_LOG_INFO("SH will soon be available for use in PS. For now, lighting is reduced to directional");
                        }
                        ++cpt;
                    }
                    ++lineNumber;
                    break;
                }
            }
        }
    }
}

void buildLightMatFromLP(const std::string& fileName,
                         const std::vector<std::string>& imageList,
                         Eigen::MatrixXf& lightMat,
                         std::vector<std::array<float, 3>>& intList)
{
    std::string pictureName;
    float x, y, z;

    int lineNumber = 0;
    std::array<float, 3> intensities = {1.0, 1.0, 1.0};

    for (auto& currentImPath : imageList)
    {
        std::stringstream stream;
        std::string line;
        std::fstream intFile;
        intFile.open(fileName, std::ios::in);

        if (!intFile.is_open())
        {
            ALICEVISION_LOG_ERROR("Unable to load Lp file");
            ALICEVISION_THROW_ERROR("Cannot open '" << fileName << "'!");
        }
        else
        {
            fs::path imagePathFS = fs::path(currentImPath);
            while (!intFile.eof())
            {
                std::getline(intFile, line);
                stream.clear();
                stream.str(line);

                stream >> pictureName >> x >> y >> z;

                std::string stringToCompare = imagePathFS.filename().string();

                if (boost::algorithm::iequals(pictureName, stringToCompare))
                {
                    lightMat(lineNumber, 0) = x;
                    lightMat(lineNumber, 1) = -y;
                    lightMat(lineNumber, 2) = -z;
                    ++lineNumber;

                    intList.push_back(intensities);

                    break;
                }
            }
            intFile.close();
        }
    }
}

void loadMask(std::string const& maskName, image::Image<float>& mask)
{
    if (maskName.empty() || !utils::exists(maskName))
    {
        if (maskName.empty())
        {
            ALICEVISION_LOG_INFO("No mask folder was provided. Every pixel will be used.");
        }
        else
        {
            ALICEVISION_LOG_WARNING("Could not open '" << maskName << "'. Every pixel will be used.");
        }
        Eigen::MatrixXf maskAux(1, 1);
        maskAux(0, 0) = 1;
        mask = maskAux;
    }
    else
    {
        image::readImage(maskName, mask, image::EImageColorSpace::SRGB);
    }
}

void getIndMask(image::Image<float> const& mask, std::vector<int>& indices)
{
    const int nbRows = mask.rows();
    const int nbCols = mask.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if (mask(i, j) > 0.7)
            {
                int currentIndex = j * nbRows + i;
                indices.push_back(currentIndex);
            }
        }
    }
}

void intensityScaling(std::array<float, 3> const& intensities, image::Image<image::RGBfColor>& imageToScale)
{
    int nbRows = imageToScale.rows();
    int nbCols = imageToScale.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            for (int ch = 0; ch < 3; ++ch)
            {
                imageToScale(i, j)(ch) /= intensities[ch];
            }
        }
    }
}

void image2PsMatrix(const image::Image<image::RGBfColor>& imageIn, const std::vector<int>& indices, Eigen::MatrixXf& imageOut)
{
    int nbRows = imageIn.rows();
    int nbCols = imageIn.cols();

    for (int iterator = 0; iterator < indices.size(); ++iterator)
    {
        int currentIdx = indices.at(iterator);
        int j = floor(currentIdx / nbRows);
        int i = currentIdx - j * nbRows;

        for (int ch = 0; ch < 3; ++ch)
        {
            imageOut(ch, iterator) = imageIn(i, j)(ch);
        }
    }
}

void image2PsMatrix(const image::Image<image::RGBfColor>& imageIn, const image::Image<float>& mask, Eigen::MatrixXf& imageOut)
{
    int nbRows = imageIn.rows();
    int nbCols = imageIn.cols();
    int index = 0;

    bool hasMask = !((mask.rows() == 1) && (mask.cols() == 1));

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if ((!hasMask) || mask(i, j) > 0.7)
            {
                for (int ch = 0; ch < 3; ++ch)
                {
                    imageOut(ch, index) = imageIn(i, j)(ch);
                }
                ++index;
            }
        }
    }
}

void image2PsMatrix(const image::Image<float>& imageIn, const image::Image<float>& mask, Eigen::VectorXf& imageOut)
{
    int nbRows = imageIn.rows();
    int nbCols = imageIn.cols();
    int index = 0;

    bool hasMask = !((mask.rows() == 1) && (mask.cols() == 1));

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if ((!hasMask) || mask(i, j) > 0.7)
            {
                imageOut(index) = imageIn(i, j);
            }
            ++index;
        }
    }
}

void reshapeInImage(const Eigen::MatrixXf& matrixIn, image::Image<image::RGBfColor>& imageOut)
{
    int nbRows = imageOut.rows();
    int nbCols = imageOut.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            int currentInd = j * nbRows + i;
            for (int ch = 0; ch < 3; ++ch)
            {
                imageOut(i, j)(ch) = matrixIn(ch, currentInd);
            }
        }
    }
}

void convertNormalMap2png(const image::Image<image::RGBfColor>& normalsIm, image::Image<image::RGBColor>& normalsImPNG)
{
    int nbRows = normalsIm.rows();
    int nbCols = normalsIm.cols();

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if (normalsIm(i, j)(0) * normalsIm(i, j)(0) + normalsIm(i, j)(1) * normalsIm(i, j)(1) + normalsIm(i, j)(2) * normalsIm(i, j)(2) == 0)
            {
                normalsImPNG(i, j)(0) = 0;
                normalsImPNG(i, j)(1) = 0;
                normalsImPNG(i, j)(2) = 0;
            }
            else
            {
                normalsImPNG(i, j)(0) = floor(255 * (normalsIm(i, j)(0) + 1) / 2);
                normalsImPNG(i, j)(1) = -floor(255 * (normalsIm(i, j)(1) + 1) / 2);
                normalsImPNG(i, j)(2) = -floor(255 * normalsIm(i, j)(2));
            }
        }
    }
}

void readMatrix(const std::string& fileName, Eigen::MatrixXf& matrix)
{
    int nbRows = matrix.rows();
    int nbCols = matrix.cols();

    std::fstream matFile;
    matFile.open(fileName, std::ios::in);

    if (matFile.is_open())
    {
        for (int row = 0; row < nbRows; row++)
        {
            for (int col = 0; col < nbCols; col++)
            {
                float item = 0.0;
                matFile >> item;
                matrix(row, col) = item;
            }
        }
    }
    matFile.close();
}

void writePSResults(const std::string& outputPath, const image::Image<image::RGBfColor>& normals, const image::Image<image::RGBfColor>& albedo)
{
    image::writeImage(
      outputPath + "/normals.exr",
      normals,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));

    image::Image<image::RGBColor> normalsImPNG(normals.cols(), normals.rows());
    convertNormalMap2png(normals, normalsImPNG);
    image::writeImage(
      outputPath + "/normals.png",
      normalsImPNG,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));

    image::writeImage(
      outputPath + "/albedo.exr",
      albedo,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
}

void writePSResults(const std::string& outputPath,
                    const image::Image<image::RGBfColor>& normals,
                    const image::Image<image::RGBfColor>& albedo,
                    const IndexT poseId)
{
    image::writeImage(
      outputPath + "/" + std::to_string(poseId) + "_normals.exr",
      normals,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));

    image::Image<image::RGBColor> normalsImPNG(normals.cols(), normals.rows());
    convertNormalMap2png(normals, normalsImPNG);
    image::writeImage(
      outputPath + "/" + std::to_string(poseId) + "_normals.png",
      normalsImPNG,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));

    image::writeImage(
      outputPath + "/" + std::to_string(poseId) + "_albedo.exr",
      albedo,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
}

}  // namespace photometricStereo
}  // namespace aliceVision
