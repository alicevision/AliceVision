// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "photometricStereo.hpp"
#include "photometricDataIO.hpp"

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/imageAlgo.hpp>
#include <aliceVision/utils/filesIO.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>
#include <sstream>
#include <filesystem>
#include <fstream>
#include <algorithm>

namespace fs = std::filesystem;

namespace aliceVision {
namespace photometricStereo {

void photometricStereo(const std::string& inputPath,
                       const std::string& lightData,
                       const std::string& outputPath,
                       const PhotometricSteroParameters& PSParameters,
                       image::Image<image::RGBfColor>& normals,
                       image::Image<image::RGBfColor>& albedo)
{
    size_t dim = 3;
    if (PSParameters.SHOrder != 0)
    {
        ALICEVISION_LOG_INFO("SH will soon be available for use in PS. For now, lighting is reduced to directional");
    }

    std::vector<std::string> imageList;
    std::string pictureFolder = inputPath + "/PS_Pictures/";
    getPicturesNames(pictureFolder, imageList);

    std::vector<std::array<float, 3>> intList;        // Light intensities
    Eigen::MatrixXf lightMat(imageList.size(), dim);  // Light directions

    if (fs::is_directory(lightData))
    {
        loadPSData(lightData, PSParameters.SHOrder, intList, lightMat);
    }
    else
    {
        buildLightMatFromJSON(lightData, imageList, lightMat, intList);
    }

    image::Image<float> mask;
    fs::path lightDataPath = fs::path(lightData);
    std::string maskName = lightDataPath.remove_filename().string() + "/mask.png";
    loadMask(maskName, mask);

    std::string pathToAmbiant = "";

    if (PSParameters.removeAmbiant)
    {
        for (auto& currentPath : imageList)
        {
            const fs::path imagePath = fs::path(currentPath);
            if (boost::algorithm::icontains(imagePath.stem().string(), "ambiant"))
            {
                pathToAmbiant = imagePath.string();
            }
        }
    }

    photometricStereo(imageList, intList, lightMat, mask, pathToAmbiant, PSParameters, normals, albedo);

    writePSResults(outputPath, normals, albedo);
    image::writeImage(outputPath + "/mask.png", mask, image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION));
}

void photometricStereo(const sfmData::SfMData& sfmData,
                       const std::string& lightData,
                       const std::string& maskPath,
                       const std::string& outputPath,
                       const PhotometricSteroParameters& PSParameters,
                       image::Image<image::RGBfColor>& normals,
                       image::Image<image::RGBfColor>& albedo)
{
    bool skipAll = true;
    bool groupedImages = false;
    size_t dim = 3;
    if (PSParameters.SHOrder != 0)
    {
        ALICEVISION_LOG_INFO("SH will soon be available for use in PS. For now, lighting is reduced to directional");
    }

    std::string pathToAmbiant = "";
    std::map<IndexT, std::vector<IndexT>> viewsPerPoseId;

    for (auto& viewIt : sfmData.getViews())
    {
        viewsPerPoseId[viewIt.second->getPoseId()].push_back(viewIt.second->getViewId());
    }

    for (auto& posesIt : viewsPerPoseId)
    {
        std::vector<std::string> imageList;

        ALICEVISION_LOG_INFO("Pose Id: " << posesIt.first);
        std::vector<IndexT>& initViewIds = posesIt.second;

        bool hasMetadata = true;
        std::map<std::string, std::string> metadataTest = sfmData.getView(initViewIds.at(0)).getImage().getMetadata();
        if (metadataTest.find("Exif:DateTimeDigitized") == metadataTest.end())
            hasMetadata = false;

        std::vector<IndexT> viewIds;
        std::map<std::string, IndexT> idMap;

        if (hasMetadata)
        {
            for (auto& viewId : initViewIds)
            {
                std::map<std::string, std::string> currentMetadata = sfmData.getView(viewId).getImage().getMetadata();
                idMap[currentMetadata.at("Exif:DateTimeDigitized")] = viewId;
            }

            for (const auto& [currentTime, viewId] : idMap)
            {
                viewIds.push_back(viewId);
            }
        }
        else
        {
            for (auto& viewId : initViewIds)
            {
                const fs::path imagePath = fs::path(sfmData.getView(viewId).getImage().getImagePath());
                idMap[imagePath] = viewId;
            }
        }
        for (const auto& [currentId, viewId] : idMap)
        {
            viewIds.push_back(viewId);
        }

        for (auto& viewId : viewIds)
        {
            const fs::path imagePath = fs::path(sfmData.getView(viewId).getImage().getImagePath());
            if (!boost::algorithm::icontains(imagePath.stem().string(), "ambiant"))
            {
                ALICEVISION_LOG_INFO(" - " << imagePath.string());
                imageList.push_back(imagePath.string());
            }
            else if (PSParameters.removeAmbiant)
            {
                ALICEVISION_LOG_INFO("Remove ambiant light - " << imagePath.string());
                pathToAmbiant = imagePath.string();
            }
        }

        std::vector<std::array<float, 3>> intList;        // Light intensities
        Eigen::MatrixXf lightMat(imageList.size(), dim);  // Light directions

        if (fs::is_directory(lightData))  // #pragma omp parallel for
        {
            loadPSData(lightData, PSParameters.SHOrder, intList, lightMat);
        }
        else
        {
            const std::string extension = fs::path(lightData).extension();

            if (extension == ".json")  // JSON File
            {
                buildLightMatFromJSON(lightData, viewIds, lightMat, intList);
            }
            else if (extension == ".lp")
            {
                buildLightMatFromLP(lightData, imageList, lightMat, intList);
            }
        }

        /* Ensure that there are as many images as light calibrations, and that the list of input images is not empty.
         * If there is a mismatch between the sizes of the list of images and the list of intensities, this means that there was
         * no light calibration for some images. Either the image had no sphere, or something went wrong during the sphere detection
         * or light calibration stages. */
        if (imageList.size() != intList.size() || imageList.size() < 1)
        {
            if (imageList.size() != intList.size())
            {
                ALICEVISION_LOG_WARNING("Mismatch between the number of images and the number of light intensities ("
                                        << imageList.size() << " images, " << intList.size() << " light intensities). This might happen "
                                        << "when no sphere was detected for an image in the list, and thus light was not calibrated for it.");
                ALICEVISION_LOG_WARNING("Skipping iteration.");
            }
            else
            {
                ALICEVISION_LOG_WARNING("Empty list, skipping iteration.");
            }
            continue;
        }
        skipAll = false;
        groupedImages = imageList.size() > 1 ? true : false;

        image::Image<float> mask;
        std::string pictureFolderName = fs::path(sfmData.getView(viewIds[0]).getImage().getImagePath()).parent_path().filename().string();
        // If no mask folder was provided, do not make up a path anyway
        std::string currentMaskPath = maskPath.empty() ? maskPath : maskPath + "/" + pictureFolderName.erase(0, 3) + ".png";

        loadMask(currentMaskPath, mask);

        photometricStereo(imageList, intList, lightMat, mask, pathToAmbiant, PSParameters, normals, albedo);

        writePSResults(outputPath, normals, albedo, posesIt.first);
        image::writeImage(outputPath + "/" + std::to_string(posesIt.first) + "_mask.png",
                          mask,
                          image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION));

        if (sfmData.getPoses().size() > 0)
        {
            const Mat3 rotation = sfmData.getPose(sfmData.getView(viewIds[0])).getTransform().rotation().transpose();
            applyRotation(rotation, normals);
        }

        image::writeImage(
          outputPath + "/" + std::to_string(posesIt.first) + "_normals_w.exr",
          normals,
          image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
    }

    if (skipAll)
    {
        ALICEVISION_LOG_ERROR("All images were skipped and no photometric stereo could be processed. This might happen if no sphere has "
                              "been detected in any of the input images, or if the light could not be calibrated.");
        ALICEVISION_THROW(std::invalid_argument, "No image was processed for the photometric stereo.");
    }

    if (!groupedImages)
    {
        ALICEVISION_LOG_WARNING("No images shared the same pose ID. "
                                << "Input images need to be located in a folder that starts with 'ps_' to be grouped together using their "
                                   "pose ID. The photometric stereo cannot run otherwise.");
    }

    sfmData::SfMData albedoSfmData = sfmData;

    std::set<IndexT> viewIdsToRemove;
    // Create Albedo SfmData
    for (auto& viewIt : albedoSfmData.getViews())
    {
        const IndexT viewId = viewIt.first;
        IndexT poseId = viewIt.second->getPoseId();

        if (viewId == poseId)
        {
            sfmData::View* view = albedoSfmData.getViews().at(viewId).get();
            std::string imagePath = outputPath + "/" + std::to_string(poseId) + "_albedo.exr";
            view->getImage().setImagePath(imagePath);
        }
        else
        {
            viewIdsToRemove.insert(viewId);
        }
    }
    for (auto r : viewIdsToRemove)
        albedoSfmData.getViews().erase(r);

    sfmDataIO::save(
      albedoSfmData, outputPath + "/albedoMaps.sfm", sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS));

    // Create Normal SfmData
    sfmData::SfMData normalSfmData = albedoSfmData;
    for (auto& viewIt : normalSfmData.getViews())
    {
        const IndexT viewId = viewIt.first;
        IndexT poseId = viewIt.second->getPoseId();

        sfmData::View* view = normalSfmData.getViews().at(viewId).get();
        std::string imagePath = outputPath + "/" + std::to_string(poseId) + "_normals_w.exr";
        view->getImage().setImagePath(imagePath);
    }

    sfmDataIO::save(
      normalSfmData, outputPath + "/normalMaps.sfm", sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS));
}

void photometricStereo(const std::vector<std::string>& imageList,
                       const std::vector<std::array<float, 3>>& intList,
                       const Eigen::MatrixXf& lightMat,
                       image::Image<float>& mask,
                       const std::string& pathToAmbiant,
                       const PhotometricSteroParameters& PSParameters,
                       image::Image<image::RGBfColor>& normals,
                       image::Image<image::RGBfColor>& albedo)
{
    size_t maskSize;
    int pictRows;
    int pictCols;

    bool hasMask = !((mask.rows() == 1) && (mask.cols() == 1));

    std::string picturePath;
    std::vector<int> indices;

    const float sizeMax = 3e9;

    if (hasMask)
    {
        if (PSParameters.downscale > 1)
        {
            imageAlgo::resizeImage(PSParameters.downscale, mask);
        }

        getIndMask(mask, indices);
        maskSize = indices.size();
        pictRows = mask.rows();
        pictCols = mask.cols();
    }
    else
    {
        picturePath = imageList.at(0);

        image::Image<image::RGBfColor> imageFloat;
        image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);

        if (PSParameters.downscale > 1)
        {
            imageAlgo::resizeImage(PSParameters.downscale, imageFloat);
        }

        pictRows = imageFloat.rows();
        pictCols = imageFloat.cols();

        maskSize = pictRows * pictCols;

        for (int i = 0; i < maskSize; ++i)
            indices.push_back(i);

        hasMask = true;
    }

    // Read ambiant
    image::Image<image::RGBfColor> imageAmbiant;

    if (boost::algorithm::icontains(fs::path(pathToAmbiant).stem().string(), "ambiant"))
    {
        ALICEVISION_LOG_INFO("Removing ambiant light");
        ALICEVISION_LOG_INFO(pathToAmbiant);

        image::readImage(pathToAmbiant, imageAmbiant, image::EImageColorSpace::NO_CONVERSION);

        if (PSParameters.downscale > 1)
        {
            imageAlgo::resizeImage(PSParameters.downscale, imageAmbiant);
        }
    }

    // Tiling
    int auxMaskSize = maskSize;
    int numberOfPixels = auxMaskSize * 3;

    int numberOfMasks = 1;

    while (numberOfPixels > (sizeMax / imageList.size()))
    {
        numberOfMasks = numberOfMasks * 2;
        auxMaskSize = floor(auxMaskSize / 2);
        numberOfPixels = floor(numberOfPixels / 2);
        hasMask = true;
    }

    Eigen::MatrixXf normalsVect = Eigen::MatrixXf::Zero(3, pictRows * pictCols);
    Eigen::MatrixXf albedoVect = Eigen::MatrixXf::Zero(3, pictRows * pictCols);

    int remainingPixels = maskSize;
    std::vector<int> currentMaskIndices;

    for (int currentMaskIndex = 0; currentMaskIndex < numberOfMasks; ++currentMaskIndex)
    {
        int currentMaskSize;

        if (numberOfMasks == 1)
        {
            currentMaskSize = maskSize;
            currentMaskIndices.resize(currentMaskSize);
            currentMaskIndices = indices;
        }
        else
        {
            if (currentMaskIndex == numberOfMasks - 1)
            {
                currentMaskIndices.resize(remainingPixels);
                slice(indices, currentMaskIndex * auxMaskSize, remainingPixels, currentMaskIndices);
                currentMaskSize = remainingPixels;
            }
            else
            {
                currentMaskIndices.resize(auxMaskSize);
                slice(indices, currentMaskIndex * auxMaskSize, auxMaskSize, currentMaskIndices);
                remainingPixels = remainingPixels - auxMaskSize;
                currentMaskSize = auxMaskSize;
            }
        }

        Eigen::MatrixXf imMat(3 * imageList.size(), currentMaskSize);
        Eigen::MatrixXf imMat_gray(imageList.size(), currentMaskSize);

        for (size_t i = 0; i < imageList.size(); ++i)
        {
            picturePath = imageList.at(i);

            image::Image<image::RGBfColor> imageFloat;
            image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);

            if (PSParameters.downscale > 1)
            {
                imageAlgo::resizeImage(PSParameters.downscale, imageFloat);
            }

            if (boost::algorithm::icontains(fs::path(pathToAmbiant).stem().string(), "ambiant"))
            {
                imageFloat = imageFloat - imageAmbiant;
            }

            intensityScaling(intList.at(i), imageFloat);

            Eigen::MatrixXf currentPicture(3, currentMaskSize);
            image2PsMatrix(imageFloat, currentMaskIndices, currentPicture);

            imMat.block(3 * i, 0, 3, currentMaskSize) = currentPicture;
            imMat_gray.block(i, 0, 1, currentMaskSize) = currentPicture.block(0, 0, 1, currentMaskSize) * 0.2126 +
                                                         currentPicture.block(1, 0, 1, currentMaskSize) * 0.7152 +
                                                         currentPicture.block(2, 0, 1, currentMaskSize) * 0.0722;
        }

        Eigen::MatrixXf M_channel(lightMat.cols(), currentMaskSize);
        int currentIdx;

        if (PSParameters.isRobust)
        {
            float mu = 0.1;
            int max_iterations = 1000;
            float epsilon = 0.001;

            // Errors (E) and Lagrange multiplicators (W) initialisation
            Eigen::MatrixXf E = lightMat * M_channel - imMat_gray;
            Eigen::MatrixXf W = Eigen::MatrixXf::Zero(E.rows(), E.cols());

            Eigen::MatrixXf M_kminus1;
            Eigen::MatrixXf newImMat;

            for (size_t k = 0; k < max_iterations; ++k)
            {
                // Copy for convergence test
                M_kminus1 = M_channel;

                // M update
                newImMat = imMat_gray + E - W / mu;
                M_channel = lightMat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(newImMat);

                // E update
                Eigen::MatrixXf E_before = E;
                shrink(lightMat * M_channel - imMat_gray + W / mu, 1.0 / mu, E);

                // W update
                W = W + mu * (lightMat * M_channel - imMat_gray - E);

                // Convergence test
                Eigen::MatrixXf dev = M_kminus1 - M_channel;
                float relativeDev = dev.norm() / M_channel.norm();

                if (k > 10 && relativeDev < epsilon)
                {
                    ALICEVISION_LOG_INFO(k);
                    ALICEVISION_LOG_INFO("Convergence");
                    break;
                }
            }

            for (size_t i = 0; i < currentMaskSize; ++i)
            {
                if (hasMask)
                {
                    currentIdx = currentMaskIndices.at(i);  // Index in picture
                }
                else
                {
                    currentIdx = i;
                }
                normalsVect.col(currentIdx) = M_channel.col(i) / M_channel.col(i).norm();
            }

            int currentIdx;
            for (size_t ch = 0; ch < 3; ++ch)
            {
                // Create I matrix for current pixel
                Eigen::MatrixXf pixelValues_channel(imageList.size(), currentMaskSize);
                for (size_t i = 0; i < imageList.size(); ++i)
                {
                    pixelValues_channel.block(i, 0, 1, currentMaskSize) = imMat.block(ch + 3 * i, 0, 1, currentMaskSize);
                }

                for (size_t i = 0; i < currentMaskSize; ++i)
                {
                    if (hasMask)
                    {
                        currentIdx = currentMaskIndices.at(i);  // Index in picture
                    }
                    else
                    {
                        currentIdx = i;
                    }
                    Eigen::VectorXf currentI = pixelValues_channel.col(i);
                    Eigen::VectorXf currentShading = lightMat * normalsVect.col(currentIdx);
                    Eigen::VectorXf result = currentI.cwiseProduct(currentShading.cwiseInverse());
                    median(result, albedoVect(ch, currentIdx));
                }
            }
        }
        else
        {
            // Normal estimation
            M_channel = lightMat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(imMat_gray);

            for (size_t i = 0; i < currentMaskSize; ++i)
            {
                if (hasMask)
                {
                    currentIdx = currentMaskIndices.at(i);  // Index in picture
                }
                else
                {
                    currentIdx = i;
                }
                normalsVect.col(currentIdx) = M_channel.col(i) / M_channel.col(i).norm();
            }

            // Channelwise albedo estimation
            for (size_t ch = 0; ch < 3; ++ch)
            {
                // Create I matrix for current pixel
                Eigen::MatrixXf pixelValues_channel(imageList.size(), currentMaskSize);
                for (size_t i = 0; i < imageList.size(); ++i)
                {
                    pixelValues_channel.block(i, 0, 1, currentMaskSize) = imMat.block(ch + 3 * i, 0, 1, currentMaskSize);
                }

                M_channel = lightMat.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(pixelValues_channel);

                for (size_t i = 0; i < currentMaskSize; ++i)
                {
                    if (hasMask)
                    {
                        currentIdx = currentMaskIndices.at(i);  // Index in picture
                    }
                    else
                    {
                        currentIdx = i;
                    }
                    albedoVect(ch, currentIdx) = M_channel.col(i).norm();
                }
            }
        }
    }

    image::Image<image::RGBfColor> normalsIm(pictCols, pictRows);
    reshapeInImage(normalsVect, normalsIm);
    normals = normalsIm;

    image::Image<image::RGBfColor> albedoIm(pictCols, pictRows);
    albedoVect = albedoVect / albedoVect.maxCoeff();
    reshapeInImage(albedoVect, albedoIm);

    albedo = albedoIm;
}

void loadPSData(const std::string& folderPath, const size_t SH_order, std::vector<std::array<float, 3>>& intList, Eigen::MatrixXf& lightMat)
{
    std::string intFileName;
    std::string pathToCM;
    std::string dirFileName;

    // Light instensities
    intFileName = folderPath + "/light_intensities.txt";
    loadLightIntensities(intFileName, intList);

    // Convertion matrix
    Eigen::MatrixXf convertionMatrix = Eigen::Matrix<float, 3, 3>::Identity();
    pathToCM = folderPath + "/convertionMatrix.txt";
    if (utils::exists(pathToCM))
    {
        readMatrix(pathToCM, convertionMatrix);
    }

    // Light directions
    if (SH_order == 0)
    {
        dirFileName = folderPath + "/light_directions.txt";
        loadLightDirections(dirFileName, convertionMatrix, lightMat);
    }
    else if (SH_order == 2)
    {
        dirFileName = folderPath + "/light_directions_SH.txt";
        loadLightSH(dirFileName, lightMat);
    }
}

void getPicturesNames(const std::string& folderPath, std::vector<std::string>& imageList)
{
    const std::vector<std::string>& extensions = image::getSupportedExtensions();

    fs::directory_iterator endItr;
    for (fs::directory_iterator itr(folderPath); itr != endItr; ++itr)
    {
        fs::path currentFilePath = itr->path();

        std::string fileExtension = currentFilePath.extension().string();
        std::transform(fileExtension.begin(), fileExtension.end(), fileExtension.begin(), ::tolower);

        if (!boost::algorithm::icontains(currentFilePath.stem().string(), "mask") &&
            !boost::algorithm::icontains(currentFilePath.stem().string(), "ambiant"))
        {
            for (const std::string& extension : extensions)
            {
                if (fileExtension == extension)
                {
                    imageList.push_back(currentFilePath.string());
                }
            }
        }
    }

    std::sort(imageList.begin(), imageList.end(), compareFunction);  // Sort the vector
}

bool compareFunction(std::string a, std::string b) { return a < b; }

void shrink(const Eigen::MatrixXf& mat, const float rho, Eigen::MatrixXf& E)
{
    for (size_t i = 0; i < E.rows(); ++i)
    {
        for (size_t j = 0; j < E.cols(); ++j)
        {
            if (mat(i, j) > 0)
            {
                E(i, j) = std::max(std::abs(mat(i, j)) - rho, float(0.0));
            }
            else
            {
                E(i, j) = -std::max(std::abs(mat(i, j)) - rho, float(0.0));
            }
        }
    }
}

void median(const Eigen::MatrixXf& d, float& median)
{
    Eigen::MatrixXf aux = d;
    std::sort(aux.data(), aux.data() + aux.size());
    size_t middle = aux.size() / 2;
    aux.size() % 2 == 0 ? median = aux((aux.size() - 1) / 2) + aux((aux.size() + 1) / 2) : median = aux(middle);
}

void slice(const std::vector<int>& inputVector, int start, int numberOfElements, std::vector<int>& currentMaskIndices)
{
    auto first = inputVector.begin() + start;
    auto last = first + numberOfElements;

    copy(first, last, currentMaskIndices.begin());
}

void applyRotation(const Eigen::MatrixXd& rotation, image::Image<image::RGBfColor>& normals)
{
    for (int i = 0; i < normals.rows(); ++i)
    {
        for (int j = 0; j < normals.cols(); ++j)
        {
            normals(i, j)(0) = rotation(0, 0) * normals(i, j)(0) + rotation(0, 1) * normals(i, j)(1) + rotation(0, 2) * normals(i, j)(2);
            normals(i, j)(1) = rotation(1, 0) * normals(i, j)(0) + rotation(1, 1) * normals(i, j)(1) + rotation(1, 2) * normals(i, j)(2);
            normals(i, j)(2) = rotation(2, 0) * normals(i, j)(0) + rotation(2, 1) * normals(i, j)(1) + rotation(2, 2) * normals(i, j)(2);
        }
    }
}

}  // namespace photometricStereo
}  // namespace aliceVision
