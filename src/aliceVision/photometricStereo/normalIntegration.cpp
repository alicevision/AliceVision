// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/io.hpp>
#include <aliceVision/sfmData/Landmark.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/numeric/projection.hpp>
#include <aliceVision/numeric/numeric.hpp>

#include <iostream>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <math.h>

#include <Eigen/Dense>

#include <aliceVision/image/io.hpp>
#include <aliceVision/image/imageAlgo.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>

#include "photometricDataIO.hpp"
#include "normalIntegration.hpp"

namespace aliceVision {
namespace photometricStereo {

void normalIntegration(const std::string& inputPath, const bool& perspective, const int& downscale, const std::string& outputFolder)
{
    std::string normalMapPath = inputPath + "/normals.png";
    std::string pathToK = inputPath + "/K.txt";

    image::Image<image::RGBColor> normalsImPNG;
    image::readImage(normalMapPath, normalsImPNG, image::EImageColorSpace::NO_CONVERSION);

    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(3, 3);
    readMatrix(pathToK, K);

    image::Image<float> normalsMask;
    std::string maskName = inputPath + "/mask.png";
    loadMask(maskName, normalsMask);

    int nbCols = normalsImPNG.cols();
    int nbRows = normalsImPNG.rows();

    image::Image<image::RGBfColor> normalsImPNG2(nbCols, nbRows);
    loadNormalMap(normalsImPNG, normalsMask, normalsImPNG2);

    if (downscale > 1)
    {
        imageAlgo::resizeImage(downscale, normalsImPNG2);
        imageAlgo::resizeImage(downscale, normalsMask);

        K = K / downscale;
        K(2, 2) = 1;

        nbCols = normalsImPNG2.cols();
        nbRows = normalsImPNG2.rows();
    }

    image::Image<float> depthMap(nbCols, nbRows);
    image::Image<float> distanceMap(nbCols, nbRows);
    DCTIntegration(normalsImPNG2, depthMap, perspective, K, normalsMask);

    // AliceVision uses distance-to-origin convention
    convertZtoDistance(depthMap, distanceMap, K);

    std::string pathToDM = outputFolder + "/output.exr";

    image::writeImage(
      pathToDM,
      distanceMap,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
}

void normalIntegration(const sfmData::SfMData& sfmData,
                       const std::string& inputPath,
                       const bool& perspective,
                       const int& downscale,
                       const std::string& outputFolder)
{
    image::Image<image::RGBColor> normalsImPNG;

    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(3, 3);
    IndexT viewId;
    IndexT poseId;
    IndexT intrinsicId;

    if (sfmData.getPoses().size() > 0)
    {
        for (auto& poseIt : sfmData.getPoses())
        {
            // Read associated normal map :
            image::readImage(inputPath + "/" + std::to_string(poseIt.first) + "_normals.png", normalsImPNG, image::EImageColorSpace::NO_CONVERSION);

            int nbCols = normalsImPNG.cols();
            int nbRows = normalsImPNG.rows();

            // Find one view associated with the pose
            for (auto& viewIt : sfmData.getViews())
            {
                poseId = viewIt.second->getPoseId();
                if (poseId == poseIt.first)
                {
                    viewId = viewIt.first;
                    // Get intrinsics associated with this view :
                    intrinsicId = viewIt.second->getIntrinsicId();
                    const float focalPx = sfmData.getIntrinsics().at(intrinsicId)->getParams().at(0);
                    const float x_p = (nbCols) / 2 + sfmData.getIntrinsics().at(intrinsicId)->getParams().at(2);
                    const float y_p = (nbRows) / 2 + sfmData.getIntrinsics().at(intrinsicId)->getParams().at(3);

                    // Create K matrix
                    K(0, 0) = focalPx;
                    K(1, 1) = focalPx;
                    K(0, 2) = x_p;
                    K(1, 2) = y_p;
                    K(2, 2) = 1;

                    break;
                }
            }

            // PNG to real normal map
            image::Image<image::RGBfColor> normalsImPNG2(nbCols, nbRows);
            image::Image<float> normalsMask(nbCols, nbRows);

            for (int j = 0; j < nbCols; ++j)
            {
                for (int i = 0; i < nbRows; ++i)
                {
                    if (normalsImPNG(i, j)(0) != 0 || normalsImPNG(i, j)(1) != 0 || normalsImPNG(i, j)(2) != 0)
                    {
                        normalsMask(i, j) = 1.0;
                        for (int ch = 0; ch < 3; ++ch)
                        {
                            if (ch == 0)
                            {
                                normalsImPNG2(i, j)(ch) = normalsImPNG(i, j)(ch) / 127.5 - 1;
                            }
                            else
                            {
                                normalsImPNG2(i, j)(ch) = -(normalsImPNG(i, j)(ch) / 127.5 - 1);
                            }
                        }
                    }
                    else
                    {
                        normalsMask(i, j) = 0.0;

                        normalsImPNG2(i, j)(0) = 0;
                        normalsImPNG2(i, j)(1) = 0;
                        normalsImPNG2(i, j)(2) = -1;
                    }
                }
            }

            if (downscale > 1)
            {
                imageAlgo::resizeImage(downscale, normalsImPNG2);
                imageAlgo::resizeImage(downscale, normalsMask);

                K = K / downscale;
                K(2, 2) = 1;

                nbCols = normalsImPNG2.cols();
                nbRows = normalsImPNG2.rows();
            }
            // Main fonction
            image::Image<float> depthMap;

            aliceVision::image::Image<float> distanceMap;
            DCTIntegration(normalsImPNG2, depthMap, perspective, K, normalsMask);
            image::Image<float> z0(nbCols, nbRows);
            image::Image<float> maskZ0(nbCols, nbRows);
            getZ0FromLandmarks(sfmData, z0, maskZ0, viewId, normalsMask);

            // AliceVision uses distance-to-origin convention
            convertZtoDistance(depthMap, distanceMap, K);

            std::string pathToDM = outputFolder + "/" + std::to_string(poseIt.first) + "_depthMap.exr";

            // Create pose for metadata
            const geometry::Pose3 pose = poseIt.second.getTransform();
            std::shared_ptr<camera::IntrinsicBase> cam = sfmData.getIntrinsics().at(intrinsicId);
            std::shared_ptr<camera::Pinhole> camPinHole = std::dynamic_pointer_cast<camera::Pinhole>(cam);
            Mat34 P = camPinHole->getProjectiveEquivalent(pose);

            oiio::ParamValueList metadata;
            metadata.attribute("AliceypeDesc::DOUBLE, oiio::TypeDesVision:storageDataType",
                               image::EStorageDataType_enumToString(image::EStorageDataType::Float));
            metadata.push_back(oiio::ParamValue("AliceVision:P", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX44), 1, P.data()));
            image::writeImage(
              pathToDM,
              distanceMap,
              image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float),
              metadata);
        }
    }
    else
    {
        // All views are associated to the same pose :
        viewId = sfmData.getViews().begin()->first;
        poseId = sfmData.getViews().begin()->second->getPoseId();

        // Read associated normal map :
        image::readImage(inputPath + "/" + std::to_string(poseId) + "_normals.png", normalsImPNG, image::EImageColorSpace::NO_CONVERSION);

        int nbCols = normalsImPNG.cols();
        int nbRows = normalsImPNG.rows();

        if (perspective)
        {
            intrinsicId = sfmData.getViews().begin()->second->getIntrinsicId();
            // Get intrinsics associated with this view :
            const float focalPx = sfmData.getIntrinsics().at(intrinsicId)->getParams().at(0);
            const float x_p = (nbCols) / 2 + sfmData.getIntrinsics().at(intrinsicId)->getParams().at(2);
            const float y_p = (nbRows) / 2 + sfmData.getIntrinsics().at(intrinsicId)->getParams().at(3);

            // Create K matrix
            K(0, 0) = focalPx;
            K(1, 1) = focalPx;
            K(0, 2) = x_p;
            K(1, 2) = y_p;
            K(2, 2) = 1;
        }
        else
        {
            // Create K matrix
            K(0, 0) = 8000;
            K(1, 1) = 8000;
            K(0, 2) = nbCols / 2;
            K(1, 2) = nbRows / 2;
            K(2, 2) = 1;
        }

        image::Image<float> normalsMask(nbCols, nbRows);
        std::string maskName = inputPath + "/" + std::to_string(poseId) + "_mask.png";
        loadMask(maskName, normalsMask);

        // Float normal map
        image::Image<image::RGBfColor> normalsImPNG2(nbCols, nbRows);
        loadNormalMap(normalsImPNG, normalsMask, normalsImPNG2);

        if (downscale > 1)
        {
            imageAlgo::resizeImage(downscale, normalsImPNG2);
            imageAlgo::resizeImage(downscale, normalsMask);

            K = K / downscale;
            K(2, 2) = 1;

            nbCols = normalsImPNG2.cols();
            nbRows = normalsImPNG2.rows();
        }

        // Main fonction
        image::Image<float> depthMap(nbCols, nbRows);
        DCTIntegration(normalsImPNG2, depthMap, perspective, K, normalsMask);

        // AliceVision uses distance-to-origin convention
        image::Image<float> distanceMap(nbCols, nbRows);
        convertZtoDistance(depthMap, distanceMap, K);

        // TODO : Add fake metadata for display
        /*Matrix3x3 K_bis;
        K_bis.m11 = double(K(0,0));
        K_bis.m12 = double(0.0);
        K_bis.m13 = double(K(0,2));

        K_bis.m21 = double(0.0);
        K_bis.m22 = double(K(1,1));
        K_bis.m23 = double(K(1,2));

        K_bis.m31 = double(0.0);
        K_bis.m32 = double(0.0);
        K_bis.m33 = double(1.0);

        Matrix3x3 R;
        R.m11 = double(1.0);
        R.m12 = double(0.0);
        R.m13 = double(0.0);

        R.m21 = double(0.0);
        R.m22 = double(1.0);
        R.m23 = double(0.0);

        R.m31 = double(0.0);
        R.m32 = double(0.0);
        R.m33 = double(1.0);

        Matrix3x3 iP = R.inverse()*K_bis.inverse(); // or K_bis.inverse()*R.inverse() ?
        Point3d C = Point3d(0.0,0.0,0.0);

        oiio::ParamValueList metadata;
        metadata.push_back(oiio::ParamValue("AliceVision:CArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::VEC3), 1, C.m)); //C.data()));
        metadata.push_back(oiio::ParamValue("AliceVision:iCamArr", oiio::TypeDesc(oiio::TypeDesc::DOUBLE, oiio::TypeDesc::MATRIX33), 1, iP.m));  //
        iP.data()));
        */

        std::string pathToDM = outputFolder + "/" + std::to_string(poseId) + "_depthMap.exr";
        image::writeImage(
          pathToDM,
          distanceMap,
          image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
    }
}

void DCTIntegration(const image::Image<image::RGBfColor>& normals,
                    image::Image<float>& depth,
                    bool perspective,
                    const Eigen::Matrix3f& K,
                    const image::Image<float>& normalsMask)
{
    int nbCols = normals.cols();
    int nbRows = normals.rows();

    Eigen::MatrixXf p(nbRows, nbCols);
    Eigen::MatrixXf q(nbRows, nbCols);

    Eigen::MatrixXf f(nbRows, nbCols);

    // Prepare normal integration
    normal2PQ(normals, p, q, perspective, K, normalsMask);
    getDivergenceField(p, q, f);
    setBoundaryConditions(p, q, f);

    // Convert f to OpenCV matrix
    cv::Mat f_openCV(nbRows, nbCols, CV_32FC1);
    cv::eigen2cv(f, f_openCV);

    // Cosine transform of f
    cv::Mat fcos(nbRows, nbCols, CV_32FC1);
    cv::dct(f_openCV, fcos);

    // Cosine transform of z
    cv::Mat z_bar_bar(nbRows, nbCols, CV_32FC1);

    for (int j = 0; j < nbCols; j++)
    {
        for (int i = 0; i < nbRows; i++)
        {
            double denom = 4 * (pow(sin(0.5 * M_PI * j / nbCols), 2) + pow(sin(0.5 * M_PI * i / nbRows), 2));
            denom = std::max(denom, 0.0001);
            z_bar_bar.at<float>(i, j) = -fcos.at<float>(i, j) / denom;
        }
    }

    // Inverse cosine transform
    cv::Mat z(nbRows, nbCols, CV_32FC1);
    cv::idct(z_bar_bar, z);

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if (normalsMask(i, j) > 0.7)
            {
                if (perspective)
                {
                    depth(i, j) = -std::exp(z.at<float>(i, j));
                }
                else
                {
                    depth(i, j) = z.at<float>(i, j);
                }
            }
            else
            {
                depth(i, j) = nanf("1");
            }
        }
    }

    // Since we are not in a MV context, we can change the position of our depthmap
    // We can then get away from negative values
    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if (normalsMask(i, j) > 0.7)
            {
                depth(i, j) = depth(i, j) - depth(floor(nbRows / 2), floor(nbCols / 2)) + 10 * K(0, 0);
            }
            else
            {
                depth(i, j) = -1.0;
            }
        }
    }
}

void normal2PQ(const image::Image<image::RGBfColor>& normals,
               Eigen::MatrixXf& p,
               Eigen::MatrixXf& q,
               bool perspective,
               const Eigen::Matrix3f& K,
               const image::Image<float>& normalsMask)
{
    image::Image<float> normalsX(p.cols(), p.rows());
    image::Image<float> normalsY(p.cols(), p.rows());
    image::Image<float> normalsZ(p.cols(), p.rows());

    bool hasMask = !((normalsMask.rows() == 1) && (normalsMask.cols() == 1));

    for (size_t j = 0; j < p.cols(); ++j)
    {
        for (size_t i = 0; i < p.rows(); ++i)
        {
            normalsX(i, j) = normals(i, j)(0);
            normalsY(i, j) = normals(i, j)(1);
            normalsZ(i, j) = normals(i, j)(2);
        }
    }

    if (perspective)
    {
        float f = (K(0, 0) + K(1, 1)) / 2;

        for (size_t j = 0; j < p.cols(); ++j)
        {
            float u = j - K(0, 2);

            for (size_t i = 0; i < p.rows(); ++i)
            {
                float v = i - K(1, 2);

                float denom = std::max(-(u * normalsX(i, j) + v * normalsY(i, j) + f * normalsZ(i, j)), float(0.0001));
                if (hasMask && (normalsMask(i, j) < 0.3))
                {
                    p(i, j) = 0;
                    q(i, j) = 0;
                }
                else
                {
                    p(i, j) = -normalsX(i, j) / denom;
                    q(i, j) = -normalsY(i, j) / denom;
                }
            }
        }
    }
    else
    {
        for (size_t j = 0; j < p.cols(); ++j)
        {
            for (size_t i = 0; i < p.rows(); ++i)
            {
                if ((normalsZ(i, j) == 0) || (hasMask && (normalsMask(i, j) == 0)))
                {
                    p(i, j) = 0;
                    q(i, j) = 0;
                }
                else
                {
                    p(i, j) = -normalsX(i, j) / normalsZ(i, j);
                    q(i, j) = -normalsY(i, j) / normalsZ(i, j);
                }
            }
        }
    }
}

void getDivergenceField(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f)
{
    int nbRows = p.rows();
    int nbCols = p.cols();

    Eigen::MatrixXf qy_below(nbRows, nbCols);
    qy_below = q;
    qy_below.block(0, 0, nbRows - 1, nbCols) = q.block(1, 0, nbRows - 1, nbCols);

    Eigen::MatrixXf qy_above(nbRows, nbCols);
    qy_above = q;
    qy_above.block(1, 0, nbRows - 1, nbCols) = q.block(0, 0, nbRows - 1, nbCols);

    Eigen::MatrixXf qy = 0.5 * (qy_below - qy_above);

    Eigen::MatrixXf px_right = p;
    px_right.block(0, 0, nbRows, nbCols - 1) = p.block(0, 1, nbRows, nbCols - 1);

    Eigen::MatrixXf px_left = p;
    px_left.block(0, 1, nbRows, nbCols - 1) = p.block(0, 0, nbRows, nbCols - 1);

    Eigen::MatrixXf px = 0.5 * (px_right - px_left);

    // Div(p,q)
    f = px + qy;
}

void setBoundaryConditions(const Eigen::MatrixXf& p, const Eigen::MatrixXf& q, Eigen::MatrixXf& f)
{
    int nbRows = p.rows();
    int nbCols = p.cols();

    // Right hand side of the boundary condition
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(nbRows, nbCols);

    b.block(0, 1, 1, nbCols - 2) = -q.block(0, 1, 1, nbCols - 2);
    b.block(nbRows - 1, 1, 1, nbCols - 2) = q.block(nbRows - 1, 1, 1, nbCols - 2);
    b.block(1, 0, nbRows - 2, 1) = -p.block(1, 0, nbRows - 2, 1);
    b.block(1, nbCols - 1, nbRows - 2, 1) = p.block(1, nbCols - 1, nbRows - 2, 1);
    b(0, 0) = (1 / sqrt(2)) * (-q(0, 0) - p(0, 0));
    b(0, nbCols - 1) = (1 / sqrt(2)) * (-q(0, nbCols - 1) + p(0, nbCols - 1));
    b(nbRows - 1, nbCols - 1) = (1 / sqrt(2)) * (q(nbRows - 1, nbCols - 1) + p(nbRows - 1, nbCols - 1));
    b(nbRows - 1, 0) = (1 / sqrt(2)) * (q(nbRows - 1, 0) - p(nbRows - 1, 0));

    // Modification near the boundaries to enforce the non-homogeneous Neumann BC
    f.block(0, 1, 1, nbCols - 2) = f.block(0, 1, 1, nbCols - 2) - b.block(0, 1, 1, nbCols - 2);
    f.block(nbRows - 1, 1, 1, nbCols - 2) = f.block(nbRows - 1, 1, 1, nbCols - 2) - b.block(nbRows - 1, 1, 1, nbCols - 2);
    f.block(1, 0, nbRows - 2, 1) = f.block(1, 0, nbRows - 2, 1) - b.block(1, 0, nbRows - 2, 1);
    f.block(1, nbCols - 1, nbRows - 2, 1) = f.block(1, nbCols - 1, nbRows - 2, 1) - b.block(1, nbCols - 1, nbRows - 2, 1);

    // Modification near the corners
    f(0, 0) = f(0, 0) - sqrt(2) * b(0, 0);
    f(0, nbCols - 1) = f(0, nbCols - 1) - sqrt(2) * b(0, nbCols - 1);
    f(nbRows - 1, nbCols - 1) = f(nbRows - 1, nbCols - 1) - sqrt(2) * b(nbRows - 1, nbCols - 1);
    f(nbRows - 1, 0) = f(nbRows - 1, 0) - sqrt(2) * b(nbRows - 1, 0);
}

void adjustScale(const sfmData::SfMData& sfmData, image::Image<float>& initDepth, size_t viewID)
{
    const sfmData::Landmarks& landmarks = sfmData.getLandmarks();
    const sfmData::LandmarksPerView landmarksPerView = sfmData::getLandmarksPerViews(sfmData);
    const sfmData::LandmarkIdSet& visibleLandmarks = landmarksPerView.at(viewID);

    size_t numberOf3dPoints = visibleLandmarks.size();

    Eigen::VectorXf knownDepths(numberOf3dPoints);
    Eigen::VectorXf estimatedDepths(numberOf3dPoints);

    const sfmData::CameraPose& currentPose = sfmData.getPose(sfmData.getView(viewID));
    const geometry::Pose3& pose = currentPose.getTransform();

    for (int i = 0; i < numberOf3dPoints; ++i)
    {
        size_t currentLandmarkIndex = visibleLandmarks.at(i);
        const sfmData::Landmark& currentLandmark = landmarks.at(currentLandmarkIndex);
        knownDepths(i) = pose.depth(currentLandmark.X);

        sfmData::Observation observationInCurrentPicture = currentLandmark.getObservations().at(viewID);

        int rowInd = observationInCurrentPicture.getY();
        int colInd = observationInCurrentPicture.getX();

        estimatedDepths(i) = initDepth(rowInd, colInd);
    }

    float num = estimatedDepths.transpose() * knownDepths;
    float denom = estimatedDepths.transpose() * estimatedDepths;
    float scale = num / denom;
    initDepth *= scale;
}

void getZ0FromLandmarks(const sfmData::SfMData& sfmData,
                        image::Image<float>& z0,
                        image::Image<float>& maskZ0,
                        const size_t viewID,
                        const image::Image<float>& mask)
{
    const sfmData::Landmarks& landmarks = sfmData.getLandmarks();
    const sfmData::LandmarksPerView landmarksPerView = sfmData::getLandmarksPerViews(sfmData);
    const sfmData::LandmarkIdSet& visibleLandmarks = landmarksPerView.at(viewID);

    size_t numberOf3dPoints = visibleLandmarks.size();

    const sfmData::CameraPose& currentPose = sfmData.getPose(sfmData.getView(viewID));
    const geometry::Pose3& pose = currentPose.getTransform();

    for (int i = 0; i < numberOf3dPoints; ++i)
    {
        size_t currentLandmarkIndex = visibleLandmarks.at(i);
        const sfmData::Landmark& currentLandmark = landmarks.at(currentLandmarkIndex);
        sfmData::Observation observationInCurrentPicture = currentLandmark.getObservations().at(viewID);

        int rowInd = observationInCurrentPicture.getY();
        int colInd = observationInCurrentPicture.getX();

        if (mask(rowInd, colInd) > 0.7)
        {
            z0(rowInd, colInd) = pose.depth(currentLandmark.X);
            maskZ0(rowInd, colInd) = 1.0;
        }
    }
}

void smoothIntegration(const image::Image<image::RGBfColor>& normals,
                       image::Image<float>& depth,
                       bool perspective,
                       const Eigen::Matrix3f& K,
                       const image::Image<float>& mask,
                       const image::Image<float>& z0,
                       const image::Image<float>& maskZ0)
{
    std::cout << "WIP" << std::endl;
}

void convertZtoDistance(const aliceVision::image::Image<float>& zMap, aliceVision::image::Image<float>& distanceMap, const Eigen::Matrix3f& K)
{
    int nbRows = zMap.rows();
    int nbCols = zMap.cols();

    float f = K(0, 0);
    float u0 = K(0, 2);
    float v0 = K(1, 2);

    for (int v = 0; v < nbRows; ++v)
    {
        for (int u = 0; u < nbCols; ++u)
        {
            float L = pow((u - u0), 2) + pow((v - v0), 2) + pow(f, 2);
            distanceMap(v, u) = zMap(v, u) * L / f;
        }
    }
}

void convertDistanceToZ(const aliceVision::image::Image<float>& distanceMap, aliceVision::image::Image<float>& zMap, const Eigen::Matrix3f& K)
{
    int nbRows = zMap.rows();
    int nbCols = zMap.cols();

    float f = K(0, 0);
    float u0 = K(0, 2);
    float v0 = K(1, 2);

    for (int v = 0; v < nbRows; ++v)
    {
        for (int u = 0; u < nbCols; ++u)
        {
            float L = pow((u - u0), 2) + pow((v - v0), 2) + pow(f, 2);
            zMap(v, u) = distanceMap(v, u) * L;
        }
    }
}

void loadNormalMap(aliceVision::image::Image<aliceVision::image::RGBColor> inputNormals,
                   const aliceVision::image::Image<float>& normalsMask,
                   aliceVision::image::Image<aliceVision::image::RGBfColor>& outputNormals)
{
    int nbCols = inputNormals.cols();
    int nbRows = inputNormals.rows();

    bool hasMask = !((normalsMask.rows() == 1) && (normalsMask.cols() == 1));

    for (int j = 0; j < nbCols; ++j)
    {
        for (int i = 0; i < nbRows; ++i)
        {
            if ((normalsMask(i, j) > 0.7) || !hasMask)
            {
                outputNormals(i, j)(0) = 2.0 * inputNormals(i, j)(0) / 255.0 - 1;
                outputNormals(i, j)(1) = -(2.0 * inputNormals(i, j)(1) / 255.0 - 1);
                outputNormals(i, j)(2) = -inputNormals(i, j)(2) / 255.0;
            }
            else
            {
                outputNormals(i, j)(0) = 0;
                outputNormals(i, j)(1) = 0;
                outputNormals(i, j)(2) = -1;
            }
        }
    }
}

}  // namespace photometricStereo
}  // namespace aliceVision
