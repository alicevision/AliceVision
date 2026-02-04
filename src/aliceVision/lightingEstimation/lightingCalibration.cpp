// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "lightingCalibration.hpp"
#include "lightingEstimation.hpp"
#include "ellipseGeometry.hpp"
#include "lightingMinimisations.hpp"

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/image/convolution.hpp>
#include <aliceVision/photometricStereo/photometricStereo.hpp>
#include <aliceVision/photometricStereo/photometricDataIO.hpp>

#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/program_options.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

#include <filesystem>
#include <math.h>

namespace fs = std::filesystem;
namespace bpt = boost::property_tree;

namespace aliceVision {
namespace lightingEstimation {

CalibrationData::CalibrationData() : 
    points(Eigen::MatrixX3f(0, 3)),
    normals(Eigen::MatrixX3f(0, 3)), 
    pixels(Eigen::MatrixX2<unsigned int>(0, 2)), 
    pixelsIntensity(Eigen::VectorXf(0))
{}

bool CalibrationData::prepareView(const aliceVision::IndexT viewId, 
								  const sfmData::SfMData& sfmData, 
								  const CalibrationSpheres& calibrationSpheres, 
								  bool usePose,
								  unsigned int resolution)
{
    aliceVision::sfmData::View::sptr currentView = sfmData.getViews().at(viewId);
	if (!currentView)
	{
		ALICEVISION_LOG_WARNING("No view found for id '" << viewId << "'.");
		return false;
	}

	const fs::path imagePath = fs::path(currentView->getImage().getImagePath());
	if (boost::algorithm::icontains(imagePath.stem().string(), "ambient"))
	{
		return false;
	}
	ALICEVISION_LOG_INFO("  - " << imagePath.string());
    
	std::shared_ptr<std::vector<CalibrationSphere>> sphereList = calibrationSpheres.at(viewId);
	if (!sphereList)
	{
		ALICEVISION_LOG_WARNING("No detected sphere found for '" << imagePath << "'.");
		return false;
	}
	std::string picturePath = imagePath.string();

	// intrinsics
	IndexT intrinsicId = currentView->getIntrinsicId();
	float focalPx = sfmData.getIntrinsics().at(intrinsicId)->getParameters().at(0);
	std::array<unsigned int, 2> imageDim = {sfmData.getIntrinsics().at(intrinsicId)->w(), sfmData.getIntrinsics().at(intrinsicId)->h()};
	float x_p = (float)imageDim[0] / 2 + sfmData.getIntrinsics().at(intrinsicId)->getParameters().at(2);
	float y_p = (float)imageDim[1] / 2 + sfmData.getIntrinsics().at(intrinsicId)->getParameters().at(3);
	// Create K matrix
	Eigen::MatrixXf K = Eigen::MatrixXf::Zero(3, 3);
	K << focalPx, 0.0, x_p, 0.0, focalPx, y_p, 0.0, 0.0, 1.0;

	// pose
	Eigen::Matrix4f RT = Eigen::Matrix4f::Identity();
	if (usePose)
	{
		IndexT poseId = currentView->getPoseId();
		Eigen::Matrix4f av_to_vis = Eigen::Matrix4f::Identity();
		av_to_vis(1, 1) = -1.;
		av_to_vis(2, 2) = -1.;
		RT = sfmData.getPoses().at(poseId)->getTransform().getHomogeneous().cast<float>() * av_to_vis;
	}
    
    // image
	image::Image<float> imageFloat;
	image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);
    
    // getting data
    std::vector<Eigen::MatrixX3f> pointsList;
    std::vector<Eigen::MatrixX3f> normalsList;
    std::vector<Eigen::VectorXf> pixelsIntensityList;

	// load set of sphere geometries
	unsigned int nb_pix = 0;
	for (auto sphere = sphereList->begin(); sphere != sphereList->end(); sphere++)
	{
		if (usePose && sphere->is2D())
			continue;
		// get sphere center and radius
		Eigen::Vector3f sphereCenterWorld = sphere->getCenter();
		Eigen::Vector3f sphereCenterCam = RT.block<3, 3>(0, 0) * sphereCenterWorld + RT.block<3, 1>(0, 3);
		float sphereRadius = sphere->getRadius();
        
		Eigen::MatrixX3f normals_cur;
		Eigen::MatrixX3f points_cur;
		Eigen::MatrixX2<unsigned int> pixels_cur;
		spherePositionAndNormalsOnImage(imageDim, K, sphereCenterCam, sphereRadius, pixels_cur, points_cur, normals_cur, resolution);
		// back on world coordinates
		points_cur = (points_cur.rowwise() - RT.block<3, 1>(0, 3).transpose()) * RT.block<3, 3>(0, 0);
		normals_cur = normals_cur * RT.block<3, 3>(0, 0);

		// pixels intensity
		Eigen::VectorXf pixelsIntensity_cur = Eigen::VectorXf(pixels_cur.rows());
		std::vector<unsigned int> nonZeroIntensities;
		for (unsigned int ind = 0; ind < pixels_cur.rows(); ind++)
		{
			pixelsIntensity_cur(ind) = imageFloat(pixels_cur(ind, 1), pixels_cur(ind, 0));
			if (pixelsIntensity_cur(ind) > 10.0 / 255.0)  // remove dark pixels
				nonZeroIntensities.push_back(ind);
		}
		normalsList.push_back(normals_cur(nonZeroIntensities, Eigen::placeholders::all));
		pixelsIntensityList.push_back(pixelsIntensity_cur(nonZeroIntensities));
		if (usePose)
			pointsList.push_back(points_cur(nonZeroIntensities, Eigen::placeholders::all));
		nb_pix += nonZeroIntensities.size();
	}

    if (nb_pix == 0)
    {
        return true; // no sphere, or sphere out of range, not a problem if other images exists
    }

	// concatenate matrices
    normals.resize(nb_pix, 3);
	pixelsIntensity.resize(nb_pix);
    if (usePose)
		points.resize(nb_pix, 3);
	
	unsigned int startAt = 0;
	for (size_t i = 0; i < normalsList.size(); ++i)
	{
		normals.block(startAt, 0, normalsList[i].rows(), 3) = normalsList[i];
		pixelsIntensity.block(startAt, 0, pixelsIntensityList[i].rows(), 1) = pixelsIntensityList[i];
		if (usePose)
		{
			points.block(startAt, 0, pointsList[i].rows(), 3) = pointsList[i];
		}
		startAt += normalsList[i].rows();
	}

	return true;
}

unsigned int CalibrationData::nbPixels() const { return normals.rows(); }

const Eigen::MatrixX3f& CalibrationData::getPoints() { return points; }

const Eigen::MatrixX3f& CalibrationData::getNormals() { return normals; }

const Eigen::MatrixX2<unsigned int>& CalibrationData::getPixels() { return pixels; }

const Eigen::VectorXf& CalibrationData::getPixelsIntensity() { return pixelsIntensity; }


bool lightCalibration(const sfmData::SfMData& sfmData, const CalibrationSpheres& calibrationSpheres, const LightType lightType, Lightings &lightings)
{
    CalibrationDatas calibrationDatas = CalibrationDatas();
    std::map<aliceVision::IndexT, std::vector<aliceVision::IndexT>> viewsId_per_lightId;
	std::vector<std::string> imageList;
	std::vector<IndexT> viewIdList;
    
    bool usePose = true;
	double epsilonHuberLoss = 2.0;

    // data preparation
	ALICEVISION_LOG_INFO("Data preparation");
    for (auto& viewIt : sfmData.getViews())
    {
        ALICEVISION_LOG_INFO("View Id: " << viewIt.first);
        
		const fs::path imagePath = fs::path(viewIt.second->getImage().getImagePath());
		imageList.push_back(imagePath.string());
		viewIdList.push_back(viewIt.first);

        std::shared_ptr<CalibrationData> calibrationData = std::make_shared<CalibrationData>();

        if (!calibrationData->prepareView(viewIt.first, sfmData, calibrationSpheres, usePose))
        {
            ALICEVISION_LOG_ERROR("Could not prepare view");
            return false;
        }
        calibrationDatas.emplace(viewIt.first, calibrationData);

        IndexT lightId = viewIt.second->getLightId();
        if (viewsId_per_lightId.find(lightId) == viewsId_per_lightId.end())
        {
            viewsId_per_lightId.emplace(lightId, std::vector<IndexT>());
        }
        viewsId_per_lightId[lightId].push_back(viewIt.first);
    }
    
    // computation per light
	ALICEVISION_LOG_INFO("Computation per light");
    for (auto itLight = viewsId_per_lightId.begin(); itLight != viewsId_per_lightId.end(); itLight++)
    {
        unsigned int lightId = itLight->first;
        
        // count number of pixels
        unsigned int nb_pix = 0;
        for (size_t i = 0; i < itLight->second.size(); ++i)
        {
            IndexT viewId = itLight->second[i];
            nb_pix += calibrationDatas.at(viewId)->nbPixels();
        }
		
		if(nb_pix == 0)
		{
			ALICEVISION_LOG_ERROR("No pixels for light " << lightId);
			return false;
		}

        // concatenate matrices
        Eigen::MatrixX3f pointsFull(nb_pix, 3);
        Eigen::MatrixX3f normalsFull(nb_pix, 3);
        Eigen::VectorXf pixelsIntensityFull(nb_pix);
        
        unsigned int startAt = 0;
        for (size_t i = 0; i < itLight->second.size(); ++i)
        {
            IndexT viewId = itLight->second[i];
            auto &calibrationData = calibrationDatas.at(viewId);
            pointsFull.block(startAt, 0, calibrationData->getPoints().rows(), 3) = calibrationData->getPoints();
            normalsFull.block(startAt, 0, calibrationData->getNormals().rows(), 3) = calibrationData->getNormals();
            pixelsIntensityFull.block(startAt, 0, calibrationData->getPixelsIntensity().rows(), 1) = calibrationData->getPixelsIntensity();
            startAt += calibrationData->nbPixels();
        }
        
        // directionnal lighting estimation

		// simple linear resolution
		Eigen::Vector3f lightingDirection = normalsFull.colPivHouseholderQr().solve(pixelsIntensityFull);

		// optimisation with better loss
		ALICEVISION_LOG_INFO("Initial lightingDirection: " << lightingDirection.transpose());
		coarseDirectionnalLightEstimation(normalsFull, pixelsIntensityFull, epsilonHuberLoss, lightingDirection);
		ALICEVISION_LOG_INFO("Estimated lightingDirection: " << lightingDirection.transpose());

		double lightingIntensity = lightingDirection.norm();
		lightingDirection = lightingDirection / lightingIntensity;
        if (lightType == LightType::Directionnal)
        {
            lightings.emplace(lightId, std::make_shared<DirectionnalLighting>(lightingDirection, lightingIntensity));
			return true;
        }

        // punctual lighting estimation

		Eigen::Vector3f sceneCenter = pointsFull.colwise().mean();
		Eigen::MatrixX3f pointsCentered = pointsFull.rowwise() - sceneCenter.transpose();
		Eigen::MatrixXf distTosceneCenter = pointsCentered.rowwise().norm();
		double lightingDistance = distTosceneCenter.mean() * 2.0;
		sceneCenter << 0. , 0. , 0.;
		lightingDistance = 10.;

		ALICEVISION_LOG_INFO("Scene center: " << sceneCenter.transpose());

		// optimisation of the position on the line (sceneCenter,lightingDirection)
		ALICEVISION_LOG_INFO("Initial lightingDistance: " << lightingDistance);
		ALICEVISION_LOG_INFO("Initial lightingIntensity: " << lightingIntensity);
		coarsePunctualLightEstimation(pointsFull, normalsFull, pixelsIntensityFull, sceneCenter, lightingDirection, epsilonHuberLoss, lightingDistance, lightingIntensity);
		ALICEVISION_LOG_INFO("Estimated lightingDistance: " << lightingDistance);
		ALICEVISION_LOG_INFO("Estimated lightingIntensity: " << lightingIntensity);

		Eigen::Vector3f lightingPosition = sceneCenter + lightingDistance * lightingDirection;
		lightingIntensity = lightingIntensity * lightingDistance * lightingDistance;
		
		// ALICEVISION_LOG_INFO("Initial lightingPosition: " << lightingPosition.transpose());
		// ALICEVISION_LOG_INFO("Initial lightingIntensity: " << lightingIntensity);
		// pointSourceModelRefinement(pointsFull, normalsFull, pixelsIntensityFull, epsilonHuberLoss, lightingPosition, lightingIntensity);
		// ALICEVISION_LOG_INFO("Estimated lightingPosition: " << lightingPosition.transpose());
		// ALICEVISION_LOG_INFO("Estimated lightingIntensity: " << lightingIntensity);

		if(lightType == LightType::Punctual){
            lightings.emplace(lightId, std::make_shared<PunctualLighting>(lightingPosition, lightingIntensity));
			return true;
		}
		return false;
    }
    
    return true;
}

void lightCalibrationOneImage(const std::string& picturePath,
                              const std::array<float, 3>& sphereParam,
                              const float focal,
                              const std::string& method,
                              Eigen::VectorXf& lightingDirection,
                              std::array<float, 3>& intensities)
{
    // Read picture :
    image::Image<float> imageFloat;
    image::readImage(picturePath, imageFloat, image::EImageColorSpace::NO_CONVERSION);

    image::Image<image::RGBfColor> imageFloatColor;
    image::readImage(picturePath, imageFloatColor, image::EImageColorSpace::NO_CONVERSION);

    // If method = brightest point :
    if (!method.compare("brightestPoint"))
    {
        // Detect brightest point :
        Eigen::Vector2f brigthestPoint;
        detectBrightestPoint(sphereParam, imageFloat, brigthestPoint);

        Eigen::Vector2f brigthestPoint_xy;
        brigthestPoint_xy(0) = brigthestPoint(0) - imageFloat.cols() / 2;
        brigthestPoint_xy(1) = brigthestPoint(1) - imageFloat.rows() / 2;

        Eigen::Vector3f normalBrightestPoint;
        getNormalOnSphere(brigthestPoint_xy(0), brigthestPoint_xy(1), sphereParam, normalBrightestPoint);

        // Observation direction :
        Eigen::Vector3f observationRayPersp;
        Eigen::Vector3f observationRay;

        // orthographic approximation :
        observationRay(0) = 0.0;
        observationRay(1) = 0.0;
        observationRay(2) = -1.0;

        observationRayPersp(0) = brigthestPoint_xy(0) / focal;
        observationRayPersp(1) = brigthestPoint_xy(1) / focal;
        observationRayPersp(2) = 1.0;
        observationRayPersp = -observationRayPersp / observationRayPersp.norm();

        // Evaluate lighting direction :
        // lightingDirection = 2 * normalBrightestPoint.dot(observationRayPersp) * normalBrightestPoint - observationRayPersp;
        // lightingDirection = lightingDirection / lightingDirection.norm();

        lightingDirection = 2 * normalBrightestPoint.dot(observationRay) * normalBrightestPoint - observationRay;

        intensities.fill(1.0);
    }
    // If method = whiteSphere :
    else if (!method.compare("whiteSphere"))
    {
        // Evaluate light direction and intensity by pseudo-inverse
        const int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows() / 2);
        const int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols() / 2);

        const float radius = sphereParam[2];

        image::Image<float> patch;
        patch = imageFloat.block(minISphere, minJSphere, 2 * radius, 2 * radius);

        image::Image<image::RGBfColor> patchRGB;
        patchRGB = imageFloatColor.block(minISphere, minJSphere, 2 * radius, 2 * radius);

        const int nbPixelsPatch = 4 * radius * radius;
        Eigen::VectorXf imSphere(nbPixelsPatch);
        Eigen::MatrixXf normalSphere(nbPixelsPatch, 3);

        Eigen::MatrixXf imSphereColor(nbPixelsPatch, 3);

        int currentIndex = 0;

        for (int j = 0; j < patch.cols(); ++j)
        {
            for (int i = 0; i < patch.rows(); ++i)
            {
                const float distanceToCenter = std::sqrt((i - radius) * (i - radius) + (j - radius) * (j - radius));
                if ((distanceToCenter < 0.95 * radius) && (patch(i, j) > 0.2) && (patch(i, j) < 0.8))
                {
                    // imSphere = normalSphere.s
                    imSphere(currentIndex) = patch(i, j);
                    imSphereColor(currentIndex, 0) = patchRGB(i, j)(0);
                    imSphereColor(currentIndex, 1) = patchRGB(i, j)(1);
                    imSphereColor(currentIndex, 2) = patchRGB(i, j)(2);

                    normalSphere(currentIndex, 0) = (float(j) - radius) / radius;
                    normalSphere(currentIndex, 1) = (float(i) - radius) / radius;
                    normalSphere(currentIndex, 2) = -sqrt(1 - normalSphere(currentIndex, 0) * normalSphere(currentIndex, 0) -
                                                          normalSphere(currentIndex, 1) * normalSphere(currentIndex, 1));

                    ++currentIndex;
                }
            }
        }

        Eigen::MatrixXf normalSphereMasked(currentIndex, 3);
        normalSphereMasked = normalSphere.block(0, 0, currentIndex, 3);

        Eigen::VectorXf imSphereMasked(currentIndex);
        imSphereMasked = imSphere.head(currentIndex);

        Eigen::MatrixXf imSphereColorMasked(currentIndex, 3);
        imSphereColorMasked = imSphereColor.block(0, 0, currentIndex, 3);

        lightingDirection = normalSphere.colPivHouseholderQr().solve(imSphere);

        float intensity = lightingDirection.norm();
        lightingDirection = lightingDirection / intensity;

        // Channelwise intensity estimation :
        Eigen::VectorXf shading(currentIndex);
        shading = normalSphereMasked * lightingDirection;

        for (int ch = 0; ch < 3; ++ch)
        {
            Eigen::VectorXf currentChannelValues(currentIndex);
            currentChannelValues = imSphereColorMasked.col(ch);
            intensities[ch] = shading.dot(currentChannelValues) / shading.squaredNorm();
        }

    }

    // If method = SH :
    else if (!method.compare("SH"))
    {
        size_t lightSize = lightingDirection.size();

        // Evaluate light direction and intensity by pseudo-inverse
        int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows() / 2);
        int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols() / 2);

        float radius = sphereParam[2];

        image::Image<float> patch;
        patch = imageFloat.block(minISphere, minJSphere, 2 * radius, 2 * radius);

        int nbPixelsPatch = 4 * radius * radius;
        Eigen::VectorXf imSphere(nbPixelsPatch);
        Eigen::MatrixXf normalSphere(nbPixelsPatch, lightSize);

        int currentIndex = 0;

        for (size_t j = 0; j < patch.cols(); ++j)
        {
            for (size_t i = 0; i < patch.rows(); ++i)
            {
                float distanceToCenter = sqrt((i - radius) * (i - radius) + (j - radius) * (j - radius));
                if (distanceToCenter < 0.95 * radius && (patch(i, j) > 0.2) && (patch(i, j) < 0.8))
                {
                    imSphere(currentIndex) = patch(i, j);

                    normalSphere(currentIndex, 0) = (float(j) - radius) / radius;
                    normalSphere(currentIndex, 1) = (float(i) - radius) / radius;
                    normalSphere(currentIndex, 2) = -sqrt(1 - normalSphere(currentIndex, 0) * normalSphere(currentIndex, 0) -
                                                          normalSphere(currentIndex, 1) * normalSphere(currentIndex, 1));
                    normalSphere(currentIndex, 3) = 1;
                    if (lightSize > 4)
                    {
                        normalSphere(currentIndex, 4) = normalSphere(currentIndex, 0) * normalSphere(currentIndex, 1);
                        normalSphere(currentIndex, 5) = normalSphere(currentIndex, 0) * normalSphere(currentIndex, 2);
                        normalSphere(currentIndex, 6) = normalSphere(currentIndex, 1) * normalSphere(currentIndex, 2);
                        normalSphere(currentIndex, 7) = normalSphere(currentIndex, 0) * normalSphere(currentIndex, 0) -
                                                        normalSphere(currentIndex, 1) * normalSphere(currentIndex, 1);
                        normalSphere(currentIndex, 8) = 3 * normalSphere(currentIndex, 2) * normalSphere(currentIndex, 2) - 1;
                    }
                    ++currentIndex;
                }
            }
        }

        Eigen::MatrixXf normalSphereMasked(currentIndex, lightSize);
        normalSphereMasked = normalSphere.block(0, 0, currentIndex, lightSize);

        Eigen::VectorXf imSphereMasked(currentIndex);
        imSphereMasked = imSphere.head(currentIndex);

        // 1) Directionnal part estimation :
        Eigen::MatrixXf normalOrdre1(currentIndex, 3);
        normalOrdre1 = normalSphereMasked.leftCols(3);
        Eigen::Vector3f directionnalPart = normalOrdre1.colPivHouseholderQr().solve(imSphereMasked);
        float intensity = directionnalPart.norm();
        intensities.fill(intensity);
        directionnalPart = directionnalPart / intensity;

        // 2) Other order estimation :
        Eigen::VectorXf imSphereModif(currentIndex);
        imSphereModif = imSphereMasked;
        for (size_t i = 0; i < currentIndex; ++i)
        {
            for (size_t k = 0; k < 3; ++k)
            {
                imSphereModif(i) -= normalSphereMasked(i, k) * directionnalPart(k);
            }
        }
        Eigen::VectorXf secondOrder(6);
        secondOrder = normalSphereMasked.rightCols(6).colPivHouseholderQr().solve(imSphereModif);

        lightingDirection << directionnalPart, secondOrder;
    }
}

void calibrateLightFromRealSphere(const image::Image<float>& imageFloat,
                                  const cv::Mat& maskCV,
                                  const Eigen::Matrix3f& K,
                                  const float sphereRadius,
                                  const std::string& method,
                                  Eigen::VectorXf& lightingDirection,
                                  float& intensity)
{
    image::Image<image::RGBfColor> normals(imageFloat.width(), imageFloat.height());
    image::Image<float> newMask(imageFloat.width(), imageFloat.height());

    getRealNormalOnSphere(maskCV, K, sphereRadius, normals, newMask);

    image::Image<image::RGBColor> normalsPNG(maskCV.cols, maskCV.rows);
    aliceVision::photometricStereo::convertNormalMap2png(normals, normalsPNG);

    // If method = brightest point :
    if (!method.compare("brightestPoint"))
    {
        // Detect brightest point :
        Eigen::Vector2f brigthestPoint;
        detectBrightestPoint(newMask, imageFloat, brigthestPoint);

        Eigen::Vector2f brigthestPoint_xy;
        brigthestPoint_xy(0) = brigthestPoint(0) - imageFloat.cols() / 2;
        brigthestPoint_xy(1) = brigthestPoint(1) - imageFloat.rows() / 2;

        Eigen::Vector3f normalBrightestPoint;
        normalBrightestPoint = normals(round(brigthestPoint(1)), round(brigthestPoint(0))).cast<float>();

        // Observation direction :
        Eigen::Vector3f observationRayPersp;

        // orthographic approximation :
        observationRayPersp(0) = brigthestPoint_xy(0) / K(0, 0);
        observationRayPersp(1) = brigthestPoint_xy(1) / K(0, 0);
        observationRayPersp(2) = 1.0;
        observationRayPersp = -observationRayPersp / observationRayPersp.norm();

        // Evaluate lighting direction :
        lightingDirection = 2 * normalBrightestPoint.dot(observationRayPersp) * normalBrightestPoint - observationRayPersp;
        lightingDirection = lightingDirection / lightingDirection.norm();

        intensity = 1.0;
    }

    // If method = whiteSphere :
    else if (!method.compare("whiteSphere"))
    {
        // Evaluate light direction and intensity by pseudo-inverse

        std::vector<int> indices;
        aliceVision::photometricStereo::getIndMask(newMask, indices);

        Eigen::VectorXf imSphere(indices.size());
        Eigen::MatrixXf normalSphere(indices.size(), 3);

        int currentIndex = 0;

        for (int j = 0; j < newMask.cols(); ++j)
        {
            for (int i = 0; i < newMask.rows(); ++i)
            {
                if (newMask(i, j) > 0.1 && (imageFloat(i, j) > 0.2) && (imageFloat(i, j) < 0.8))
                {
                    // imSphere = normalSphere.s
                    imSphere(currentIndex) = imageFloat(i, j);

                    normalSphere(currentIndex, 0) = normals(i, j)(0);
                    normalSphere(currentIndex, 1) = normals(i, j)(1);
                    normalSphere(currentIndex, 2) = normals(i, j)(2);

                    ++currentIndex;
                }
            }
        }

        lightingDirection = normalSphere.colPivHouseholderQr().solve(imSphere);

        intensity = lightingDirection.norm();
        lightingDirection = lightingDirection / intensity;
    }
}

void detectBrightestPoint(const std::array<float, 3>& sphereParam, const image::Image<float>& imageFloat, Eigen::Vector2f& brigthestPoint)
{
    image::Image<float> patch;
    std::array<float, 2> patchOrigin;
    cutImage(imageFloat, sphereParam, patch, patchOrigin);

    image::Image<float> convolutedPatch1;
    image::Image<float> convolutedPatch2;

    // Create Kernel
    size_t kernelSize = round(sphereParam[2] / 20);  // arbitrary
    Eigen::VectorXf kernel(2 * kernelSize + 1);
    createTriangleKernel(kernelSize, kernel);

    image::imageVerticalConvolution(patch, kernel, convolutedPatch1);
    image::imageHorizontalConvolution(convolutedPatch1, kernel, convolutedPatch2);

    Eigen::Index maxRow, maxCol;
    static_cast<void>(convolutedPatch2.maxCoeff(&maxRow, &maxCol));

    brigthestPoint(0) = maxCol + patchOrigin[0];
    brigthestPoint(1) = maxRow + patchOrigin[1];
}

void detectBrightestPoint(const image::Image<float> newMask, const image::Image<float>& imageFloat, Eigen::Vector2f& brigthestPoint)
{
    image::Image<float> patch;
    std::array<float, 2> patchOrigin;
    cutImage(imageFloat, newMask, patch, patchOrigin);

    image::Image<float> convolutedPatch1;
    image::Image<float> convolutedPatch2;

    // Create Kernel
    size_t kernelSize = round(patch.rows() / 40);  // arbitrary
    Eigen::VectorXf kernel(2 * kernelSize + 1);
    createTriangleKernel(kernelSize, kernel);

    image::imageVerticalConvolution(patch, kernel, convolutedPatch1);
    image::imageHorizontalConvolution(convolutedPatch1, kernel, convolutedPatch2);

    Eigen::Index maxRow, maxCol;
    static_cast<void>(convolutedPatch2.maxCoeff(&maxRow, &maxCol));

    brigthestPoint(0) = maxCol + patchOrigin[0];
    brigthestPoint(1) = maxRow + patchOrigin[1];
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

void getNormalOnSphere(const float xPicture, const float yPicture, const std::array<float, 3>& sphereParam, Eigen::Vector3f& currentNormal)
{
    currentNormal(0) = (xPicture - sphereParam[0]) / sphereParam[2];
    currentNormal(1) = (yPicture - sphereParam[1]) / sphereParam[2];
    currentNormal(2) = -sqrt(1 - currentNormal(0) * currentNormal(0) - currentNormal(1) * currentNormal(1));
}

void cutImage(const image::Image<float>& imageFloat,
              const std::array<float, 3>& sphereParam,
              image::Image<float>& patch,
              std::array<float, 2>& patchOrigin)
{
    // Absolute position of the patch's top left corner in image
    const int minISphere = floor(sphereParam[1] - sphereParam[2] + imageFloat.rows() / 2);
    const int minJSphere = floor(sphereParam[0] - sphereParam[2] + imageFloat.cols() / 2);

    patchOrigin[0] = minJSphere;
    patchOrigin[1] = minISphere;

    const int radius = round(sphereParam[2]);

    patch = imageFloat.block(minISphere, minJSphere, 2 * radius, 2 * radius);

    for (int i = 0; i < patch.rows(); ++i)
    {
        for (int j = 0; j < patch.cols(); ++j)
        {
            const float distanceToCenter = (i - patch.rows() / 2) * (i - patch.rows() / 2) + (j - patch.cols() / 2) * (j - patch.cols() / 2);
            if (distanceToCenter > radius * radius + 2)
            {
                patch(i, j) = 0;
            }
        }
    }
}

void cutImage(const image::Image<float>& imageFloat,
              const image::Image<float>& newMask,
              image::Image<float>& patch,
              std::array<float, 2>& patchOrigin)
{
    int minISphere = newMask.rows();
    int minJSphere = newMask.cols();
    int maxISphere = 0;
    int maxJSphere = 0;

    for (int j = 0; j < newMask.cols(); ++j)
    {
        for (int i = 0; i < newMask.rows(); ++i)
        {
            if (newMask(i, j) == 1)
            {
                if (minISphere > i)
                    minISphere = i;

                if (minJSphere > j)
                    minJSphere = j;

                if (maxISphere < i)
                    maxISphere = i;

                if (maxJSphere < j)
                    maxJSphere = j;
            }
        }
    }

    patchOrigin[0] = minJSphere;
    patchOrigin[1] = minISphere;

    patch = imageFloat.block(minISphere, minJSphere, maxISphere - minISphere, maxJSphere - minJSphere);
}

void writeJSON(const std::string& fileName,
               const sfmData::SfMData& sfmData,
               const std::vector<std::string>& imageList,
               const Eigen::MatrixXf& lightMat,
               const std::vector<std::array<float, 3>>& intList,
               const bool saveAsModel,
               const std::string method)
{
    bpt::ptree lightsTree;
    bpt::ptree fileTree;

    int imgCpt = 0;

    std::cout << lightMat << std::endl;

	ALICEVISION_LOG_INFO("Building tree from views");
    for (auto& viewIt : sfmData.getViews())
    {
        auto &viewId = sfmData.getView(viewIt.first);
        const fs::path imagePath = fs::path(viewId.getImage().getImagePath());

        // The file may be in the input SfMData but may not have been calibrated: in that case, it is not in imageList
        const bool calibratedFile = (std::find(imageList.begin(), imageList.end(), viewId.getImage().getImagePath()) != imageList.end());

        // Only write images that were actually used for the lighting calibration, instead of all the input images
        if (!boost::algorithm::icontains(imagePath.stem().string(), "ambient") && calibratedFile)
        {
            bpt::ptree lightTree;
            if (saveAsModel)
            {
                lightTree.put("lightId", imgCpt);
            }
            else
            {
                lightTree.put("viewId", viewId.getViewId());
            }
            if (!method.compare("SH"))
                lightTree.put("type", "SH");
            else
                lightTree.put("type", "directional");

            // Light direction
            bpt::ptree directionNode;
            int lightMatSize = lightMat.cols();
            for (int i = 0; i < lightMatSize; ++i)
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
                cell.put_value<float>(intList.at(imgCpt)[i]);
                intensityNode.push_back(std::make_pair("", cell));
            }
            lightTree.add_child("intensity", intensityNode);

            imgCpt++;

            lightsTree.push_back(std::make_pair("", lightTree));
        }
        else
        {
            ALICEVISION_LOG_INFO("'" << imagePath << "' is in the input SfMData but has not been used for the lighting "
                                     << "calibration or contains 'ambient' in its filename.");
        }
    }

    fileTree.add_child("lights", lightsTree);
    bpt::write_json(fileName, fileTree);
}

void sphereFromLighting(const Eigen::VectorXf& lightVector,
                        const std::array<float, 3> intensity,
                        const std::string outputFileName,
                        const int outputSize)
{
    float radius = (outputSize * 0.9) / 2;
    image::Image<image::RGBfColor> pixelsValues(outputSize, outputSize);

    for (size_t j = 0; j < outputSize; ++j)
    {
        for (size_t i = 0; i < outputSize; ++i)
        {
            float center_xy = outputSize / 2;
            Eigen::VectorXf normalSphere(lightVector.size());
            float distanceToCenter = sqrt((i - center_xy) * (i - center_xy) + (j - center_xy) * (j - center_xy));
            for (size_t ch = 0; ch < 3; ++ch)
            {
                pixelsValues(i, j)(ch) = 0;
            }
            if (distanceToCenter < radius)
            {
                normalSphere(0) = (float(j) - center_xy) / radius;
                normalSphere(1) = (float(i) - center_xy) / radius;
                normalSphere(2) = -sqrt(1 - normalSphere(0) * normalSphere(0) - normalSphere(1) * normalSphere(1));
                if (lightVector.size() > 3)
                {
                    normalSphere(3) = 1;
                }
                if (lightVector.size() > 4)
                {
                    normalSphere(4) = normalSphere(0) * normalSphere(1);
                    normalSphere(5) = normalSphere(0) * normalSphere(2);
                    normalSphere(6) = normalSphere(1) * normalSphere(2);
                    normalSphere(7) = normalSphere(0) * normalSphere(0) - normalSphere(1) * normalSphere(1);
                    normalSphere(8) = 3 * normalSphere(2) * normalSphere(2) - 1;
                }

                for (size_t ch = 0; ch < 3; ++ch)
                {
                    for (size_t k = 0; k < lightVector.size(); ++k)
                    {
                        pixelsValues(i, j)(ch) += normalSphere(k) * lightVector(k);
                    }
                    pixelsValues(i, j)(ch) *= intensity[ch];
                }
            }
        }
    }

    image::writeImage(
      outputFileName,
      pixelsValues,
      image::ImageWriteOptions().toColorSpace(image::EImageColorSpace::NO_CONVERSION).storageDataType(image::EStorageDataType::Float));
}

}  // namespace lightingEstimation
}  // namespace aliceVision
