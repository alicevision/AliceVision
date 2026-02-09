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
			// if (pixelsIntensity_cur(ind) > 10.0 / 255.0)  // remove dark pixels
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
	double epsilonHuberLoss = 1.0;

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

		Eigen::MatrixXf pixelIntensityEstimated = normalsFull * lightingDirection;
		Eigen::VectorXf residuals = pixelsIntensityFull - pixelIntensityEstimated;
		double meansq = residuals.cwiseProduct(residuals).mean();

		ALICEVISION_LOG_INFO("Initial meansq : " << meansq);
		ALICEVISION_LOG_INFO("NB pix: " << nb_pix);

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
		double lightingDistance = 2.0 * distTosceneCenter.mean();

		ALICEVISION_LOG_INFO("Scene center: " << sceneCenter.transpose());

		// optimisation of the position on the line (sceneCenter,lightingDirection)
		ALICEVISION_LOG_INFO("Initial lightingDistance: " << lightingDistance);
		ALICEVISION_LOG_INFO("Initial lightingIntensity: " << lightingIntensity);
		coarsePunctualLightEstimation(pointsFull, normalsFull, pixelsIntensityFull, sceneCenter, lightingDirection, lightingIntensity, epsilonHuberLoss, lightingDistance);
		ALICEVISION_LOG_INFO("Estimated lightingDistance: " << lightingDistance);
		ALICEVISION_LOG_INFO("Estimated lightingIntensity: " << lightingIntensity);

		Eigen::Vector3f lightingPosition = sceneCenter + lightingDistance * lightingDirection;
		lightingIntensity = lightingIntensity * lightingDistance * lightingDistance;
		
		ALICEVISION_LOG_INFO("Initial lightingPosition: " << lightingPosition.transpose());
		ALICEVISION_LOG_INFO("Initial lightingIntensity: " << lightingIntensity);
		pointSourceModelRefinement(pointsFull, normalsFull, pixelsIntensityFull, epsilonHuberLoss, lightingPosition, lightingIntensity);
		ALICEVISION_LOG_INFO("Estimated lightingPosition: " << lightingPosition.transpose());
		ALICEVISION_LOG_INFO("Estimated lightingIntensity: " << lightingIntensity);

		if(lightType == LightType::Punctual){
            lightings.emplace(lightId, std::make_shared<PunctualLighting>(lightingPosition, lightingIntensity));
			return true;
		}
		return false;
    }
    
    return true;
}

}  // namespace lightingEstimation
}  // namespace aliceVision
