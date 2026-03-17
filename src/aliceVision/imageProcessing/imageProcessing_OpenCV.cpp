// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/imageProcessing/imageProcessing_OpenCV.hpp>

#include <aliceVision/image/conversionOpenCV.hpp>


namespace aliceVision {
namespace imageProcessing {


bool BilateralFilterProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
											image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views
		
		return true;
	}

	// Create temporary OpenCV Mat (keep only 3 Channels) to handled Eigen data of our image
	cv::Mat openCVMatIn = image::imageRGBAToCvMatBGR(image, CV_32FC3);
	cv::Mat openCVMatOut(image.width(), image.height(), CV_32FC3);

	cv::bilateralFilter(openCVMatIn, openCVMatOut, _distance, _sigmaColor, _sigmaSpace);

	// Copy filtered data from openCV Mat(3 channels) to our image(keep the alpha channel unfiltered)
	image::cvMatBGRToImageRGBA(openCVMatOut, image);


	return true;
}

bool ClaheFilterProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
										image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	// Convert alicevision::image to BGR openCV Mat
	cv::Mat BGRMat = image::imageRGBAToCvMatBGR(image);

	// Convert BGR format to Lab format
	cv::Mat labImg;
	cv::cvtColor(BGRMat, labImg, cv::COLOR_LBGR2Lab);

	// Extract the L channel
	cv::Mat L;
	cv::extractChannel(labImg, L, 0);

	// normalise L channel from [0, 100] to [0, 1]
	std::for_each(L.begin<float>(), L.end<float>(), [](float& pixel) { pixel /= 100.0; });

	// Convert float image to 16bit
	L.convertTo(L, CV_16U, 65535.0);

	// apply Clahe algorithm to the L channel
	{
		const cv::Ptr<cv::CLAHE> clahe =
		  cv::createCLAHE(_clipLimit, cv::Size(_tileGridSize, _tileGridSize));
		clahe->apply(L, L);
	}

	// Convert 16bit image to float
	L.convertTo(L, CV_32F, 1.0 / 65535.0);

	// normalise back L channel from [0, 1] to [0, 100]
	std::for_each(L.begin<float>(), L.end<float>(), [](float& pixel) { pixel *= 100.0; });

	// Merge back Lab colors channels
	cv::insertChannel(L, labImg, 0);

	// Convert Lab format to BGR format
	cv::cvtColor(labImg, BGRMat, cv::COLOR_Lab2LBGR);

	// Copy filtered data from openCV Mat to our alicevision image(keep the alpha channel unfiltered)
	image::cvMatBGRToImageRGBA(BGRMat, image);

	return true;
}


bool NlmFilterProcess::processInternal(const sfmData::SfMData & sfmData, sfmData::View & view, camera::IntrinsicBase * camera,
									  image::Image<image::RGBAfColor> & image, bool dryRun)
{
	if (dryRun)
	{
		// No change on image properties, metadata
		// No change on intrinsics
		// No change on Views

		return true;
	}

	// Create temporary OpenCV Mat (keep only 3 channels) to handle Eigen data of our image
	cv::Mat openCVMatIn = image::imageRGBAToCvMatBGR(image, CV_8UC3);
	cv::Mat openCVMatOut(image.width(), image.height(), CV_8UC3);

	cv::fastNlMeansDenoisingColored(openCVMatIn, openCVMatOut, _filterStrength, _filterStrengthColor, _templateWindowSize,
									_searchWindowSize);

	// Copy filtered data from OpenCV Mat(3 channels) to our image (keep the alpha channel unfiltered)
	image::cvMatBGRToImageRGBA(openCVMatOut, image);

	return true;
}


}  // namespace imageProcessing
}  // namespace aliceVision
