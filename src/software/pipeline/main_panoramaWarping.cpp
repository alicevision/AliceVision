// This file is part of the AliceVision project.
// Copyright (c) 2020 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Reading command line options
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

// Image related
#include <aliceVision/image/all.hpp>

// Sfmdata
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

// Internal functions
#include <aliceVision/panorama/coordinatesMap.hpp>
#include <aliceVision/panorama/remapBbox.hpp>
#include <aliceVision/panorama/warper.hpp>
#include <aliceVision/panorama/distance.hpp>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

bool computeOptimalPanoramaSize(std::pair<int, int>& optimalSize, const sfmData::SfMData& sfmData, const float ratioUpscale)
{
		// We use a small panorama for probing
		optimalSize.first = 512;
		optimalSize.second = 256;

		// Loop over views to estimate best scale
		std::vector<double> scales;
		for(auto& viewIt : sfmData.getViews())
		{

				// Ignore non positionned views
				const sfmData::View& view = *viewIt.second.get();
				if(!sfmData.isPoseAndIntrinsicDefined(&view))
				{
						continue;
				}

				// Get intrinsics and extrinsics
				const geometry::Pose3 camPose = sfmData.getPose(view).getTransform();
				const camera::IntrinsicBase& intrinsic = *sfmData.getIntrinsicPtr(view.getIntrinsicId());

				// Compute coarse bounding box
				BoundingBox coarseBbox;
				if(!computeCoarseBB(coarseBbox, optimalSize, camPose, intrinsic))
				{
						continue;
				}

				CoordinatesMap map;
				if(!map.build(optimalSize, camPose, intrinsic, coarseBbox))
				{
						continue;
				}

				double scale;
				if(!map.computeScale(scale, ratioUpscale))
				{
						continue;
				}

				scales.push_back(scale);
		}

		if(scales.empty())
		{
				return false;
		}

		std::sort(scales.begin(), scales.end());
		int selected_index = int(floor(float(scales.size() - 1) * ratioUpscale));
		double selected_scale = scales[selected_index];

		optimalSize.first = optimalSize.first * selected_scale;
		optimalSize.second = optimalSize.second * selected_scale;

		ALICEVISION_LOG_INFO("Estimated panorama size: " << optimalSize.first << "x" << optimalSize.second);

		return true;
}

int aliceVision_main(int argc, char** argv)
{
		std::string sfmDataFilename;
		std::string outputDirectory;
		std::pair<int, int> panoramaSize = {0, 0};
		int percentUpscale = 50;
		int tileSize = 256;
		int maxPanoramaWidth = 0;

		image::EStorageDataType storageDataType = image::EStorageDataType::Float;

		int rangeStart = -1;
		int rangeSize = 1;

		// Program description
		po::options_description allParams(
				"Perform panorama stiching of cameras around a nodal point for 360° panorama creation. \n"
				"AliceVision PanoramaWarping");

		// Description of mandatory parameters
		po::options_description requiredParams("Required parameters");
		requiredParams.add_options()("input,i", po::value<std::string>(&sfmDataFilename)->required(), "SfMData file.")(
				"output,o", po::value<std::string>(&outputDirectory)->required(), "Path of the output folder.");
		allParams.add(requiredParams);

		// Description of optional parameters
		po::options_description optionalParams("Optional parameters");
		optionalParams.add_options()
				("panoramaWidth,w", po::value<int>(&panoramaSize.first)->default_value(panoramaSize.first), "Panorama Width in pixels.")
				("maxPanoramaWidth", po::value<int>(&maxPanoramaWidth)->default_value(maxPanoramaWidth), "Max Panorama Width in pixels.")
				("percentUpscale", po::value<int>(&percentUpscale)->default_value(percentUpscale), "Percentage of upscaled pixels.")
				("storageDataType", po::value<image::EStorageDataType>(&storageDataType)->default_value(storageDataType),
				("Storage data type: " + image::EStorageDataType_informations()).c_str())
				("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart), "Range image index start.")
				("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize), "Range size.");
		allParams.add(optionalParams);

		// Setup log level given command line
		std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
		po::options_description logParams("Log parameters");
		logParams.add_options()("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
														"verbosity level (fatal, error, warning, info, debug, trace).");
		allParams.add(logParams);

		// Effectively parse command line given parse options
		po::variables_map vm;
		try
		{
				po::store(po::parse_command_line(argc, argv, allParams), vm);

				if(vm.count("help") || (argc == 1))
				{
						ALICEVISION_COUT(allParams);
						return EXIT_SUCCESS;
				}
				po::notify(vm);
		}
		catch(boost::program_options::required_option& e)
		{
				ALICEVISION_CERR("ERROR: " << e.what());
				ALICEVISION_COUT("Usage:\n\n" << allParams);
				return EXIT_FAILURE;
		}
		catch(boost::program_options::error& e)
		{
				ALICEVISION_CERR("ERROR: " << e.what());
				ALICEVISION_COUT("Usage:\n\n" << allParams);
				return EXIT_FAILURE;
		}

		ALICEVISION_COUT("Program called with the following parameters:");
		ALICEVISION_COUT(vm);

		// Set verbose level given command line
		system::Logger::get()->setLogLevel(verboseLevel);

		bool clampHalf = false;
		oiio::TypeDesc typeColor = oiio::TypeDesc::FLOAT;
		if (storageDataType == image::EStorageDataType::Half || storageDataType == image::EStorageDataType::HalfFinite) 
		{
			typeColor = oiio::TypeDesc::HALF;
			if (storageDataType == image::EStorageDataType::HalfFinite) 
			{
				clampHalf = true;
			}
		} 
		
		// Load information about inputs
		// Camera images
		// Camera intrinsics
		// Camera extrinsics
		sfmData::SfMData sfmData;
		if(!sfmDataIO::Load(sfmData, sfmDataFilename,
												sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS | sfmDataIO::EXTRINSICS)))
		{
				ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmDataFilename << "' cannot be read.");
				return EXIT_FAILURE;
		}

		// Order views by their image names for easier debugging
		std::vector<std::shared_ptr<sfmData::View>> viewsOrderedByName;
		for(auto& viewIt : sfmData.getViews())
		{
				viewsOrderedByName.push_back(viewIt.second);
		}
		std::sort(viewsOrderedByName.begin(), viewsOrderedByName.end(),
							[](const std::shared_ptr<sfmData::View>& a, const std::shared_ptr<sfmData::View>& b) -> bool {
									if(a == nullptr || b == nullptr)
											return true;
									return (a->getImagePath() < b->getImagePath());
							});


		// Define range to compute
		if(rangeStart != -1)
		{
			if(rangeStart < 0 || rangeSize < 0 ||
				std::size_t(rangeStart) > viewsOrderedByName.size())
			{
				ALICEVISION_LOG_ERROR("Range is incorrect");
				return EXIT_FAILURE;
			}

			if(std::size_t(rangeStart + rangeSize) > viewsOrderedByName.size())
			{
				rangeSize = int(viewsOrderedByName.size()) - rangeStart;
			}
		}
		else
		{
				rangeStart = 0;
				rangeSize = int(viewsOrderedByName.size());
		}

		ALICEVISION_LOG_DEBUG("Range to compute: rangeStart=" << rangeStart << ", rangeSize=" << rangeSize);


		// If panorama width is undefined, estimate it
		if(panoramaSize.first <= 0)
		{
			float ratioUpscale = clamp(float(percentUpscale) / 100.0f, 0.0f, 1.0f);

			std::pair<int, int> optimalPanoramaSize;
			if(computeOptimalPanoramaSize(optimalPanoramaSize, sfmData, ratioUpscale))
			{
					panoramaSize = optimalPanoramaSize;
			}
			else
			{
					ALICEVISION_LOG_INFO("Impossible to compute an optimal panorama size");
					return EXIT_FAILURE;
			}

			if (maxPanoramaWidth != 0 && panoramaSize.first > maxPanoramaWidth)
			{
				ALICEVISION_LOG_INFO("The optimal size of the panorama exceeds the maximum size (estimated width: " << panoramaSize.first << ", max width: " << maxPanoramaWidth << ").");
				panoramaSize.first = maxPanoramaWidth;
			}
		}
		
		// Make sure panorama size has required size properties
		/*double max_scale = 1.0 / pow(2.0, 10);
		panoramaSize.first = int(ceil(double(panoramaSize.first) * max_scale) / max_scale);*/
		panoramaSize.second = panoramaSize.first / 2;
		ALICEVISION_LOG_INFO("Choosen panorama size : "  << panoramaSize.first << "x" << panoramaSize.second);


		// Define empty tiles data
		std::unique_ptr<float> empty_float(new float[tileSize * tileSize * 3]);
		std::unique_ptr<char> empty_char(new char[tileSize * tileSize]);
		std::memset(empty_float.get(), 0, tileSize * tileSize * 3 * sizeof(float));
		std::memset(empty_char.get(), 0, tileSize * tileSize * sizeof(char));

		// Preprocessing per view
		for(std::size_t i = std::size_t(rangeStart); i < std::size_t(rangeStart + rangeSize); ++i)
		{
				const std::shared_ptr<sfmData::View> & viewIt = viewsOrderedByName[i];

				// Retrieve view
				const sfmData::View& view = *viewIt;
				if (!sfmData.isPoseAndIntrinsicDefined(&view))
				{
					continue;
				}

				ALICEVISION_LOG_INFO("[" << int(i) + 1 - rangeStart << "/" << rangeSize << "] Processing view " << view.getViewId() << " (" << i + 1 << "/" << viewsOrderedByName.size() << ")");

				// Get intrinsics and extrinsics
				geometry::Pose3 camPose = sfmData.getPose(view).getTransform();
				std::shared_ptr<camera::IntrinsicBase> intrinsic = sfmData.getIntrinsicsharedPtr(view.getIntrinsicId());
			
				// Compute coarse bounding box to make computations faster
				BoundingBox coarseBbox;
				if (!computeCoarseBB(coarseBbox, panoramaSize, camPose, *(intrinsic.get()))) 
				{
					continue;
				}    

				// round to the closest tiles
				BoundingBox snappedCoarseBbox;
				snappedCoarseBbox = coarseBbox;
				snappedCoarseBbox.snapToGrid(tileSize);

				//Initialize bouding box for image
				BoundingBox globalBbox;
				
				std::vector<BoundingBox> boxes;
				for (int y = 0; y < snappedCoarseBbox.height; y += tileSize) 
				{
					for (int x = 0; x < snappedCoarseBbox.width; x += tileSize) 
					{
						BoundingBox localBbox;
						localBbox.left = x + snappedCoarseBbox.left;
						localBbox.top = y + snappedCoarseBbox.top;
					    localBbox.width = tileSize;
						localBbox.height = tileSize;

						localBbox.clampRight(snappedCoarseBbox.getRight());
						localBbox.clampBottom(snappedCoarseBbox.getBottom());

						boxes.push_back(localBbox);
					}
				}

				#pragma omp parallel for
				for (int boxId = 0; boxId < boxes.size(); boxId++) {

					BoundingBox localBbox = boxes[boxId];

					// Prepare coordinates map
					CoordinatesMap map;
					if (!map.build(panoramaSize, camPose, *(intrinsic.get()), localBbox)) {
						continue;
					}

					if (map.getBoundingBox().isEmpty()) 
					{
						continue;
					}


					#pragma omp critical 
					{
						globalBbox = globalBbox.unionWith(map.getBoundingBox());
					}
				}

				//Rare case ... When all boxes valid are after the loop
				if (globalBbox.left >= panoramaSize.first)
				{
					globalBbox.left -= panoramaSize.first;
				}

				globalBbox.width = std::min(globalBbox.width, panoramaSize.first);


				globalBbox.height = std::min(globalBbox.height, panoramaSize.second);

				// Load image and convert it to linear colorspace
				std::string imagePath = view.getImagePath();
				ALICEVISION_LOG_INFO("Load image with path " << imagePath);
				image::Image<image::RGBfColor> source;
				image::readImage(imagePath, source, image::EImageColorSpace::LINEAR);

				// Load metadata and update for output
				oiio::ParamValueList metadata = image::readImageMetadata(imagePath);
				metadata.push_back(oiio::ParamValue("AliceVision:offsetX", globalBbox.left));
				metadata.push_back(oiio::ParamValue("AliceVision:offsetY", globalBbox.top));
				metadata.push_back(oiio::ParamValue("AliceVision:panoramaWidth", panoramaSize.first));
				metadata.push_back(oiio::ParamValue("AliceVision:panoramaHeight", panoramaSize.second));
				metadata.push_back(oiio::ParamValue("AliceVision:tileSize", tileSize));

				// Images will be converted in Panorama coordinate system, so there will be no more extra orientation.
				metadata.remove("Orientation");
				metadata.remove("orientation");

				// Define output paths
				const std::string viewIdStr = std::to_string(view.getViewId());
				const std::string viewFilepath = (fs::path(outputDirectory) / (viewIdStr + ".exr")).string();
				const std::string maskFilepath = (fs::path(outputDirectory) / (viewIdStr + "_mask.exr")).string();
				const std::string weightFilepath = (fs::path(outputDirectory) / (viewIdStr + "_weight.exr")).string();

				// Create output images
				std::unique_ptr<oiio::ImageOutput> out_view = oiio::ImageOutput::create(viewFilepath);
				std::unique_ptr<oiio::ImageOutput> out_mask = oiio::ImageOutput::create(maskFilepath);
				std::unique_ptr<oiio::ImageOutput> out_weights = oiio::ImageOutput::create(weightFilepath);


				// Define output properties
				oiio::ImageSpec spec_view(globalBbox.width, globalBbox.height, 3, typeColor);
				oiio::ImageSpec spec_mask(globalBbox.width, globalBbox.height, 1, oiio::TypeDesc::UCHAR);
				oiio::ImageSpec spec_weights(globalBbox.width, globalBbox.height, 1, oiio::TypeDesc::HALF);

				spec_view.tile_width = tileSize;
				spec_view.tile_height = tileSize;
				spec_mask.tile_width = tileSize;
				spec_mask.tile_height = tileSize;
				spec_weights.tile_width = tileSize;
				spec_weights.tile_height = tileSize;
				spec_view.attribute("compression", "zips");
				spec_weights.attribute("compression", "zips");
				spec_mask.attribute("compression", "zips");
				spec_view.extra_attribs = metadata;
				spec_mask.extra_attribs = metadata;
				spec_weights.extra_attribs = metadata;

				out_view->open(viewFilepath, spec_view);
				out_mask->open(maskFilepath, spec_mask);
				out_weights->open(weightFilepath, spec_weights);

				GaussianPyramidNoMask pyramid(source.Width(), source.Height());
				if (!pyramid.process(source)) {
					ALICEVISION_LOG_ERROR("Problem creating pyramid.");
					continue;
				}

				boxes.clear();
				for (int y = 0; y < globalBbox.height; y += tileSize) 
				{
					for (int x = 0; x < globalBbox.width; x += tileSize) 
					{
						BoundingBox localBbox;
						localBbox.left = x + globalBbox.left;
						localBbox.top = y + globalBbox.top;
						localBbox.width = tileSize;
						localBbox.height = tileSize;
						boxes.push_back(localBbox);
					}
				}

				
				#pragma omp parallel for 
				for (int boxId = 0; boxId < boxes.size(); boxId++) 
				{
					BoundingBox localBbox = boxes[boxId];

					int x = localBbox.left - globalBbox.left;
					int y = localBbox.top - globalBbox.top;

					// Prepare coordinates map
					CoordinatesMap map;
					if (!map.build(panoramaSize, camPose, *(intrinsic.get()), localBbox)) 
					{
						continue;
					}

					// Warp image
					GaussianWarper warper;
					if (!warper.warp(map, pyramid, clampHalf)) {
						continue;
					}

					// Alpha mask
					aliceVision::image::Image<float> weights;
					if (!distanceToCenter(weights, map, intrinsic->w(), intrinsic->h())) {
						continue;
					}

					// Store
					#pragma omp critical 
					{
						out_view->write_tile(x, y, 0, oiio::TypeDesc::FLOAT, warper.getColor().data());
					}

					// Store
					#pragma omp critical 
					{
						out_mask->write_tile(x, y, 0, oiio::TypeDesc::UCHAR, warper.getMask().data());
					}

					// Store
					#pragma omp critical 
					{
						out_weights->write_tile(x, y, 0, oiio::TypeDesc::FLOAT, weights.data());
					}
				}

				out_view->close();
				out_mask->close();
				out_weights->close();
		}

		return EXIT_SUCCESS;
}
