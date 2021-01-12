// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <OpenImageIO/imagebufalgo.h>

/*SFMData*/
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

/*HDR Related*/
#include <aliceVision/hdr/rgbCurve.hpp>
#include <aliceVision/hdr/hdrMerge.hpp>
#include <aliceVision/hdr/DebevecCalibrate.hpp>
#include <aliceVision/hdr/GrossbergCalibrate.hpp>
#include <aliceVision/hdr/emorCurve.hpp>
#include <aliceVision/hdr/LaguerreBACalibration.hpp>

/*Command line parameters*/
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = boost::filesystem;

Eigen::Matrix3d getRotationForCode(int code)
{
    Eigen::Matrix3d R_metadata = Eigen::Matrix3d::Identity();

    switch(code)
    {
        case 0:
            R_metadata = Eigen::Matrix3d::Identity();
            break;
        case 3:
            R_metadata = Eigen::AngleAxisd(M_PI, Vec3(0, 0, 1));
            break;
        case 5:
            R_metadata = Eigen::AngleAxisd(-M_PI_2, Vec3(0, 0, 1));
            break;
        case 6:
            R_metadata = Eigen::AngleAxisd(M_PI_2, Vec3(0, 0, 1));
            break;
        default:
            ALICEVISION_LOG_ERROR("RotationCode flip is not valid ?");
            break;
    }

    return R_metadata;
}

int aliceVision_main(int argc, char* argv[])
{
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmInputDataFilename;
    std::string sfmOutputDataFilename;

    // Command line parameters
    po::options_description allParams("Prepare images set for use in panorama.\n"
                                      "AliceVision panoramaPrepareImages");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("output,o", po::value<std::string>(&sfmOutputDataFilename)->required(),
         "SfMData file output.");

    po::options_description optionalParams("Optional parameters");

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
         "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(optionalParams).add(logParams);

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

    system::Logger::get()->setLogLevel(verboseLevel);

    // Analyze path
    boost::filesystem::path path(sfmOutputDataFilename);
    std::string outputPath = path.parent_path().string();

    // Read sfm data
    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    size_t countImages = sfmData.getViews().size();
    if(countImages == 0)
    {
        ALICEVISION_LOG_ERROR("The input SfMData contains no input !");
        return EXIT_FAILURE;
    }

    // Make sure there is only one kind of image in dataset
    if(sfmData.getIntrinsics().size() > 2)
    {
        ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found)");
        return EXIT_FAILURE;
    }

    if(sfmData.getIntrinsics().size() > 1)
    {
        unsigned int refw = sfmData.getIntrinsics().begin()->second->w();
        unsigned int refh = sfmData.getIntrinsics().begin()->second->h();

        for(auto item : sfmData.getIntrinsics())
        {
            if(item.second->w() == refw && item.second->h() == refh)
            {
                continue;
            }

            if(item.second->w() == refh && item.second->h() == refw)
            {
                continue;
            }

            ALICEVISION_LOG_ERROR("Multiple intrinsics : Different kind of images in dataset.\n"
                                  << " - Current intrinsic: " << item.second->w() << "x" << item.second->w() << "\n"
                                  << " - Reference intrinsic: " << refw << "x" << refh);
            return EXIT_FAILURE;
        }
    }

    sfmData::Views& views = sfmData.getViews();

    // Read the flip values from metadata, or create it if necessary
    std::map<int, size_t> count_flips;
    for(const auto& v : views)
    {
        if(v.second->hasMetadata({"raw:flip"}))
        {
            std::string str = v.second->getMetadata({"raw:flip"});
            int flip_code = std::stoi(str);
            count_flips[flip_code]++;
        }
        else
        {
            // Add fake raw:flip if needed
            std::size_t width = v.second->getWidth();
            std::size_t height = v.second->getHeight();
            if(width > height)
            {
                v.second->addMetadata("raw:flip", "0");
                count_flips[0]++;
            }
            else
            {
                v.second->addMetadata("raw:flip", "5");
                count_flips[5]++;
            }
        }
    }

    if(count_flips.size() <= 1 && sfmData.getIntrinsics().size() == 2)
    {
        ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size()
                                                             << " found, count flips: " << count_flips.size() << ")");
        return EXIT_FAILURE;
    }

    // Decide which rotation is the most used
    int max_flip = -1;
    size_t max_count = 0;
    for(auto item : count_flips)
    {
        if(item.second > max_count)
        {
            max_flip = item.first;
            max_count = item.second;
        }
    }

    // Get the intrinsic of the best flip
    IndexT refIntrinsic = UndefinedIndexT;
    for(auto& v : views)
    {
        // Now, all views have "raw:flip"
        std::string str = v.second->getMetadata({"raw:flip"});
        int flip_code = std::stoi(str);

        if(flip_code == max_flip)
        {
            IndexT intid = v.second->getIntrinsicId();
            if(refIntrinsic != intid && refIntrinsic != UndefinedIndexT)
            {
                ALICEVISION_LOG_ERROR("Multiple intrinsics for the correct flip code !");
                return EXIT_FAILURE;
            }

            refIntrinsic = intid;
        }
    }

    for(sfmData::Intrinsics::iterator it = sfmData.getIntrinsics().begin(); it != sfmData.getIntrinsics().end(); ++it)
    {
        if(it->first != refIntrinsic)
        {
            it = sfmData.getIntrinsics().erase(it);
        }
    }

    for(auto& v : views)
    {
        // Now, all views have raw:flip
        std::string str = v.second->getMetadata({"raw:flip"});
        int flip_code = std::stoi(str);

        if(flip_code == max_flip)
        {
            continue;
        }

        if(refIntrinsic != v.second->getIntrinsicId())
        {
            v.second->setIntrinsicId(refIntrinsic);
        }

        Eigen::Matrix3d R = getRotationForCode(flip_code) * getRotationForCode(max_flip).transpose();
        Eigen::AngleAxisd aa(R);
        Eigen::Vector3d axis = aa.axis();
        double angle = aa.angle();

        if(axis(2) < -0.99)
        {
            axis(2) = 1.0;
            angle = -angle;
        }

        // Prepare output file
        image::Image<image::RGBfColor> output;
        boost::filesystem::path origImgPath(v.second->getImagePath());
        std::string origFilename = origImgPath.stem().string();
        std::string rotatedImagePath = (fs::path(outputPath) / (origFilename + ".exr")).string();
        oiio::ParamValueList metadata = image::readImageMetadata(v.second->getImagePath());

        // Read input file
        image::Image<image::RGBfColor> originalImage;

        image::ImageReadOptions options;
        options.outputColorSpace = image::EImageColorSpace::LINEAR;
        options.applyWhiteBalance = v.second->getApplyWhiteBalance();
        

        image::readImage(v.second->getImagePath(), originalImage, options);
        oiio::ImageBuf bufInput(
            oiio::ImageSpec(originalImage.Width(), originalImage.Height(), 3, oiio::TypeDesc::FLOAT),
            originalImage.data());

        // Find the correct operation to perform
        bool validTransform = false;
        if(axis(2) > 0.99)
        {
            if(std::abs(angle - M_PI_2) < 1e-4)
            {
                validTransform = true;
                output.resize(originalImage.Height(), originalImage.Width());
                oiio::ImageBuf bufOutput(oiio::ImageSpec(output.Width(), output.Height(), 3, oiio::TypeDesc::FLOAT),
                                         output.data());
                oiio::ImageBufAlgo::rotate90(bufOutput, bufInput);
            }
            else if(std::abs(angle + M_PI_2) < 1e-4)
            {
                validTransform = true;
                output.resize(originalImage.Height(), originalImage.Width());
                oiio::ImageBuf bufOutput(oiio::ImageSpec(output.Width(), output.Height(), 3, oiio::TypeDesc::FLOAT),
                                         output.data());
                oiio::ImageBufAlgo::rotate90(bufOutput, bufInput);
            }
            else if(std::abs(std::abs(angle) - M_PI) < 1e-4)
            {
                validTransform = true;
                output.resize(originalImage.Width(), originalImage.Height());
                oiio::ImageBuf bufOutput(oiio::ImageSpec(output.Width(), output.Height(), 3, oiio::TypeDesc::FLOAT),
                                         output.data());
                oiio::ImageBufAlgo::rotate180(bufOutput, bufInput);
            }
        }

        if(validTransform == false)
        {
            ALICEVISION_LOG_ERROR("Unrecognized intermediate transformation : ");
            ALICEVISION_LOG_ERROR(axis.transpose());
            ALICEVISION_LOG_ERROR(angle);
            return EXIT_FAILURE;
        }

        image::writeImage(rotatedImagePath, output, image::EImageColorSpace::AUTO, metadata);
        v.second->setWidth(output.Width());
        v.second->setHeight(output.Height());
        v.second->setImagePath(rotatedImagePath);
    }

    // Export output sfmData
    if(!sfmDataIO::Save(sfmData, sfmOutputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("Can not save output sfm file at " << sfmOutputDataFilename);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
