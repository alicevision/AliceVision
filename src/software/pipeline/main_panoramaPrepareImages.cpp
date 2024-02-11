// This file is part of the AliceVision project.
// Copyright (c) 2019 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/image/all.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/cmdline/cmdline.hpp>
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
#include <filesystem>
#include <sstream>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;

namespace po = boost::program_options;
namespace fs = std::filesystem;

Eigen::Matrix3d getRotationForCode(int code)
{
    Eigen::Matrix3d R_metadata = Eigen::Matrix3d::Identity();

    switch (code)
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
    std::string sfmInputDataFilename;
    std::string sfmOutputDataFilename;

    // Command line parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilename)->required(),
         "SfMData file input.")
        ("output,o", po::value<std::string>(&sfmOutputDataFilename)->required(),
         "SfMData file output.");
    // clang-format on

    CmdLine cmdline("Prepares images for use in the panorama pipeline "
                    "by correcting inconsistent orientations caused by the camera being in zenith or nadir position.\n"
                    "AliceVision panoramaPrepareImages");
    cmdline.add(requiredParams);
    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    // Analyze path
    fs::path path(sfmOutputDataFilename);
    std::string outputPath = path.parent_path().string();

    // Read sfm data
    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmInputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilename << "' cannot be read.");
        return EXIT_FAILURE;
    }

    size_t countImages = sfmData.getViews().size();
    if (countImages == 0)
    {
        ALICEVISION_LOG_ERROR("The input SfMData contains no input !");
        return EXIT_FAILURE;
    }

    // Make sure there is only one kind of image in dataset
    if (sfmData.getIntrinsics().size() > 2)
    {
        ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found)");
        return EXIT_FAILURE;
    }

    if (sfmData.getIntrinsics().size() > 1)
    {
        const unsigned int refw = sfmData.getIntrinsics().begin()->second->w();
        const unsigned int refh = sfmData.getIntrinsics().begin()->second->h();

        for (const auto& item : sfmData.getIntrinsics())
        {
            if (item.second->w() == refw && item.second->h() == refh)
            {
                continue;
            }

            if (item.second->w() == refh && item.second->h() == refw)
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
    for (auto& v : views)
    {
        if (v.second->getImage().hasMetadata({"raw:flip"}))
        {
            std::string str = v.second->getImage().getMetadata({"raw:flip"});
            int flip_code = std::stoi(str);
            count_flips[flip_code]++;
        }
        else
        {
            // Add fake raw:flip if needed
            std::size_t width = v.second->getImage().getWidth();
            std::size_t height = v.second->getImage().getHeight();
            if (width > height)
            {
                v.second->getImage().addMetadata("raw:flip", "0");
                count_flips[0]++;
            }
            else
            {
                v.second->getImage().addMetadata("raw:flip", "5");
                count_flips[5]++;
            }
        }
    }

    if (count_flips.size() <= 1 && sfmData.getIntrinsics().size() == 2)
    {
        ALICEVISION_LOG_ERROR("Only one intrinsic allowed (" << sfmData.getIntrinsics().size() << " found, count flips: " << count_flips.size()
                                                             << ")");
        return EXIT_FAILURE;
    }

    // Decide which rotation is the most used
    int max_flip = -1;
    size_t max_count = 0;
    for (const auto& item : count_flips)
    {
        if (item.second > max_count)
        {
            max_flip = item.first;
            max_count = item.second;
        }
    }

    // Get the intrinsic of the best flip
    IndexT refIntrinsic = UndefinedIndexT;
    for (const auto& v : views)
    {
        // Now, all views have "raw:flip"
        const std::string str = v.second->getImage().getMetadata({"raw:flip"});
        const int flip_code = std::stoi(str);

        if (flip_code == max_flip)
        {
            const IndexT intid = v.second->getIntrinsicId();
            if (refIntrinsic != intid && refIntrinsic != UndefinedIndexT)
            {
                ALICEVISION_LOG_ERROR("Multiple intrinsics for the correct flip code !");
                return EXIT_FAILURE;
            }

            refIntrinsic = intid;
        }
    }

    // Remove all other intrinsics
    for (sfmData::Intrinsics::iterator it = sfmData.getIntrinsics().begin(); it != sfmData.getIntrinsics().end(); ++it)
    {
        if (it->first != refIntrinsic)
        {
            it = sfmData.getIntrinsics().erase(it);
        }
    }

    for (auto& v : views)
    {
        // Now, all views have raw:flip
        const std::string str = v.second->getImage().getMetadata({"raw:flip"});
        const int flip_code = std::stoi(str);

        if (flip_code == max_flip)
        {
            continue;
        }

        if (refIntrinsic != v.second->getIntrinsicId())
        {
            v.second->setIntrinsicId(refIntrinsic);
        }

        const Eigen::Matrix3d R = getRotationForCode(flip_code) * getRotationForCode(max_flip).transpose();
        const Eigen::AngleAxisd aa(R);
        Eigen::Vector3d axis = aa.axis();
        double angle = aa.angle();

        if (axis(2) < -0.99)
        {
            axis(2) = 1.0;
            angle = -angle;
        }

        // Prepare output file
        image::Image<image::RGBfColor> output;
        const fs::path origImgPath(v.second->getImage().getImagePath());
        const std::string origFilename = origImgPath.stem().string();
        const std::string rotatedImagePath = (fs::path(outputPath) / (origFilename + ".exr")).string();
        oiio::ParamValueList metadata = image::readImageMetadata(v.second->getImage().getImagePath());

        // Read input file
        image::Image<image::RGBfColor> originalImage;

        image::ImageReadOptions options;
        options.workingColorSpace = image::EImageColorSpace::LINEAR;
        options.rawColorInterpretation = image::ERawColorInterpretation_stringToEnum(v.second->getImage().getRawColorInterpretation());
        options.colorProfileFileName = v.second->getImage().getColorProfileFileName();

        image::readImage(v.second->getImage().getImagePath(), originalImage, options);
        oiio::ImageBuf bufInput(oiio::ImageSpec(originalImage.width(), originalImage.height(), 3, oiio::TypeDesc::FLOAT), originalImage.data());

        // Find the correct operation to perform
        bool validTransform = false;
        if (axis(2) > 0.99)
        {
            if (std::abs(angle - M_PI_2) < 1e-4)
            {
                validTransform = true;
                output.resize(originalImage.height(), originalImage.width());
                oiio::ImageBuf bufOutput(oiio::ImageSpec(output.width(), output.height(), 3, oiio::TypeDesc::FLOAT), output.data());
                oiio::ImageBufAlgo::rotate90(bufOutput, bufInput);
            }
            else if (std::abs(angle + M_PI_2) < 1e-4)
            {
                validTransform = true;
                output.resize(originalImage.height(), originalImage.width());
                oiio::ImageBuf bufOutput(oiio::ImageSpec(output.width(), output.height(), 3, oiio::TypeDesc::FLOAT), output.data());
                oiio::ImageBufAlgo::rotate90(bufOutput, bufInput);
            }
            else if (std::abs(std::abs(angle) - M_PI) < 1e-4)
            {
                validTransform = true;
                output.resize(originalImage.width(), originalImage.height());
                oiio::ImageBuf bufOutput(oiio::ImageSpec(output.width(), output.height(), 3, oiio::TypeDesc::FLOAT), output.data());
                oiio::ImageBufAlgo::rotate180(bufOutput, bufInput);
            }
        }

        if (validTransform == false)
        {
            ALICEVISION_LOG_ERROR("Unrecognized intermediate transformation : ");
            ALICEVISION_LOG_ERROR(axis.transpose());
            ALICEVISION_LOG_ERROR(angle);
            return EXIT_FAILURE;
        }

        image::writeImage(rotatedImagePath, output, image::ImageWriteOptions(), metadata);
        v.second->getImage().setWidth(output.width());
        v.second->getImage().setHeight(output.height());
        v.second->getImage().setImagePath(rotatedImagePath);
    }

    // Export output sfmData
    if (!sfmDataIO::save(sfmData, sfmOutputDataFilename, sfmDataIO::ESfMData::ALL))
    {
        ALICEVISION_LOG_ERROR("Can not save output sfm file at " << sfmOutputDataFilename);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
