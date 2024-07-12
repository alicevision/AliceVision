// This file is part of the AliceVision project.
// Copyright (c) 2023 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/cmdline/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/camera/camera.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/image/all.hpp>

#include <boost/program_options.hpp>

#include <string>
#include <sstream>
#include <fstream>
#include <memory>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

namespace po = boost::program_options;

using namespace aliceVision;
using namespace aliceVision::camera;

std::string toNuke(std::shared_ptr<camera::IntrinsicScaleOffsetDisto> intrinsic)
{
    std::stringstream ss;

    const double focal = intrinsic->getFocalLengthMM();
    const double sensorWidth = intrinsic->sensorWidth();
    const double sensorHeight = intrinsic->sensorHeight();
    
    const auto undistortion = intrinsic->getUndistortion();    
    const std::vector<double>& params = undistortion->getParameters();
    const auto& size = undistortion->getSize();
    const Vec2 offset = undistortion->getScaledOffset();
    const double pa = undistortion->getPixelAspectRatio();
    const double updatedSensorHeight = (undistortion->isDesqueezed())?sensorHeight:sensorHeight/pa;

    switch (undistortion->getType())
    {
        case EUNDISTORTION::UNDISTORTION_RADIALK3:
            ss << "LensDistortion2 {" << "\n"
               << "\n"
               << " distortionModelPreset Custom" << "\n"
               << " distortionModelPreset \"3DEqualizer/3DE4 Anamorphic - Standard, Degree 4\""
               << " distortionNumerator0 " << params[0] << "\n"
               << "\n"
               << " distortionNumerator1 " << params[1] << "\n"
               << " lens Anamorphic"
               << " distortionNumerator2 " << params[2] << "\n"
               << " centre {" << offset[0] << " " << -offset[1] << "}" << "\n"
               << " output Undistort" << "\n"
               << " distortionScalingType Format" << "\n"
               << " distortionScalingFormat \"" << size(0) << " " << size(1) << " 0 0 " << size(0) << " " << size(1) << " 1 AV_undist_fmt \""
               << "\n"
               << "\n"
               << " distortionOrder {3 0}" << "\n"
               << " normalisationType Diagonal" << "\n"
               << " distortInFisheyeSpace false" << "\n"
               << "}"
               << "\n";
            break;
        case EUNDISTORTION::UNDISTORTION_3DEANAMORPHIC4:
            ss  << "LD_3DE4_Anamorphic_Standard_Degree_4 {\n"
                << "direction undistort \n"
                << "tde4_focal_length_cm " << focal << " \n" 
                << "tde4_custom_focus_distance_cm 1.0 \n"
                << "tde4_filmback_width_cm " << sensorWidth << " \n"
                << "tde4_filmback_height_cm " << updatedSensorHeight << " \n"
                << "tde4_lens_center_offset_x_cm 0.0000000 \n"
                << "tde4_lens_center_offset_y_cm 0.0000000 \n"
                << "tde4_pixel_aspect " << pa << " \n"
                << "field_of_view_xa_unit 0.0000000 \n"
                << "field_of_view_xb_unit 1.0000000 \n"
                << "field_of_view_ya_unit 0.0000000 \n"
                << "field_of_view_yb_unit 1.0000000 \n"
                << "Cx02_Degree_2 " << params[0] << " \n"
                << "Cy02_Degree_2 " << params[1] << " \n"
                << "Cx22_Degree_2 " << params[2] << " \n"
                << "Cy22_Degree_2 " << params[3] << " \n"
                << "Cx04_Degree_4 " << params[4] << " \n"
                << "Cy04_Degree_4 " << params[5] << " \n"
                << "Cx24_Degree_4 " << params[6] << " \n"
                << "Cy24_Degree_4 " << params[7] << " \n"
                << "Cx44_Degree_4 " << params[8] << " \n"
                << "Cy44_Degree_4 " << params[9] << " \n"
                << "Lens_Rotation " << radianToDegree(params[10]) << " \n"
                << "Squeeze_X " << params[11] << "\n"
                << "Squeeze_Y " << params[12] << "\n"
                << "}\n";
            break;
        case EUNDISTORTION::UNDISTORTION_3DERADIAL4:
            ss  << "LD_3DE4_Radial_Standard_Degree_4 {\n"
                << "direction undistort \n"
                << "tde4_focal_length_cm " << focal << " \n" 
                << "tde4_custom_focus_distance_cm 1.0 \n"
                << "tde4_filmback_width_cm " << sensorWidth << " \n"
                << "tde4_filmback_height_cm " << updatedSensorHeight << " \n"
                << "tde4_lens_center_offset_x_cm 0.0000000 \n"
                << "tde4_lens_center_offset_y_cm 0.0000000 \n"
                << "tde4_pixel_aspect " << pa << " \n"
                << "field_of_view_xa_unit 0.0000000 \n"
                << "field_of_view_xb_unit 1.0000000 \n"
                << "field_of_view_ya_unit 0.0000000 \n"
                << "field_of_view_yb_unit 1.0000000 \n"
                << "Distortion_Degree_2 " << params[0] << " \n"
                << "U_Degree_2 " << params[1] << " \n"
                << "V_Degree_2 " << params[2] << " \n"
                << "Quartic_Distortion_Degree_4 " << params[3] << " \n"
                << "U_Degree_4 " << params[4] << " \n"
                << "V_Degree_4 " << params[5] << " \n"
                << "Phi_Cylindric_Direction " << params[6] << " \n"
                << "B_Cylindric_Bending " << params[7] << " \n"
                << "}\n";
            break;
        case EUNDISTORTION::UNDISTORTION_3DECLASSICLD:
            ss  << "LD_3DE4_Radial_Standard_Degree_4 {\n"
                << "direction undistort \n"
                << "tde4_focal_length_cm " << focal << " \n" 
                << "tde4_custom_focus_distance_cm 1.0 \n"
                << "tde4_filmback_width_cm " << sensorWidth << " \n"
                << "tde4_filmback_height_cm " << updatedSensorHeight << " \n"
                << "tde4_lens_center_offset_x_cm 0.0000000 \n"
                << "tde4_lens_center_offset_y_cm 0.0000000 \n"
                << "tde4_pixel_aspect " << pa << " \n"
                << "field_of_view_xa_unit 0.0000000 \n"
                << "field_of_view_xb_unit 1.0000000 \n"
                << "field_of_view_ya_unit 0.0000000 \n"
                << "field_of_view_yb_unit 1.0000000 \n"
                << "Distortion " << params[0] << " \n"
                << "Anamorphic_Squeeze " << params[1] << " \n"
                << "Curvature_X " << params[2] << " \n"
                << "Curvature_Y " << params[3] << " \n"
                << "Quartic_Distortion " << params[4] << " \n"
                << "}\n";
            break;
        default:
            ALICEVISION_THROW_ERROR(
              "Unsupported intrinsic type for conversion to Nuke LensDistortion node: " << EUNDISTORTION_enumToString(undistortion->getType()));
    }

    return ss.str();
}

void toSTMap(image::Image<image::RGBAfColor>& stmap,
             std::shared_ptr<camera::IntrinsicScaleOffsetDisto> intrinsic,
             bool distort,
             const oiio::ROI& roi = oiio::ROI())
{
    int widthRoi = intrinsic->w();
    int heightRoi = intrinsic->h();
    int xOffset = 0;
    int yOffset = 0;
    if (roi.defined())
    {
        widthRoi = roi.width();
        heightRoi = roi.height();
        xOffset = roi.xbegin;
        yOffset = roi.ybegin;
    }

    stmap.resize(widthRoi, heightRoi, true, image::RGBAfColor(0.0f));

#pragma omp parallel for
    for (int i = 0; i < heightRoi; ++i)
    {
        for (int j = 0; j < widthRoi; ++j)
        {
            const Vec2 pix((j + xOffset), (i + yOffset));

            const Vec2 disto_pix = distort ? intrinsic->getUndistortedPixel(pix) : intrinsic->getDistortedPixel(pix);

            stmap(i, j).b() = disto_pix[0] / (static_cast<float>(intrinsic->w()) - 1.0f);
            stmap(i, j).a() = (static_cast<float>(intrinsic->h()) - 1.0f - disto_pix[1]) / (static_cast<float>(intrinsic->h()) - 1.0f);
            stmap(i, j).r() = stmap(i, j).b();
            stmap(i, j).g() = stmap(i, j).a();
        }
    }
}

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string outputFilePath;
    bool exportLensGridsUndistorted = true;
    bool exportNukeNode = true;
    bool exportSTMaps = true;

    // Command line parameters
    // clang-format off
    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
        ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(), 
         "SfMData file input.")
        ("output,o", po::value<std::string>(&outputFilePath)->required(), 
         "Output directory.");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("exportNukeNode", po::value<bool>(&exportNukeNode)->default_value(exportLensGridsUndistorted),
         "Export Nuke node as FILE.nk.")
        ("exportSTMaps", po::value<bool>(&exportSTMaps)->default_value(exportLensGridsUndistorted),
         "Export STMaps.")
        ("exportLensGridsUndistorted,e", po::value<bool>(&exportLensGridsUndistorted)->default_value(exportLensGridsUndistorted),
         "Export lens grids undistorted for validation.");
    // clang-format on

    CmdLine cmdline("AliceVision exportDistortion");
    cmdline.add(requiredParams);
    cmdline.add(optionalParams);

    if (!cmdline.execute(argc, argv))
    {
        return EXIT_FAILURE;
    }

    sfmData::SfMData sfmData;
    if (!sfmDataIO::load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::VIEWS | sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    for (const auto& [intrinsicId, intrinsicPtr] : sfmData.getIntrinsics())
    {
        ALICEVISION_LOG_INFO("Exporting distortion for intrinsic " << intrinsicId);

        const auto intrinsicDisto = std::dynamic_pointer_cast<IntrinsicScaleOffsetDisto>(intrinsicPtr);
        if (!intrinsicDisto)
            continue;

        const auto undistortion = intrinsicDisto->getUndistortion();
        if (!undistortion)
            continue;

        if (exportNukeNode)
        {
            ALICEVISION_LOG_INFO("Computing Nuke LensDistortion node");

            const std::string nukeNodeStr = toNuke(intrinsicDisto);

            ALICEVISION_LOG_INFO("Writing Nuke LensDistortion node in " << intrinsicId << ".nk");
            std::stringstream ss;
            ss << outputFilePath << "/nukeLensDistortion_" << intrinsicId << ".nk";
            std::ofstream of(ss.str());
            of << nukeNodeStr;
            of.close();
        }

        if (exportSTMaps)
        {
            ALICEVISION_LOG_INFO("Computing STMaps");

            image::Image<image::RGBAfColor> stmap_distort;
            toSTMap(stmap_distort, intrinsicDisto, true);

            image::Image<image::RGBAfColor> stmap_undistort;
            toSTMap(stmap_undistort, intrinsicDisto, false);

            {
                ALICEVISION_LOG_INFO("Export distortion STMap: stmap_" << intrinsicId << "_distort.exr");
                std::stringstream ss;
                ss << outputFilePath << "/stmap_" << intrinsicId << "_distort.exr";
                image::writeImage(ss.str(), stmap_distort, image::ImageWriteOptions().storageDataType(image::EStorageDataType::Float));
            }

            {
                ALICEVISION_LOG_INFO("Export undistortion STMap: stmap_" << intrinsicId << "_undistort.exr");
                std::stringstream ss;
                ss << outputFilePath << "/stmap_" << intrinsicId << "_undistort.exr";
                image::writeImage(ss.str(), stmap_undistort, image::ImageWriteOptions().storageDataType(image::EStorageDataType::Float));
            }
        }

        if (exportLensGridsUndistorted)
        {
            for (const auto& pv : sfmData.getViews())
            {
                if (pv.second->getIntrinsicId() == intrinsicId)
                {
                    // Read image
                    image::Image<image::RGBfColor> image;
                    std::string path = pv.second->getImageInfo()->getImagePath();
                    image::readImage(path, image, image::EImageColorSpace::LINEAR);
                    oiio::ParamValueList metadata = image::readImageMetadata(path);

                    // Undistort
                    image::Image<image::RGBfColor> image_ud;
                    camera::UndistortImage(image, intrinsicPtr.get(), image_ud, image::FBLACK, false);

                    // Save undistorted
                    std::stringstream ss;
                    ss << outputFilePath << "/lensgrid_" << pv.first << "_undistort.exr";
                    ALICEVISION_LOG_INFO("Export lens grid undistorted (Source image: " << path << ", Undistorted image: " << ss.str() << ").");
                    image::writeImage(ss.str(), image_ud, image::ImageWriteOptions(), metadata);
                }
            }
        }
    }

    return EXIT_SUCCESS;
}
