// This file is part of the AliceVision project.
// Copyright (c) 2021 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <Eigen/Dense>

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <sstream>

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/system/Timer.hpp>

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/image/all.hpp>
#include <OpenImageIO/imagebufalgo.h>


#include <aliceVision/calibration/distortionEstimation.hpp>
#include <aliceVision/calibration/checkerDetector.hpp>

#include <chrono>
#include <vector>



// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace aliceVision;


image::Image<image::RGBColor> undistort(Vec2 & offset, const std::shared_ptr<camera::Pinhole> & camera, const image::Image<image::RGBColor> & source) 
{
    const double w = source.Width();
    const double h = source.Height();

    double minx = w;
    double maxx = 0;
    double miny = h;
    double maxy = 0;

    for (int i = 0; i < h; i++) 
    {
        for (int j = 0; j < w; j++)
        {
            Vec2 pos = camera->get_ud_pixel(Vec2(j, i));
            minx = std::min(minx, pos.x());
            maxx = std::max(maxx, pos.x());
            miny = std::min(miny, pos.y());
            maxy = std::max(maxy, pos.y());
        }
    }

    const int width = maxx - minx + 1;
    const int height = maxy - miny + 1;

    image::Image<image::RGBColor> result(width, height, true, image::RGBColor(0, 0, 0));

    const image::Sampler2d<image::SamplerLinear> sampler;

    for (int i = 0; i < height; i++) 
    {
        const double y = miny + double(i);

        for (int j = 0; j < width; j++)
        {
            const double x = minx + double(j);

            const Vec2 pos(x, y);
            const Vec2 dist = camera->get_d_pixel(pos);

            if (dist.x() < 0 || dist.x() >= source.Width()) continue;
            if (dist.y() < 0 || dist.y() >= source.Height()) continue;
            
            const image::RGBColor c = sampler(source, dist.y(), dist.x());
            
            result(i, j) = c;
        }
    }

    offset.x() = minx;
    offset.y() = miny;

    return result;
}

image::Image<image::RGBfColor> undistortSTMAP(Vec2 & offset, const std::shared_ptr<camera::Pinhole> & camera, const image::Image<image::RGBColor> & source) 
{
    const double w = source.Width();
    const double h = source.Height();

    double minx = w;
    double maxx = 0;
    double miny = h;
    double maxy = 0;

    for (int i = 0; i < h; i++) 
    {
        for (int j = 0; j < w; j++)
        {
            const Vec2 pos = camera->get_ud_pixel(Vec2(j, i));
            minx = std::min(minx, pos.x());
            maxx = std::max(maxx, pos.x());
            miny = std::min(miny, pos.y());
            maxy = std::max(maxy, pos.y());
        }
    }

    const int width = maxx - minx + 1;
    const int height = maxy - miny + 1;

    image::Image<image::RGBfColor> result(width, height, true, image::FBLACK);

    const image::Sampler2d<image::SamplerLinear> sampler;

    for (int i = 0; i < height; i++) 
    {
        const double y = miny + double(i);

        for (int j = 0; j < width; j++)
        {
            const double x = minx + double(j);

            const Vec2 pos(x, y);
            const Vec2 dist = camera->get_d_pixel(pos);

            if (dist.x() < 0 || dist.x() >= source.Width()) continue;
            if (dist.y() < 0 || dist.y() >= source.Height()) continue;

            result(i, j).r() = dist.x() / (float(width) - 1);
            result(i, j).g() = (float(height) - 1.0f - dist.y()) / (float(height) - 1.0f);
            result(i, j).b() = 0.0f;
        }
    }

    offset.x() = minx;
    offset.y() = miny;

    return result;
}

bool retrieveLines(std::vector<calibration::LineWithPoints> & lineWithPoints, const image::Image<image::RGBColor> & input, const std::string & checkerImagePath)
{
    calibration::CheckerDetector detect;
    if (!detect.process(input))
    {
        return false;
    }

    if(!checkerImagePath.empty())
    {
        image::Image<image::RGBColor> drawing = input;
        detect.drawCheckerBoard(drawing);
        image::writeImage(checkerImagePath, drawing, image::EImageColorSpace::NO_CONVERSION);
    }

    lineWithPoints.clear();

    std::vector<calibration::CheckerDetector::CheckerBoardCorner> corners = detect.getCorners();
    for (auto & b : detect.getBoards())
    {
        // Create horizontal lines
        for (int i = 0; i < b.rows(); i ++)
        {
            //Random init
            calibration::LineWithPoints line;
            line.angle = M_PI_4;
            line.dist = 1;
            line.horizontal = true;
            line.index = i;

            for (int j = 0; j < b.cols(); j++)
            {
                const IndexT idx = b(i, j);
                if (idx == UndefinedIndexT) continue;

                const calibration::CheckerDetector::CheckerBoardCorner& p = corners[idx];
                line.points.push_back(p.center);
            }

            //Check we don't have a too small line which won't be easy to estimate
            if (line.points.size() < 10) continue;
            lineWithPoints.push_back(line);
        }

        // Create vertical lines
        for (int j = 0; j < b.cols(); j++)
        {
            calibration::LineWithPoints line;
            line.angle = M_PI_4;
            line.dist = 1;
            line.horizontal = false;
            line.index = j;

            for (int i = 0; i < b.rows(); i++)
            {
                const IndexT idx = b(i, j);
                if (idx == UndefinedIndexT) continue;

                const calibration::CheckerDetector::CheckerBoardCorner& p = corners[idx];
                line.points.push_back(p.center);
            }

            //Check we don't have a too small line which won't be easy to estimate
            if (line.points.size() < 10) continue;

            lineWithPoints.push_back(line);
        }
    }

    if (lineWithPoints.size() < 2)
    {
        return false;
    }

    return true;
}

template <class T>
bool estimateDistortionK1(std::shared_ptr<camera::Pinhole> & camera, calibration::Statistics & statistics, std::vector<T> & items)
{
    std::vector<bool> locksDistortions = {true};

    //Everything locked except lines parameters
    locksDistortions[0] = true;
    if (!calibration::estimate(camera, statistics, items, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax distortion 1st order
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, statistics, items, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    return true;
}

template <class T>
bool estimateDistortionK3(std::shared_ptr<camera::Pinhole> & camera, calibration::Statistics & statistics, std::vector<T> & items)
{
    std::vector<bool> locksDistortions = {true, true, true};

    //Everything locked except lines parameters
    locksDistortions[0] = true;
    if (!calibration::estimate(camera, statistics, items, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax distortion 1st order
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, statistics, items, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    locksDistortions[1] = false;
    locksDistortions[2] = false;
    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    return true;
}

template <class T>
bool estimateDistortion3DER4(std::shared_ptr<camera::Pinhole> & camera, calibration::Statistics & statistics, std::vector<T> & items)
{
    std::vector<bool> locksDistortions = {true, true, true, true, true, true};

    //Everything locked except lines parameters
    locksDistortions[0] = true;
    if (!calibration::estimate(camera, statistics, items, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax distortion 1st order
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, statistics, items, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    locksDistortions[1] = false;
    locksDistortions[2] = false;
    locksDistortions[3] = false;
    locksDistortions[4] = false;
    locksDistortions[5] = false;
    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    return true;
}

template <class T>
bool estimateDistortion3DEA4(std::shared_ptr<camera::Pinhole> & camera, calibration::Statistics & statistics, std::vector<T> & items)
{
    std::shared_ptr<camera::Pinhole> simpleCamera = std::make_shared<camera::PinholeRadialK1>(camera->w(), camera->h(), camera->getScale()[0], camera->getScale()[1], camera->getOffset()[0], camera->getOffset()[1], 0.0);
    if (!estimateDistortionK1(simpleCamera, statistics, items))
    {
        return false;
    }

    std::vector<bool> locksDistortions = {true, true, true, true};

    //Relax distortion all orders
    locksDistortions[0] = false;
    locksDistortions[1] = false;
    locksDistortions[2] = false;
    locksDistortions[3] = false;

    const double k1 = simpleCamera->getDistortionParams()[0];
    camera->setDistortionParams({k1,k1,k1,k1});

    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    return true;
}

template <class T>
bool estimateDistortion3DELD(std::shared_ptr<camera::Pinhole> & camera, calibration::Statistics & statistics, std::vector<T> & items)
{
    std::vector<double> params = camera->getDistortionParams();
    params[0] = 0.0;
    params[1] = M_PI_2;
    params[2] = 0.0;
    params[3] = 0.0;
    params[4] = 0.0;
    camera->setDistortionParams(params);

    std::vector<bool> locksDistortions = {true, true, true, true, true};

    //Everything locked except lines parameters
    locksDistortions[0] = true;
    if (!calibration::estimate(camera, statistics, items, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax distortion 1st order
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, statistics, items, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    locksDistortions[1] = true;
    locksDistortions[2] = false;
    locksDistortions[3] = false;
    locksDistortions[4] = true;
    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    locksDistortions[1] = false;
    locksDistortions[2] = false;
    locksDistortions[3] = false;
    locksDistortions[4] = false;
    if (!calibration::estimate(camera, statistics, items, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return false;
    }

    return true;
}

bool generatePoints(std::vector<calibration::PointPair> & points, const std::shared_ptr<camera::Pinhole> & camera, const std::vector<calibration::LineWithPoints> & lineWithPoints)
{
    for (auto & l : lineWithPoints)
    {
        for (auto & pt : l.points)
        {
            calibration::PointPair pp;

            //Everything is reverted in the given model (distorting equals to undistorting)
            pp.undistortedPoint = camera->get_d_pixel(pt);
            pp.distortedPoint = pt;

            const double err = (camera->get_ud_pixel(pp.undistortedPoint) - pp.distortedPoint).norm();
            if (err > 1e-3)
            {
                continue;
            }

            points.push_back(pp);
        }
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::vector<std::string> lensGridFilepaths;
    std::string sfmOutputDataFilepath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    // Command line parameters
    po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaInit");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(), "SfMData file input.")
    ("lensGrid", po::value<std::vector<std::string>>(&lensGridFilepaths)->multitoken(), "Lens Grid images.")
    ("outSfMData,o", po::value<std::string>(&sfmOutputDataFilepath)->required(), "SfMData file output.")
    ;

    po::options_description logParams("Log parameters");
    logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
        "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(requiredParams).add(logParams);

    // Parse command line
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

    sfmData::SfMData sfmData;
    if(!sfmDataIO::Load(sfmData, sfmInputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The input SfMData file '" << sfmInputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    sfmData::Intrinsics& intrinsics = sfmData.getIntrinsics();

    if(intrinsics.empty())
    {
        ALICEVISION_LOG_ERROR("No valid intrinsics in input: '" << sfmInputDataFilepath << "'.");
        return EXIT_FAILURE;
    }

    if(intrinsics.size() > 1)
    {
        ALICEVISION_LOG_ERROR("This Lens Distortion Calibration software does not support the calibration of multiple intrinsics for now.");
        return EXIT_FAILURE;
    }

    // Analyze path
    boost::filesystem::path path(sfmOutputDataFilepath);
    std::string outputPath = path.parent_path().string();

    if(lensGridFilepaths.empty())
    {
        ALICEVISION_LOG_WARNING("No lens grid image to perform the lens calibration.");
        if(!sfmDataIO::Save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
        {
            ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be read.");
            return EXIT_FAILURE;
        }
        return EXIT_SUCCESS;
    }

    for(auto intrinsicIt: intrinsics)
    {
        std::shared_ptr<camera::IntrinsicBase>& intrinsicPtr = intrinsicIt.second;
        const std::string intrinsicIdStr = std::to_string(intrinsicIt.first);
        std::shared_ptr<camera::Pinhole> cameraPinhole = std::dynamic_pointer_cast<camera::Pinhole>(intrinsicPtr);
        if(!cameraPinhole)
        {
            ALICEVISION_LOG_ERROR("Only work for pinhole cameras");
            return EXIT_FAILURE;
        }
        ALICEVISION_LOG_INFO("Processing Intrinsic " << intrinsicIt.first);

        std::vector<double> params = cameraPinhole->getDistortionParams();
        for(std::size_t i = 0; i < params.size(); i++)
            params[i] = 0.0;
        cameraPinhole->setDistortionParams(params);

        std::vector<calibration::LineWithPoints> allLineWithPoints;
        std::vector<std::vector<calibration::LineWithPoints>> lineWithPointsPerImage;

        for(const std::string lensGridFilepath : lensGridFilepaths)
        {
            //Check pixel ratio
            double pixelRatio = 1.0; // view->getDoubleMetadata({"PixelAspectRatio"}); // TODO
            if (pixelRatio < 0.0)
            {
                pixelRatio = 1.0;
            }

            //Read image
            image::Image<image::RGBColor> input;
            image::readImage(lensGridFilepath, input, image::EImageColorSpace::SRGB);

            if (pixelRatio != 1.0)
            {
                // if pixel are not squared, convert the image for lines extraction
                const double w = input.Width();
                const double h = input.Height();
                const double nw = w;
                const double nh = h / pixelRatio;
                image::Image<image::RGBColor> resizedInput(nw, nh);

                const oiio::ImageSpec imageSpecResized(nw, nh, 3, oiio::TypeDesc::UCHAR);
                const oiio::ImageSpec imageSpecOrigin(w, h, 3, oiio::TypeDesc::UCHAR);

                const oiio::ImageBuf inBuf(imageSpecOrigin, input.data());
                oiio::ImageBuf outBuf(imageSpecResized, resizedInput.data());

                oiio::ImageBufAlgo::resize(outBuf, inBuf);
                input.swap(resizedInput);
            }

            const Vec2 originalScale = cameraPinhole->getScale();
        
            const double w = input.Width();
            const double h = input.Height();
            if(w != cameraPinhole->w())
            {
                ALICEVISION_THROW_ERROR("Inconsistant size between the image and the camera intrinsics (image: "
                                        << w << "x" << h << ", camera: " << cameraPinhole->w() << "x" << cameraPinhole->h());
            }

            fs::copy_file(lensGridFilepath, fs::path(outputPath) / fs::path(lensGridFilepath).filename(),
                          fs::copy_option::overwrite_if_exists);

            const std::string checkerImagePath =
                (fs::path(outputPath) / fs::path(lensGridFilepath).stem()).string() + "_checkerboard.exr";

            // Retrieve lines
            std::vector<calibration::LineWithPoints> lineWithPoints;
            if (!retrieveLines(lineWithPoints, input, checkerImagePath))
            {
                ALICEVISION_LOG_ERROR("Impossible to extract the checkerboards lines");
                continue;
            }
            lineWithPointsPerImage.push_back(lineWithPoints);
            allLineWithPoints.insert(allLineWithPoints.end(), lineWithPoints.begin(), lineWithPoints.end());
        }

        calibration::Statistics statistics;

        // Estimate distortion
        if(std::dynamic_pointer_cast<camera::PinholeRadialK1>(cameraPinhole))
        {
            if (!estimateDistortionK1(cameraPinhole, statistics, allLineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else if(std::dynamic_pointer_cast<camera::PinholeRadialK3>(cameraPinhole))
        {
            if (!estimateDistortionK3(cameraPinhole, statistics, allLineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else if(std::dynamic_pointer_cast<camera::Pinhole3DERadial4>(cameraPinhole))
        {
            if (!estimateDistortion3DER4(cameraPinhole, statistics, allLineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else if(std::dynamic_pointer_cast<camera::Pinhole3DEAnamorphic4>(cameraPinhole))
        {
            if (!estimateDistortion3DEA4(cameraPinhole, statistics, allLineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else if(std::dynamic_pointer_cast<camera::Pinhole3DEClassicLD>(cameraPinhole))
        {
            if (!estimateDistortion3DELD(cameraPinhole, statistics, allLineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else 
        {
            ALICEVISION_LOG_ERROR("Incompatible camera distortion model");
        }

        ALICEVISION_LOG_INFO("Result quality of calibration: ");
        ALICEVISION_LOG_INFO("Mean of error (stddev): " << statistics.mean << "(" << statistics.stddev <<")");
        ALICEVISION_LOG_INFO("Median of error: " << statistics.median);

        std::vector<calibration::PointPair> points;
        if (!generatePoints(points, cameraPinhole, allLineWithPoints))
        {
            ALICEVISION_LOG_ERROR("Error generating points");
            continue;
        }
 
        // Estimate distortion
        if(std::dynamic_pointer_cast<camera::PinholeRadialK1>(cameraPinhole))
        {
            if (!estimateDistortionK1(cameraPinhole, statistics, points))
            {
                ALICEVISION_LOG_ERROR("Error estimating reverse distortion");
                continue;
            }
        }
        else if(std::dynamic_pointer_cast<camera::PinholeRadialK3>(cameraPinhole))
        {
            if (!estimateDistortionK3(cameraPinhole, statistics, points))
            {
                ALICEVISION_LOG_ERROR("Error estimating reverse distortion");
                continue;
            }
        }
        else if(std::dynamic_pointer_cast<camera::Pinhole3DERadial4>(cameraPinhole))
        {
            if (!estimateDistortion3DER4(cameraPinhole, statistics, points))
            {
                ALICEVISION_LOG_ERROR("Error estimating reverse distortion");
                continue;
            }
        }
        else if(std::dynamic_pointer_cast<camera::Pinhole3DEAnamorphic4>(cameraPinhole))
        {
            if (!estimateDistortion3DEA4(cameraPinhole, statistics, points))
            {
                ALICEVISION_LOG_ERROR("Error estimating reverse distortion");
                continue;
            }
        }
        else if(std::dynamic_pointer_cast<camera::Pinhole3DEClassicLD>(cameraPinhole))
        {
            if (!estimateDistortion3DELD(cameraPinhole, statistics, points))
            {
                ALICEVISION_LOG_ERROR("Error estimating reverse distortion");
                continue;
            }
        }
        else 
        {
            ALICEVISION_LOG_ERROR("Incompatible camera distortion model");
        }

        ALICEVISION_LOG_INFO("Result quality of inversion: ");
        ALICEVISION_LOG_INFO("Mean of error (stddev): " << statistics.mean << "(" << statistics.stddev << ")");
        ALICEVISION_LOG_INFO("Median of error: " << statistics.median);
        
        // Export debug images using the estimated distortion
        for(std::size_t i = 0; i < lensGridFilepaths.size(); ++i)
        {
            const std::string lensGridFilepath = lensGridFilepaths[i];

            image::Image<image::RGBColor> input;
            image::readImage(lensGridFilepath, input, image::EImageColorSpace::SRGB);

            const std::string undistortedImagePath =
                (fs::path(outputPath) / fs::path(lensGridFilepath).stem()).string() + "_undistorted.exr";
            const std::string stMapImagePath =
                (fs::path(outputPath) / fs::path(lensGridFilepath).stem()).string() + "_stmap.exr";

            Vec2 offset;
            image::Image<image::RGBColor> ud = undistort(offset, cameraPinhole, input);
            image::writeImage(undistortedImagePath, ud, image::EImageColorSpace::AUTO);

            image::Image<image::RGBfColor> stmap = undistortSTMAP(offset, cameraPinhole, input);
            image::writeImage(stMapImagePath, stmap, image::EImageColorSpace::NO_CONVERSION);
        }
    }

    if(!sfmDataIO::Save(sfmData, sfmOutputDataFilepath, sfmDataIO::ESfMData(sfmDataIO::ALL)))
    {
        ALICEVISION_LOG_ERROR("The output SfMData file '" << sfmOutputDataFilepath << "' cannot be read.");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
