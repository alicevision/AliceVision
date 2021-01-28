/*Command line parameters*/
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

#include <opencv2/opencv.hpp>

#include <aliceVision/calibration/distortionEstimation.hpp>

#include "libcbdetect/boards_from_corners.h"
#include "libcbdetect/config.h"
#include "libcbdetect/find_corners.h"
#include "libcbdetect/plot_boards.h"
#include "libcbdetect/plot_corners.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace aliceVision;



cv::Mat undistort(const std::shared_ptr<camera::Pinhole> & camera, const image::Image<image::RGBColor> & source) 
{
    double w = source.Width();
    double h = source.Height();

    double minx = w;
    double maxx = 0;
    double miny = h;
    double maxy = 0;

    for (int i = 0; i < h; i++) 
    {
        for (int j = 0; j < w; j++)
        {
            Vec2 pos = camera->get_d_pixel(Vec2(j, i));
            minx = std::min(minx, pos.x());
            maxx = std::max(maxx, pos.x());
            miny = std::min(miny, pos.y());
            maxy = std::max(maxy, pos.y());
        }
    }

    int width = maxx - minx + 1;
    int height = maxy - miny + 1;

    cv::Mat result(height, width, CV_8UC3);
    result = 0;

    const image::Sampler2d<image::SamplerLinear> sampler;

    for (int i = 0; i < height; i++) 
    {
        double y = miny + double(i);

        for (int j = 0; j < width; j++)
        {
            double x = minx + double(j);

            Vec2 pos(x, y);
            Vec2 dist = camera->get_ud_pixel(pos);


            if (dist.x() < 0 || dist.x() >= source.Width()) continue;
            if (dist.y() < 0 || dist.y() >= source.Height()) continue;
            
            image::RGBColor c = sampler(source, dist.y(), dist.x());
            
            result.at<cv::Vec3b>(i, j) = cv::Vec3b(c.r(), c.g(), c.b());
        }
    }

    return result;
}

bool retrieveLines(std::vector<calibration::LineWithPoints> & lineWithPoints, const image::Image<image::RGBColor> & input, const std::string & checkerImagePath)
{
    cv::Mat inputOpencvWrapper(input.Height(), input.Width(), CV_8UC3, (void*)input.data());

    cbdetect::Params params;
    params.show_processing = false;
    params.show_debug_image = false;
    params.corner_type = cbdetect::SaddlePoint;
    params.norm = true;

    cbdetect::Corner corners;
    cbdetect::find_corners(inputOpencvWrapper, corners, params);
    if (corners.p.size() < 20) 
    {
        return false;
    }

    std::vector<cbdetect::Board> boards;
    cbdetect::boards_from_corners(inputOpencvWrapper, corners, boards, params);
    
    

    cv::Mat draw = inputOpencvWrapper.clone();
    for (cbdetect::Board & b : boards)
    {
        int height = b.idx.size();
        int width = b.idx[0].size();

        // Create horizontal lines
        for (int i = 0; i < height - 1; i ++)
        {
            for (int j = 0; j < width - 1; j++)
            {
                int idx = b.idx[i][j];
                if (idx < 0) continue;

                cv::Point2d p = corners.p[idx];

                cv::circle(draw, p, 5, cv::Scalar(255,0,0));

                idx = b.idx[i][j + 1];
                if (idx >=0) 
                {
                    cv::Point2d p2 = corners.p[idx];
                    cv::line(draw, p, p2, cv::Scalar(0,0,255));    
                }

                idx = b.idx[i + 1][j];
                if (idx >=0) 
                {
                    cv::Point2d p2 = corners.p[idx];
                    cv::line(draw, p, p2, cv::Scalar(0,0,255));    
                }
            }
        }
    }

    cv::imwrite(checkerImagePath, draw);
    
    
    lineWithPoints.clear();
    for (cbdetect::Board & b : boards)
    {
        int height = b.idx.size();
        int width = b.idx[0].size();

        // Create horizontal lines
        for (int i = 0; i < height; i ++)
        {
            //Random init
            calibration::LineWithPoints line;
            line.angle = M_PI_4;
            line.dist = 1;

            for (int j = 0; j < width; j++)
            {
                int idx = b.idx[i][j];
                if (idx < 0) continue;

                cv::Point2d p = corners.p[idx];

                Vec2 pt;
                pt.x() = p.x;
                pt.y() = p.y;

                line.points.push_back(pt);
            }

            //Check we don't have a too small line which won't be easy to estimate
            if (line.points.size() < 10) continue;
            lineWithPoints.push_back(line);
        }

        // Create horizontal lines
        for (int j = 0; j < width; j++)
        {
            calibration::LineWithPoints line;
            line.angle = M_PI_4;
            line.dist = 1;

            for (int i = 0; i < height; i++)
            {
                int idx = b.idx[i][j];
                if (idx < 0) continue;

                cv::Point2d p = corners.p[idx];

                Vec2 pt;
                pt.x() = p.x;
                pt.y() = p.y;

                line.points.push_back(pt);
            }

            //Check we don't have a too small line which won't be easy to estimate
            if (line.points.size() < 10) continue;

            lineWithPoints.push_back(line);
        }
    }

    return true;
}

bool estimateDistortionK1(std::shared_ptr<camera::Pinhole> & camera, std::vector<calibration::LineWithPoints> & lineWithPoints)
{
    std::vector<bool> locksDistortions = {true};

    //Everything locked except lines paramters
    locksDistortions[0] = true;
    if (!calibration::estimate(camera, lineWithPoints, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    //Relax distortion 1st order
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, lineWithPoints, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, lineWithPoints, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    return true;
}

bool estimateDistortionK3(std::shared_ptr<camera::Pinhole> & camera, std::vector<calibration::LineWithPoints> & lineWithPoints)
{
    std::vector<bool> locksDistortions = {true, true, true};

    //Everything locked except lines paramters
    locksDistortions[0] = true;
    if (!calibration::estimate(camera, lineWithPoints, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    //Relax distortion 1st order
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, lineWithPoints, true, true, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    if (!calibration::estimate(camera, lineWithPoints, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    //Relax offcenter
    locksDistortions[0] = false;
    locksDistortions[1] = false;
    locksDistortions[2] = false;
    if (!calibration::estimate(camera, lineWithPoints, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    return true;
}

bool estimateDistortionA4(std::shared_ptr<camera::Pinhole> & camera, std::vector<calibration::LineWithPoints> & lineWithPoints)
{
    
    std::shared_ptr<camera::Pinhole> simpleCamera = std::make_shared<camera::PinholeRadialK1>(camera->w(), camera->h(), camera->getScale()[0], camera->getScale()[1], camera->getOffset()[0], camera->getOffset()[1], 0.0);
    if (!estimateDistortionK1(simpleCamera, lineWithPoints))
    {
        return false;
    }


    std::vector<bool> locksDistortions = {true, true, true, true};

    //Relax distortion all orders
    locksDistortions[0] = false;
    locksDistortions[1] = false;
    locksDistortions[2] = false;
    locksDistortions[3] = false;

    double k1 = simpleCamera->getDistortionParams()[0];
    camera->setDistortionParams({k1,k1,k1,k1});

    if (!calibration::estimate(camera, lineWithPoints, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    return true;
}

bool estimateDistortionA10(std::shared_ptr<camera::Pinhole> & camera, std::vector<calibration::LineWithPoints> & lineWithPoints)
{
    
    std::shared_ptr<camera::Pinhole> simplerCamera = std::make_shared<camera::PinholeAnamorphic4>(camera->w(), camera->h(), camera->getScale()[0], camera->getScale()[1], camera->getOffset()[0], camera->getOffset()[1]);
    if (!estimateDistortionA4(simplerCamera, lineWithPoints))
    {
        return false;
    }


    std::vector<bool> locksDistortions = {true, true, true, true, true, true, true, true, true, true};

    //Relax distortion all orders
    locksDistortions[0] = false;
    locksDistortions[1] = false;
    locksDistortions[2] = false;
    locksDistortions[3] = false;
    locksDistortions[4] = false;
    locksDistortions[5] = false;
    locksDistortions[6] = false;
    locksDistortions[7] = false;
    locksDistortions[8] = false;
    locksDistortions[9] = false;

    double cxx = simplerCamera->getDistortionParams()[0];
    double cxy = simplerCamera->getDistortionParams()[1];
    double cyx = simplerCamera->getDistortionParams()[2];
    double cyy = simplerCamera->getDistortionParams()[3];

    camera->setDistortionParams({cxx, cxy, 0.0, 0.0, 0.0, cyx, cyy, 0.0, 0.0, 0.0});

    if (!calibration::estimate(camera, lineWithPoints, true, false, locksDistortions))
    {
        ALICEVISION_LOG_ERROR("Failed to calibrate");
        return EXIT_FAILURE;
    }

    return true;
}

int aliceVision_main(int argc, char* argv[])
{
    std::string sfmInputDataFilepath;
    std::string sfmOutputDataFilepath;
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());

    // Command line parameters
    po::options_description allParams(
    "Parse external information about cameras used in a panorama.\n"
    "AliceVision PanoramaInit");

    po::options_description requiredParams("Required parameters");
    requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmInputDataFilepath)->required(), "SfMData file input.")
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

    // Analyze path
    boost::filesystem::path path(sfmOutputDataFilepath);
    std::string outputPath = path.parent_path().string();

    int pos = 0;
    for (auto v : sfmData.getViews()) 
    {
        auto view = v.second;
        const std::string viewIdStr = std::to_string(v.first);

        ALICEVISION_LOG_INFO("Processing view " << v.first);

        //Check pixel ratio
        float pixelRatio = view->getDoubleMetadata({"PixelAspectRatio"});
        if (pixelRatio < 0.0) 
        {
            pixelRatio = 1.0f;
        }

        //Read image
        image::Image<image::RGBColor> input;
        std::string pathImage = view->getImagePath();
        image::readImage(v.second->getImagePath(), input, image::EImageColorSpace::SRGB);       

        if (pixelRatio != 1.0f)
        {
            double w = input.Width();
            double h = input.Height();
            double nw = w;
            double nh = h / pixelRatio;
            image::Image<image::RGBColor> resizedInput(nw, nh);

            const oiio::ImageSpec imageSpecResized(nw, nh, 3, oiio::TypeDesc::UCHAR);
            const oiio::ImageSpec imageSpecOrigin(w, h, 3, oiio::TypeDesc::UCHAR);

            const oiio::ImageBuf inBuf(imageSpecOrigin, input.data());
            oiio::ImageBuf outBuf(imageSpecResized, resizedInput.data());

            oiio::ImageBufAlgo::resize(outBuf, inBuf);
            input.swap(resizedInput);
        }

        
        std::shared_ptr<camera::IntrinsicBase> cameraBase = sfmData.getIntrinsicsharedPtr(v.second->getIntrinsicId());
        std::shared_ptr<camera::Pinhole> cameraPinhole = std::dynamic_pointer_cast<camera::Pinhole>(cameraBase);
        if (!cameraPinhole)
        {
            ALICEVISION_LOG_ERROR("Only work for pinhole cameras");
            continue;
        }

        
        double w = input.Width();
        double h = input.Height();
        double d = sqrt(w*w + h*h);

        //Force the 'focal' to normalize the image such that its semi diagonal is 1
        cameraPinhole->setWidth(w);
        cameraPinhole->setHeight(h);
        cameraPinhole->setScale(d, d);
        cameraPinhole->setOffset(w / 2, h / 2);
        
        std::string undistortedImagePath = (fs::path(outputPath) / (viewIdStr + "_undistorted.png")).string();
        std::string checkerImagePath = (fs::path(outputPath) / (viewIdStr + "_checker.png")).string();

        //Retrieve lines
        std::vector<calibration::LineWithPoints> lineWithPoints;
        if (!retrieveLines(lineWithPoints, input, checkerImagePath))
        {
            ALICEVISION_LOG_ERROR("Impossible to extract the checkerboards lines");
            continue;
        }

        //Estimate distortion
        if (std::dynamic_pointer_cast<camera::PinholeRadialK1>(cameraBase))
        {
            if (!estimateDistortionK1(cameraPinhole, lineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else if (std::dynamic_pointer_cast<camera::PinholeRadialK3>(cameraBase))
        {
            if (!estimateDistortionK3(cameraPinhole, lineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else if (std::dynamic_pointer_cast<camera::PinholeAnamorphic4>(cameraBase))
        {
            if (!estimateDistortionA4(cameraPinhole, lineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else if (std::dynamic_pointer_cast<camera::PinholeAnamorphic10>(cameraBase))
        {
            if (!estimateDistortionA10(cameraPinhole, lineWithPoints))
            {
                ALICEVISION_LOG_ERROR("Error estimating distortion");
                continue;
            }
        }
        else 
        {
            ALICEVISION_LOG_ERROR("Incompatible camera distortion model");
        }

        cv::Mat ud = undistort(cameraPinhole, input);
        cv::imwrite(undistortedImagePath, ud);
    }

    return EXIT_SUCCESS;
}
