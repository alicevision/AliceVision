// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>

#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>
#include <aliceVision/config.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/mcc.hpp>

#include <string>
#include <fstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilename;
    std::string outputFolder;

    // user optional parameters
    bool debug = false;

    po::options_description allParams(
        "This program is used to perform color checker detection\n"
        "AliceVision colorCheckerDetection");

    po::options_description inputParams("Required parameters");
    inputParams.add_options()
        ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
         "SfMData file.")
        ("output,o", po::value<std::string>(&outputFolder)->required(),
         "Output path for the color checker detection files (*.ccdata).");

    po::options_description optionalParams("Optional parameters");
    optionalParams.add_options()
        ("debug", po::value<bool>(&debug),
         "Output debug data.");

    po::options_description logParams("Log parameters");
    logParams.add_options()
        ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
         "verbosity level (fatal, error, warning, info, debug, trace).");

    allParams.add(inputParams).add(optionalParams).add(logParams);

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

    // set verbose level
    system::Logger::get()->setLogLevel(verboseLevel);

    // create output folder
    if(!fs::exists(outputFolder))
    {
        if(!fs::create_directory(outputFolder))
        {
            ALICEVISION_LOG_ERROR("Cannot create output folder");
            return EXIT_FAILURE;
        }
    }

    // load input scene
    sfmData::SfMData sfmData;
    std::cout << sfmData.getViews().size()  << std::endl;
    if(!sfmDataIO::Load(sfmData, sfmDataFilename, sfmDataIO::ESfMData(sfmDataIO::VIEWS|sfmDataIO::INTRINSICS)))
    {
        ALICEVISION_LOG_ERROR("The input file '" + sfmDataFilename + "' cannot be read");
        return EXIT_FAILURE;
    }

    // ----------------------------------------------------------
    // ----------------------------------------------------------
    // ----------------------------------------------------------

    std::ofstream f;
    f.open(outputFolder + "/outputDebug.txt");

    f << "Start" << std::endl;
    assert(1 > 5 && "erreur");
    // ----------------------------------------------------------
    // Scroll down a bit (~40 lines) to find actual relevant code
    // ----------------------------------------------------------

    int nc = 1;
    std::string path("C:/Users/ludch/Ludwig/samples/color checker/dataset1/rMPC_0086.jpg");
    f << "Loading image" << std::endl;
    cv::Mat image = cv::imread(path, 1);
    if(image.cols == 0 || image.rows == 0)
    {
        f << "Loading image -- error: image is empty " << std::endl;
        exit(-1);
    }
    f << "Loading image -- success: size: " << image.size() << std::endl;
    if(atoi(argv[2]) != 0)
    {
        cv::resize(image, image, cv::Size(1000, 1000 * image.rows / image.cols));
        f << "Image resized to: " << image.size() << std::endl;
    }

    //--------------------------------------------------------------------------
    //-------------------------Actual Relevant Code-----------------------------
    //--------------------------------------------------------------------------

    cv::Ptr<cv::mcc::CCheckerDetector> detector = cv::mcc::CCheckerDetector::create();

    f << "Processing image" << std::endl;
    if(!detector->process(image, cv::mcc::TYPECHART(0), nc))
    {
        f << "Processing image -- not detected" << std::endl;
    }
    else
    {
        f << "Processing image -- success" << std::endl;
        // get checker
        std::vector<cv::Ptr<cv::mcc::CChecker>> checkers = detector->getListColorChecker();

        for(cv::Ptr<cv::mcc::CChecker> checker : checkers)
        {
            // current checker
            cv::Ptr<cv::mcc::CCheckerDraw> cdraw = cv::mcc::CCheckerDraw::create(checker);
            cdraw->draw(image);
            if(atoi(argv[3]) == 1)
            {
                f << "Writing output in: " << outputFolder + "/output.jpg"  << std::endl;
                cv::imwrite(outputFolder + "/output.jpg", image);
            }
        }
    }

    f << "End" << std::endl;

    f.close();

    // ----------------------------------------------------------
    // ----------------------------------------------------------
    // ----------------------------------------------------------

    return EXIT_SUCCESS;
}
