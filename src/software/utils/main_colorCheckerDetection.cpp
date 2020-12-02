#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/main.hpp>

#include <dependencies/vectorGraphics/svgDrawer.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <OpenImageIO/imageio.h>
#include <OpenImageIO/imagebuf.h>
#include <OpenImageIO/imagebufalgo.h>

#include <opencv2/mcc.hpp>

#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 1
#define ALICEVISION_SOFTWARE_VERSION_MINOR 0

using namespace aliceVision;

namespace fs = boost::filesystem;
namespace po = boost::program_options;
namespace oiio = OIIO;


int aliceVision_main(int argc, char** argv)
{
    // command-line parameters
    std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string inputPath;                      // media file path list
    std::string outputFolder;                   // output folder for splited images
    std::string splitMode;                      // split mode (exif, dualfisheye, equirectangular)
    std::string dualFisheyeSplitPreset;         // dual-fisheye split type preset
    std::size_t equirectangularNbSplits;        // nb splits for equirectangular image
    std::size_t equirectangularSplitResolution; // split resolution for equirectangular image
    bool equirectangularDemoMode;

    std::cout << OPENCV_VERSION << std::endl;

    po::options_description allParams(
        "This program is used to perform color checker detection\n"
        "AliceVision colorCheckerDetection");

    po::options_description logParams("Log parameters");
    logParams.add_options()("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
                            "verbosity level (fatal,  error, warning, info, debug, trace).");

    return EXIT_SUCCESS;
}
