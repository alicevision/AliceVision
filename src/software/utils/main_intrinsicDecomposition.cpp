#include <aliceVision/intrinsicDecomposition/IntrinsicDecompositionSolver.hpp>
#include <aliceVision/intrinsicDecomposition/SolverParams.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/imageIO/image.hpp>
#include <aliceVision/sfmDataIO/sfmDataIO.hpp>
#include <aliceVision/system/cmdline.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>
#include <boost/filesystem/path.hpp>

#include <string>
#include <sstream>
#include <vector>

// These constants define the current software version.
// They must be updated when the command line is changed.
#define ALICEVISION_SOFTWARE_VERSION_MAJOR 0
#define ALICEVISION_SOFTWARE_VERSION_MINOR 1

using namespace aliceVision;
// using namespace aliceVision::sfm;
using namespace aliceVision::sfmData;
using namespace aliceVision::sfmDataIO;
using namespace aliceVision::intrinsicDecomposition;

namespace po = boost::program_options;
namespace fs = boost::filesystem;


int main(int argc, const char * argv[])
{
	std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
    std::string sfmDataFilepath;
    std::string inputUndistortFolder;
    std::string inputNormalMapsFolder;
    std::string outputFolder;

    int rangeStart = -1;
    int rangeSize = 1;

	po::options_description params("AliceVision Uncertainty");
	params.add_options()
		("input,i", po::value<std::string>(&sfmDataFilepath)->required(),
         "SfMData file to align.")
        ("inputUndistortFolder,o", po::value<std::string>(&inputUndistortFolder)->required(),
         "Output folder.")
        ("inputNormalMapsFolder,o", po::value<std::string>(&inputNormalMapsFolder), //->required(),
         "Output folder.")
		("output,o", po::value<std::string>(&outputFolder)->required(),
         "Output folder.")

        ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
         "Range image index start.")
        ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
         "Range size.")

		("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
         "verbosity level (fatal,  error, warning, info, debug, trace).");

	po::variables_map vm;
	try
	{
		po::store(po::parse_command_line(argc, argv, params), vm);

		if(vm.count("help") || (argc == 1))
		{
			ALICEVISION_COUT(params);
			return EXIT_SUCCESS;
		}
		po::notify(vm);
	}
	catch(boost::program_options::required_option& e)
	{
		ALICEVISION_CERR("ERROR: " << e.what());
		ALICEVISION_COUT("Usage:\n\n" << params);
		return EXIT_FAILURE;
	}
	catch(boost::program_options::error& e)
	{
		ALICEVISION_CERR("ERROR: " << e.what());
		ALICEVISION_COUT("Usage:\n\n" << params);
		return EXIT_FAILURE;
	}
	ALICEVISION_COUT(vm);

	// set verbose level
	system::Logger::get()->setLogLevel(verboseLevel);

	if (sfmDataFilepath.empty() ||
	outputFolder.empty())
	{
		std::cerr << "Invalid input or output filename." << std::endl;
		return EXIT_FAILURE;
	}

	// Load input scene
	SfMData sfmData;
	if (!Load(sfmData, sfmDataFilepath, ESfMData(ALL)))
	{
		std::cerr << std::endl
		<< "The input SfMData file \"" << sfmDataFilepath << "\" cannot be read." << std::endl;
		return EXIT_FAILURE;
	}

    int rangeEnd = sfmData.getViews().size();
    // set range
    if(rangeStart != -1)
    {
      if(rangeStart < 0 || rangeSize < 0 ||
          rangeStart > sfmData.getViews().size())
      {
        ALICEVISION_LOG_ERROR("Range is incorrect");
        return EXIT_FAILURE;
      }

      if(rangeStart + rangeSize > sfmData.views.size())
          rangeSize = sfmData.views.size() - rangeStart;

      rangeEnd = rangeStart + rangeSize;
    }
    else
    {
      rangeStart = 0;
    }

    SolverParams solverParams;
    solverParams.nonLinearIter = 7;
    solverParams.linearIter = 10;
    solverParams.useOptGN = false;
    solverParams.useOptLM = true;
	// params.optDoublePrecision = true;

    mvsUtils::MultiViewParams mp(sfmData, inputUndistortFolder, "", inputNormalMapsFolder, false);

    unsigned int maxWidth = mp.getMaxImageWidth();
    unsigned int maxHeight = mp.getMaxImageHeight();

    IntrinsicDecompositionSolver solver(maxWidth, maxHeight, solverParams);

    std::set<IndexT> validViews = sfmData.getValidViews();

// #pragma omp parallel for num_threads(3)
    for(int i = rangeStart; i < rangeEnd; ++i)
    {
        auto viewIt = validViews.begin();
        std::advance(viewIt, i);

        const IndexT viewId = *viewIt;
        const std::string viewIdStr = std::to_string(viewId);

        const std::string colorPath = mp.getImagePath(mp.getIndexFromViewId(viewId));
        const std::string normalsPath = mvsUtils::getFileNameFromViewId(&mp, viewId, mvsUtils::EFileType::normalMap, 0);

		int width = 0;
		int height = 0;
		std::vector<Color> colorImage;
        ALICEVISION_LOG_INFO("Load image: " << colorPath);
        imageIO::readImage(colorPath, width, height, colorImage);
        ALICEVISION_LOG_INFO("Load normals map: " << normalsPath);
        int width_n = 0;
        int height_n = 0;
        std::vector<Color> normalsImage;
        imageIO::readImage(normalsPath, width_n, height_n, normalsImage);
        if(width != width_n || height != height_n)
            throw std::runtime_error("Image and normal maps size are not the same.");

        /*{
            fs::path cPath = fs::path(outputFolder) / (viewIdStr + "_inputColor.exr");
            imageIO::writeImage(cPath.string(), width, height, colorImage, imageIO::EImageQuality::OPTIMIZED);
            fs::path imgPath = fs::path(outputFolder) / (viewIdStr + "_inputNormals.exr");
            imageIO::writeImage(imgPath.string(), width, height, normalsImage, imageIO::EImageQuality::OPTIMIZED);
        }*/

        ALICEVISION_LOG_INFO("width x height: " << width << " x " << height);

        solver.setImage(colorImage, normalsImage, width, height);

        solver.solveAll();

        std::vector<Color> albedoImage(colorImage.size());
        std::vector<float> shadingImage(colorImage.size());

		solver.retrieveImages(shadingImage, albedoImage);

        fs::path albedoPath = fs::path(outputFolder) / (viewIdStr + "_albedo.exr");
        imageIO::writeImage(albedoPath.string(), width, height, albedoImage, imageIO::EImageQuality::OPTIMIZED);

        fs::path shadingPath = fs::path(outputFolder) / (viewIdStr + "_shading.exr");
        writeImage(shadingPath.string(), width, height, shadingImage, imageIO::EImageQuality::OPTIMIZED);
	}

	return 0;
}
