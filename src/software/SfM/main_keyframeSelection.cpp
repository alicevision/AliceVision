#include <openMVG/keyframe/KeyframeSelector.hpp>
#include <openMVG/logger.hpp>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp> 

#include <string>
#include <vector>

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

using namespace openMVG::keyframe;

int main(int argc, char** argv)
{
  // command-line parameters
  
  std::vector<std::string> mediaPaths;   // media file path list
  std::vector<std::string> brands;       // media brand list
  std::vector<std::string> models;       // media model list
  std::vector<float> mmFocals;           // media focal (mm) list
  std::vector<float> pxFocals;           // media focal (px) list
  std::string sensorDbPath;              // camera sensor width database
  std::string voctreeFilePath;           // SIFT voctree file path
  std::string outputDirectory;           // output folder for keyframes

  // algorithm variables

  std::string sharpnessPreset = ESharpnessSelectionPreset_enumToString(ESharpnessSelectionPreset::NORMAL);
  unsigned int sharpSubset = 4;
  unsigned int minFrameStep = 12;
  unsigned int maxFrameStep = 36;
  unsigned int maxNbOutFrame = 0;

  po::options_description allParams("This program is used to extract keyframes from single camera or a camera rig");

  po::options_description inputParams("Required parameters");  
  inputParams.add_options()
      ("mediaPaths", po::value< std::vector<std::string> >(&mediaPaths)->required()->multitoken(),
        "Input video files or image sequence directories.")
      ("sensorDbPath", po::value<std::string>(&sensorDbPath)->required(),
        "Camera sensor width database path.")
      ("voctreePath", po::value<std::string>(&voctreeFilePath)->required(),
        "Vocabulary tree path.")
      ("outputDirectory", po::value<std::string>(&outputDirectory)->required(),
        "Output keyframes directory for .jpg");

  po::options_description metadataParams("Metadata parameters");  
  metadataParams.add_options()
      ("brands", po::value< std::vector<std::string> >(&brands)->default_value(brands)->multitoken(),
        "Camera brands.")
      ("models", po::value< std::vector<std::string> >(&models)->default_value(models)->multitoken(),
        "Camera models.")
      ("mmFocals", po::value< std::vector<float> >(&mmFocals)->default_value(mmFocals)->multitoken(),
        "Focals in mm (will be use if not 0).")
      ("pxFocals", po::value< std::vector<float> >(&mmFocals)->default_value(mmFocals)->multitoken(),
        "Focals in px (will be use and convert in mm if not 0).");
  
  po::options_description algorithmParams("Algorithm parameters");
  algorithmParams.add_options()
      ("sharpnessPreset", po::value<std::string>(&sharpnessPreset)->default_value(sharpnessPreset),
        "Preset for sharpnessSelection : "
        "{ultra, high, normal, low, very_low, none}")
      ("sharpSubset", po::value<unsigned int>(&sharpSubset)->default_value(sharpSubset), 
        "sharp part of the image (1 = all, 2 = size/2, ...) ")
      ("minFrameStep", po::value<unsigned int>(&minFrameStep)->default_value(minFrameStep), 
        "minimum number of frames between two keyframes")
      ("maxFrameStep", po::value<unsigned int>(&maxFrameStep)->default_value(maxFrameStep), 
        "maximum number of frames for trying to select a keyframe")
      ("maxNbOutFrame", po::value<unsigned int>(&maxNbOutFrame)->default_value(maxNbOutFrame), 
        "maximum number of output frames (0 = no limit)");

  allParams.add(inputParams).add(metadataParams).add(algorithmParams);

  po::variables_map vm;
  try
  {
    po::store(po::parse_command_line(argc, argv, allParams), vm);

    if(vm.count("help") || (argc == 1))
    {
      OPENMVG_COUT(allParams);
      return EXIT_SUCCESS;
    }
    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    OPENMVG_CERR("ERROR: " << e.what() << std::endl);
    OPENMVG_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    OPENMVG_CERR("ERROR: " << e.what() << std::endl);
    OPENMVG_COUT("Usage:\n\n" << allParams);
    return EXIT_FAILURE;
  }
  
  // check output directory and update to its absolute path
  {
    const bfs::path outDir = bfs::absolute(outputDirectory);
    outputDirectory = outDir.string();
    if(!bfs::is_directory(outDir))
    {
      OPENMVG_CERR("ERROR: can't find directory " << outputDirectory << std::endl);
      return EXIT_FAILURE;
    }
  }

  const std::size_t nbCameras = mediaPaths.size();

  if(nbCameras < 1)
  {
    OPENMVG_CERR("ERROR: program need at least one media path" << std::endl);
    return EXIT_FAILURE;
  }

  brands.resize(nbCameras);
  models.resize(nbCameras);
  mmFocals.resize(nbCameras);
  pxFocals.resize(nbCameras);

  // debugging prints, print out all the parameters
  {
    OPENMVG_COUT("Program called with the following parameters:");

    if(nbCameras == 1)
      OPENMVG_COUT("\tSingle camera");
    else
      OPENMVG_COUT("\tCamera Rig");

    for(std::size_t i = 0; i < nbCameras; ++i)
    {
      OPENMVG_COUT("\tcamera : "        << mediaPaths.at(i));
      OPENMVG_COUT("\t - brand : "      << brands.at(i));
      OPENMVG_COUT("\t - model : "      << models.at(i));
      OPENMVG_COUT("\t - focal (mm) : " << mmFocals.at(i));
      OPENMVG_COUT("\t - focal (px) : " << pxFocals.at(i));
    }

    OPENMVG_COUT("\tsensor database file path : "  << sensorDbPath);
    OPENMVG_COUT("\tvocabulary tree file path : "  << voctreeFilePath);
    OPENMVG_COUT("\toutput directory : "           << outputDirectory);
    OPENMVG_COUT("\tsharpness selection preset : " << sharpnessPreset);
    OPENMVG_COUT("\tsharp subset : "               << sharpSubset);
    OPENMVG_COUT("\tmin frame step : "             << minFrameStep);
    OPENMVG_COUT("\tmax frame step : "             << maxFrameStep);
    OPENMVG_COUT("\tmax nb out frame : "           << maxNbOutFrame);
  }

  // initialize KeyframeSelector
  KeyframeSelector selector(mediaPaths, sensorDbPath, voctreeFilePath, outputDirectory);
  
  // initialize media metadatas vector
  std::vector<KeyframeSelector::CameraInfo> cameraInfos(nbCameras);

  for(std::size_t i = 0; i < nbCameras; ++i)
  {
    KeyframeSelector::CameraInfo& metadata = cameraInfos.at(i);

    const std::string& brand = brands.at(i);
    const std::string& model = brands.at(i);
    const float mmFocal = mmFocals.at(i);
    const float pxFocal = pxFocals.at(i);

    if(!brand.empty())
      metadata.brand = brand;
    if(!model.empty())
      metadata.model = model;

    if((pxFocal == .0f) && (mmFocal == .0f))
      continue;

    metadata.focalIsMM = (pxFocal == .0f);
    metadata.focalLength = metadata.focalIsMM ? mmFocal : std::fabs(pxFocal);
  }

  selector.setCameraInfos(cameraInfos);

  // set algorithm parameters
  selector.setSharpnessSelectionPreset(ESharpnessSelectionPreset_stringToEnum(sharpnessPreset));
  selector.setSharpSubset(sharpSubset);
  selector.setMinFrameStep(minFrameStep);
  selector.setMaxFrameStep(maxFrameStep);
  selector.setMaxOutFrame(maxNbOutFrame);
  
  // process
  selector.process();        
          
  return EXIT_SUCCESS;
}
