#include <openMVG/keyframe/KeyframeSelector.hpp>
#include <openMVG/logger.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp> 
#include <string>

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

using namespace openMVG::keyframe;

int main(int argc, char** argv)
{
  // command-line parameters
  
  std::string mediaFilePath;    // media file path
  std::string sensorDbPath;     // camera sensor width database
  std::string voctreeFilePath;  // SIFT voctree file path
  std::string outputDirectory;  // output folder for keyframes
  
  // media metadata
  
  std::string brand = "Pinhole";
  std::string model = "Pinhole";
  float mmFocal = 1.0f;
  float pxFocal = 0.0f;
  
  // algorithm variables
  
  unsigned int sharpSubset = 4; 
  unsigned int minFrameStep = 12;
  unsigned int maxFrameStep = 36;
  unsigned int maxNbOutFrame = 0;

  po::options_description allParams("This program is used to extract keyframes from a video file or an image folder");

  po::options_description inputParams("Required parameters");  
  inputParams.add_options()
      ("mediaPath", po::value<std::string>(&mediaFilePath)->required(),
          "Input video file or image sequence directory.")
      ("sensorDbPath", po::value<std::string>(&sensorDbPath)->required(),
              "Camera sensor width database path.")
      ("voctreePath", po::value<std::string>(&voctreeFilePath)->required(),
              "Vocabulary tree path.")
      ("outputDirectory", po::value<std::string>(&outputDirectory)->required(),
          "Output keyframes directory for .jpg");

  po::options_description metadataParams("Metadata parameters");  
  metadataParams.add_options()
      ("brand", po::value<std::string>(&brand)->default_value(brand),
          "Camera brand.")
      ("model", po::value<std::string>(&model)->default_value(model),
          "Camera model.")
      ("mmFocal", po::value<float>(&mmFocal)->default_value(mmFocal),
          "Focal in mm.")
      ("pxFocal", po::value<float>(&pxFocal)->default_value(pxFocal),
          "Focal in px (will be use and convert in mm if not 0).");
  
  po::options_description algorithmParams("Algorithm parameters");
  algorithmParams.add_options()
      ("sharpSubset", po::value<unsigned int>(&sharpSubset)->default_value(sharpSubset), 
            "sharp part of the image (1 = all, 2 = size/2, ...) ")
      ("minFrameStep", po::value<unsigned int>(&minFrameStep)->default_value(minFrameStep), 
          "minimum number of frame between two keyframes")
      ("maxFrameStep", po::value<unsigned int>(&maxFrameStep)->default_value(maxFrameStep), 
          "maximum number of frame for evaluation")
      ("maxNbOutFrame", po::value<unsigned int>(&maxNbOutFrame)->default_value(maxNbOutFrame), 
        "maximum number of output frame (0 = no limit)");

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
  
  // check output directory and update to it's absolute path
  {
    bfs::path outDir = bfs::absolute(outputDirectory);
    outputDirectory = outDir.string();
    if(!bfs::is_directory(outDir))
    {
      OPENMVG_CERR("ERROR: can't find directory " << outputDirectory << std::endl);
      return EXIT_FAILURE;
    }
  }

  // debugging prints, print out all the parameters
  {
    OPENMVG_COUT("Program called with the following parameters:");
    OPENMVG_COUT("\tvideo file path : " << mediaFilePath);
    OPENMVG_COUT("\tsensor database file path : " << sensorDbPath);
    OPENMVG_COUT("\tvocabulary tree file path : " << voctreeFilePath);
    OPENMVG_COUT("\toutput directory : " << outputDirectory);
    OPENMVG_COUT("\tcamera brand : " << brand);
    OPENMVG_COUT("\tcamera model : " << model);
    OPENMVG_COUT("\tfocal in mm  : " << mmFocal);
    OPENMVG_COUT("\tfocal in px  : " << pxFocal);
    OPENMVG_COUT("\tsharp subset : " << sharpSubset);
    OPENMVG_COUT("\tmin frame step : " << minFrameStep);
    OPENMVG_COUT("\tmax frame step : " << maxFrameStep);
    OPENMVG_COUT("\tmax nb out frame : " << maxNbOutFrame);
  }
  
  // init KeyframeSelector
  
  KeyframeSelector selector(mediaFilePath, sensorDbPath, voctreeFilePath, outputDirectory);
  
  // set metadata parameters
  
  {
    bool focalisMM = (pxFocal == 0.0f);
    float focalLength = focalisMM ? mmFocal : std::fabs(pxFocal);
    selector.setCameraInfo(brand, model, focalLength, focalisMM);
  }
  
  // set algorithm parameters
  
  selector.setSharpSubset(sharpSubset);
  selector.setMinFrameStep(minFrameStep);
  selector.setMaxFrameStep(maxFrameStep);
  selector.setMaxOutFrame(maxNbOutFrame);
  
  // process
  
  selector.process();        
          
  return EXIT_SUCCESS;
}
