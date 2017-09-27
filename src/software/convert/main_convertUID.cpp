// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/image/image.hpp"
#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/system/Timer.hpp"

#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <boost/program_options.hpp>

#include <cereal/archives/json.hpp>

#include <cstdlib>
#include <fstream>
#include <string>

using namespace std;
using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::feature;
using namespace aliceVision::sfm;
namespace po = boost::program_options;

int main(int argc, char** argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string featuresDirectory;
  
  po::options_description allParams("AliceVision convertUID");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("featuresDirectory,f", po::value<std::string>(&featuresDirectory)->required(),
      "ath to a directory containing the extracted features.");
  
  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal,  error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(logParams);

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  //Check directory name
  if (!stlplus::is_folder(featuresDirectory))
  {
    std::cout << "The directory can't be found" << std::endl;
    return false;
  }
  
  //Load Smf_data file:
  SfMData sfmData;
  if (!Load(sfmData, sfmDataFilename, ESfMData(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input file \""<< sfmDataFilename << "\" cannot be read" << std::endl;
    return false;
  }
  
  Views::const_iterator iterViews = sfmData.views.begin();
  Views::const_iterator iterViewsEnd = sfmData.views.end();
  
  for(; iterViews != iterViewsEnd; ++iterViews)
  {
    const View * view = iterViews->second.get();
    
    std::string filename;
    std::string newname;
    std::string compared2(1,'/');
    std::string id = to_string(view->getViewId());
            
    if (featuresDirectory.substr(featuresDirectory.size() - 1, featuresDirectory.size()).compare(compared2) == 0)
    {
      filename = featuresDirectory + view->getImagePath().substr(1 ,view->getImagePath().find('.'));
      newname = featuresDirectory + id;
    }
    else 
    {
      filename = featuresDirectory + view->getImagePath().substr(0 ,view->getImagePath().find('.') + 1);
      newname = featuresDirectory + compared2 + id;
    }
    
    std::string oldDesc = filename + "desc";
    std::string newDesc = newname + ".desc";
    
    std::string oldFeat = filename + "feat";
    std::string newFeat = newname + ".feat";
    
    if (rename(oldDesc.c_str(), newDesc.c_str()) != 0)
      std::cout<< "Cannot rename" << oldDesc <<  std::endl;
    
    if (rename(oldFeat.c_str(), newFeat.c_str()) != 0)
      std::cout<< "Cannot rename" << oldFeat <<  std::endl;
      
  }
  
  return 0;
}


