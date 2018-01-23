// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/config.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/feature/imageDescriberCommon.hpp>
#include <aliceVision/feature/feature.hpp>
#include <aliceVision/system/Timer.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/system/cmdline.hpp>

#include <dependencies/stlplus3/filesystemSimplified/file_system.hpp>

#include <boost/progress.hpp>
#include <boost/program_options.hpp>

#include <cereal/archives/json.hpp>

#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iostream>
#include <functional>
#include <limits>

using namespace aliceVision;
using namespace aliceVision::image;
using namespace aliceVision::feature;
using namespace aliceVision::sfm;
using namespace std;
namespace po = boost::program_options;

// ----------
// Dispatcher
// ----------

#ifdef __linux__
#include <unistd.h>
#include <sys/wait.h>

// Returns a map containing information about the memory usage o
// of the system, it basically reads /proc/meminfo  
std::map<std::string, unsigned long> memInfos()
{
  std::map<std::string, unsigned long> memoryInfos;
  std::ifstream meminfoFile("/proc/meminfo");
  if (meminfoFile.is_open())
  {
    std::string line;
    while(getline(meminfoFile, line))
    {
      auto separator = line.find(":"); 
      if (separator!=std::string::npos)
      {
        const std::string key = line.substr(0, separator);
        const std::string value = line.substr(separator+1);
        memoryInfos[key] = std::strtoul(value.c_str(), nullptr, 10);
      }
    }
  }
  return memoryInfos;
}


// Count the number of processors of the machine using /proc/cpuinfo
unsigned int countProcessors()
{
  unsigned int nprocessors = 0;
  std::ifstream cpuinfoFile("/proc/cpuinfo");
  if (cpuinfoFile.is_open())
  {
    std::string line;
    while(getline(cpuinfoFile, line))
    {
        // The line must start with the word "processor"
        if (line.compare(0, std::strlen("processor"), "processor") == 0)
            nprocessors++;
    }
  }
  return nprocessors; 
}

// Returns the number of jobs to run simultaneously if one job should 
// run with jobMemoryRequirement Kb 
int remainingJobSlots(unsigned long jobMemoryRequirement)
{
  assert(jobMemoryRequirement!=0);
  auto meminfos = memInfos();
  const unsigned long available = meminfos["MemFree"] + meminfos["Buffers"] + meminfos["Cached"]; 
  const unsigned int memSlots = static_cast<unsigned int>(available/jobMemoryRequirement); 
  const unsigned int cpuSlots = countProcessors(); 
  return std::max(std::min(memSlots, cpuSlots), 1u);
}

// Returns the peak virtual memory of the process processID
// returns 0 if the process is not alive anymore, ie the 
// /proc/pid/status can't be opened
unsigned long peakMemory(pid_t processID)
{  
  char processStatusFileName[256];
  snprintf(processStatusFileName, 256, "/proc/%d/status", processID);
  std::ifstream processStatusFile(processStatusFileName);
  if (processStatusFile.is_open())
  {
    std::string line;
    constexpr const char *peakString = "VmPeak:";
    constexpr size_t peakStringLen = std::strlen(peakString);
    while(getline(processStatusFile, line))
    {
      if (line.compare(0, peakStringLen, peakString) == 0)
      {
        const std::string value = line.substr(peakStringLen);
        return std::strtoul(value.c_str(), nullptr, 10);  
      }
    }
  }
  return 0ul;
}

// This function dispatch the compute function on several sub processes
// keeping the maximum number of subprocesses under maxJobs
void dispatch(const int maxJobs, std::function<void()> compute)
{
  static int nbJobs;
  static unsigned int subProcessPeakMemory = 0;
  static int possibleJobs=0;
  pid_t pid = fork();
  if (pid < 0)           
  {                      
    // Something bad happened
    std::cerr << "fork failed\n";
    _exit(EXIT_FAILURE);
  }
  else if(pid == 0)
  {
    // Disable OpenMP as we dispatch the work on multiple sub processes
    // and we don't want that each subprocess use all the cpu ressource
    omp_set_num_threads(1); 
    compute();
    _exit(EXIT_SUCCESS);
  }
  else 
  {
    nbJobs++;
    // Use subprocess peak memory and assume the next job will use the 
    // same amount of memory. It allows to roughly determine the number 
    // of possible jobs running simultaneously
    if (subProcessPeakMemory==0)
    {
      unsigned int readPeak = peakMemory(pid);
      while(readPeak != 0) 
      { // sample memory of the first job 
        sleep(0.2); // sample every 0.2 seconds;
        if( subProcessPeakMemory < readPeak )
        {
          subProcessPeakMemory = readPeak;
        }  
        readPeak = peakMemory(pid);
      } 
      possibleJobs = remainingJobSlots(subProcessPeakMemory);
    }
    
    // Wait for a subprocess to stop when no more job slots are available
    if (nbJobs >= possibleJobs || (maxJobs != 0 && nbJobs >= maxJobs))
    {
      pid_t pids;
      while(pids = waitpid(-1, NULL, 0)) 
      {
        nbJobs--;
        break;
      }
    }
  }
}

// Waits for all subprocesses to terminate 
void waitForCompletion()
{
  pid_t pids;
  while(pids = waitpid(-1, NULL, 0)) 
  {
    if (errno == ECHILD)
        break;
  }
}


#else // __linux__

void dispatch(const int maxJobs, std::function<void()> compute)
{
  if(maxJobs != 0)
    omp_set_num_threads(maxJobs);
  compute();
}
void waitForCompletion() {}
int remainingJobSlots(unsigned long jobMemoryRequirement) {return 1;}  

#endif // __linux__

/// - Compute view image description (feature & descriptor extraction)
/// - Export computed data
int main(int argc, char **argv)
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outputFolder;

  // user optional parameters

  std::string describerTypesName = EImageDescriberType_enumToString(EImageDescriberType::SIFT);
  std::string describerPreset = EImageDescriberPreset_enumToString(EImageDescriberPreset::NORMAL);
  bool describersAreUpRight = false;
  int rangeStart = -1;
  int rangeSize = 1;
  int maxJobs = 0;

  po::options_description allParams("AliceVision featureExtraction");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outputFolder)->required(),
      "Output path for the features and descriptors files (*.feat, *.desc).");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("describerTypes,d", po::value<std::string>(&describerTypesName)->default_value(describerTypesName),
      EImageDescriberType_informations().c_str())
    ("describerPreset,p", po::value<std::string>(&describerPreset)->default_value(describerPreset),
      "Control the ImageDescriber configuration (low, medium, normal, high, ultra).\n"
      "Configuration 'ultra' can take long time !")
    ("upright,u", po::value<bool>(&describersAreUpRight)->default_value(describersAreUpRight),
      "Use Upright feature.")
    ("rangeStart", po::value<int>(&rangeStart)->default_value(rangeStart),
      "Range image index start.")
    ("rangeSize", po::value<int>(&rangeSize)->default_value(rangeSize),
      "Range size.")
    ("jobs", po::value<int>(&maxJobs)->default_value(maxJobs),
      "Specifies the number of jobs to run simultaneously (0 for automatic mode).");

  po::options_description logParams("Log parameters");
  logParams.add_options()
    ("verboseLevel,v", po::value<std::string>(&verboseLevel)->default_value(verboseLevel),
      "verbosity level (fatal, error, warning, info, debug, trace).");

  allParams.add(requiredParams).add(optionalParams).add(logParams);

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

  if (outputFolder.empty())
  {
    ALICEVISION_LOG_ERROR("Error: It is an invalid output folder");
    return EXIT_FAILURE;
  }

  // Create output dir
  if (!stlplus::folder_exists(outputFolder))
  {
    if (!stlplus::folder_create(outputFolder))
    {
      ALICEVISION_LOG_ERROR("Error: Cannot create output folder");
      return EXIT_FAILURE;
    }
  }

  // a. Load input scene

  SfMData sfmData;

  if(stlplus::is_file(sfmDataFilename))
  {
    if(!Load(sfmData, sfmDataFilename, ESfMData(VIEWS|INTRINSICS)))
    {
      ALICEVISION_LOG_ERROR("Error: The input file '" + sfmDataFilename + "' cannot be read");
      return EXIT_FAILURE;
    }
  }
  else
  {
    ALICEVISION_LOG_ERROR("Error: The input file argument is required.");
    return EXIT_FAILURE;
  }

  // b. Init vector of imageDescriber 
  
  struct DescriberMethod
  {
    std::string typeName;
    EImageDescriberType type;
    std::shared_ptr<ImageDescriber> describer;
  };
  std::vector<DescriberMethod> imageDescribers;
  
  {
    if(describerTypesName.empty())
    {
      ALICEVISION_LOG_ERROR("Error: describerMethods argument is empty.");
      return EXIT_FAILURE;
    }
    std::vector<EImageDescriberType> describerMethodsVec = EImageDescriberType_stringToEnums(describerTypesName);

    for(const auto& describerMethod: describerMethodsVec)
    {
      DescriberMethod method;
      method.typeName = EImageDescriberType_enumToString(describerMethod);
      method.type = describerMethod;
      method.describer = createImageDescriber(method.type);
      method.describer->setUpRight(describersAreUpRight);
      imageDescribers.push_back(method);
      imageDescriber->setConfigurationPreset(describerPreset);
    }
  }

  using namespace aliceVision::feature;

  // Feature extraction routines
  // For each View of the SfMData container:
  // - if regions file exist continue,
  // - if no file, compute features
  {
    system::Timer timer;
    boost::progress_display my_progress_bar( sfmData.GetViews().size(), std::cout, "Extract features\n" );

    Views::const_iterator iterViews = sfmData.views.begin();
    Views::const_iterator iterViewsEnd = sfmData.views.end();
    
    if(rangeStart != -1)
    {
      if(rangeStart < 0 || rangeStart > sfmData.views.size())
      {
       ALICEVISION_LOG_ERROR("Bad specific index");
        return EXIT_FAILURE;
      }
      if(rangeSize < 0)
      {
        ALICEVISION_LOG_ERROR("Bad range size");
        return EXIT_FAILURE;
      }

      if(rangeStart + rangeSize > sfmData.views.size())
        rangeSize = sfmData.views.size() - rangeStart;

      std::advance(iterViews, rangeStart);
      iterViewsEnd = iterViews;
      std::advance(iterViewsEnd, rangeSize);
    }
    
    struct DescriberComputeMethod
    {
      std::size_t methodIndex;
      std::string featFilename;
      std::string descFilename;
    };
    
    for(; iterViews != iterViewsEnd; ++iterViews, ++my_progress_bar)
    {
      const View* view = iterViews->second.get();
      const std::string viewFilename = view->getImagePath();
      ALICEVISION_LOG_INFO("Extract features in view : " << viewFilename);
      
      std::vector<DescriberComputeMethod> computeMethods;
      
      for(std::size_t i = 0; i < imageDescribers.size(); ++i)
      {
        DescriberComputeMethod computeMethod;
        
        computeMethod.featFilename = stlplus::create_filespec(outputFolder,
              stlplus::basename_part(std::to_string(view->getViewId())), imageDescribers[i].typeName + ".feat");
        computeMethod.descFilename = stlplus::create_filespec(outputFolder,
              stlplus::basename_part(std::to_string(view->getViewId())), imageDescribers[i].typeName + ".desc");
      
        if (stlplus::file_exists(computeMethod.featFilename) &&
            stlplus::file_exists(computeMethod.descFilename))
        {
          // Skip the feature extraction as the results are already computed.
          continue;
        }
        
        computeMethod.methodIndex = i;
        
        // If features or descriptors file are missing, compute and export them
        computeMethods.push_back(computeMethod);
      }
      
      if(!computeMethods.empty())
      {
        auto computeFunction = [&]() {
            Image<float> imageGrayFloat;
            Image<unsigned char> imageGrayUChar;

            readImage(viewFilename, imageGrayFloat);

            for(auto& compute : computeMethods)
            {
              // Compute features and descriptors and export them to files
              ALICEVISION_LOG_INFO("Extracting " + imageDescribers[compute.methodIndex].typeName  + " features from view " + std::to_string(view->getViewId()) + " : '" + view->getImagePath() +"'");
              std::unique_ptr<Regions> regions;

              if(imageDescribers[compute.methodIndex].describer->useFloatImage())
              {
                // image buffer use float image, use the read buffer
                imageDescribers[compute.methodIndex].describer->Describe(imageGrayFloat, regions);
              }
              else
              {
                // image buffer can't use float image
                if(imageGrayUChar.Width() == 0) // the first time, convert the float buffer to uchar
                  imageGrayUChar = imageGrayFloat.GetMat().cast<unsigned char>() * 255;
                imageDescribers[compute.methodIndex].describer->Describe(imageGrayUChar, regions);
              }

              imageDescribers[compute.methodIndex].describer->Save(regions.get(), compute.featFilename, compute.descFilename);
            }
        };
        
        if (maxJobs == 1)
          computeFunction();
        else
          dispatch(maxJobs, computeFunction);
      }
    }

    if (maxJobs != 1) waitForCompletion();

    std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
  }
  return EXIT_SUCCESS;
}
