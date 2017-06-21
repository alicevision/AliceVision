
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <openMVG/config.hpp>
#include "openMVG/image/image.hpp"
#include "openMVG/sfm/sfm.hpp"

/// Feature/Regions & Image describer interfaces
#include "openMVG/features/ImageDescriberCommon.hpp"
#include "openMVG/features/features.hpp"

#include "openMVG/exif/exif_IO_EasyExif.hpp"
#include "openMVG/stl/split.hpp"
#include "openMVG/system/timer.hpp"

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"
#include "third_party/progress/progress.hpp"

#include <cereal/archives/json.hpp>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <iostream>
#include <functional>
#include <limits>


using namespace openMVG;
using namespace openMVG::image;
using namespace openMVG::features;
using namespace openMVG::sfm;
using namespace std;



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
void dispatch(const int &maxJobs, std::function<void()> compute)
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

void dispatch(const int &maxJobs, std::function<void()> compute)
{
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
  // MAX_JOBS_DEFAULT is the default value for maxJobs which keeps 
  // the original behavior of the program:
  constexpr static int MAX_JOBS_DEFAULT = std::numeric_limits<int>::max();

  CmdLine cmd;

  std::string sfmDataFilename;
  std::string outDirectory = "";
  std::string featurePreset = "NORMAL";
  std::string describerMethods = "SIFT";
  bool describersAreUpRight = false;
  int rangeStart = -1;
  int rangeSize = 1;
  int maxJobs = MAX_JOBS_DEFAULT;

  // required
  cmd.add( make_option('i', sfmDataFilename, "input_file") );
  cmd.add( make_option('o', outDirectory, "outdir") );
  
  // Optional
  cmd.add( make_option('m', describerMethods, "describerMethods") );
  cmd.add( make_option('u', describersAreUpRight, "upright") );
  cmd.add( make_option('p', featurePreset, "describerPreset") );
  cmd.add( make_option('s', rangeStart, "range_start") );
  cmd.add( make_option('r', rangeSize, "range_size") );
  cmd.add( make_option('j', maxJobs, "jobs") );

  try 
  {
    if (argc == 1) throw std::string("Invalid command line parameter.");
    cmd.process(argc, argv);
  } 
  catch(const std::string& s) 
  {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] a SfM_Data file \n"
    << "[-o|--outdir path] output path for the features and descriptors files (*.feat, *.desc)\n"
    << "\n[Optional]\n"
    << "[-f|--force] Force to recompute data\n"
    << "[-m|--describerMethods]\n"
    << "  (methods to use to describe an image):\n"
    << "   SIFT (default),\n"
    << "   SIFT_FLOAT to use SIFT stored as float,\n"
    << "   AKAZE: AKAZE with floating point descriptors,\n"
    << "   AKAZE_MLDB:  AKAZE with binary descriptors\n"
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_CCTAG)
    << "   CCTAG3: CCTAG markers with 3 crowns\n"
    << "   CCTAG4: CCTAG markers with 4 crowns\n"
#endif
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OPENCV)
#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_OCVSIFT)
    << "   SIFT_OCV: OpenCV SIFT\n"
#endif
    << "   AKAZE_OCV: OpenCV AKAZE\n"
#endif
    << "[-u|--upright] Use Upright feature 0 or 1\n"
    << "[-p|--describerPreset]\n"
    << "  (used to control the Image_describer configuration):\n"
    << "   LOW,\n"
    << "   MEDIUM,\n"
    << "   NORMAL (default),\n"
    << "   HIGH,\n"
    << "   ULTRA: !!Can take long time!!\n"
    << "[-s]--range_start] range image index start\n"
    << "[-r]--range_size] range size\n"
    << "[-j|--jobs] Specifies the number of jobs to run simultaneously. Use -j 0 for automatic mode.\n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;

  }

  std::cout << " You called : " <<std::endl
            << argv[0] << std::endl
            << "--input_file " << sfmDataFilename << std::endl
            << "--outdir " << outDirectory << std::endl
            << "--describerMethods " << describerMethods << std::endl
            << "--upright " << describersAreUpRight << std::endl
            << "--describerPreset " << (featurePreset.empty() ? "NORMAL" : featurePreset) << std::endl
            << "--range_start " << rangeStart << std::endl
            << "--range_size " << rangeSize << std::endl;

  if (maxJobs != MAX_JOBS_DEFAULT)
  {
    std::cout << "--jobs " << maxJobs << std::endl;
    if (maxJobs < 0) 
    {
      std::cerr << "\nInvalid value for -j option, the value must be >= 0" << std::endl;
      return EXIT_FAILURE;
    } 
  }

  if (outDirectory.empty())
  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  // Create output dir
  if (!stlplus::folder_exists(outDirectory))
  {
    if (!stlplus::folder_create(outDirectory))
    {
      std::cerr << "Cannot create output directory" << std::endl;
      return EXIT_FAILURE;
    }
  }

  //---------------------------------------
  // a. Load input scene
  //---------------------------------------
  SfM_Data sfm_data;
  if(sfmDataFilename.empty())
  {
    std::cerr << "\nError: The input file argument is required." << std::endl;
    return EXIT_FAILURE;
  }
  else if(stlplus::is_file( sfmDataFilename))
  {
    if(!Load(sfm_data, sfmDataFilename, ESfM_Data(VIEWS|INTRINSICS)))
    {
      std::cerr << std::endl
        << "The input file \""<< sfmDataFilename << "\" cannot be read" << std::endl;
      return EXIT_FAILURE;
    }
  }
  else if(stlplus::is_folder(sfmDataFilename))
  {
    // Retrieve image paths
    std::vector<std::string> vec_images;
    const std::vector<std::string> supportedExtensions {"jpg", "jpeg"};
    
    vec_images = stlplus::folder_files(sfmDataFilename);
    std::sort(vec_images.begin(), vec_images.end());
    
    sfm_data.s_root_path = "";
    if(!sfmDataFilename.empty())
      sfm_data.s_root_path = sfmDataFilename; // Setup main image root_path
    Views & views = sfm_data.views;
    
    for(const auto &imageName : vec_images)
    {
      exif::Exif_IO_EasyExif exifReader(imageName);

      const std::size_t uid = exif::computeUID(exifReader, imageName);

      // Build the view corresponding to the image
      View v(imageName, (IndexT)uid);
      v.id_intrinsic = UndefinedIndexT;
      views[v.id_view] = std::make_shared<View>(v);
    }
  }

  // b. Init vector of imageDescriber 
  
  struct DescriberMethod
  {
    std::string typeName;
    EImageDescriberType type;
    std::shared_ptr<Image_describer> describer; //TODO
  };
  std::vector<DescriberMethod> imageDescribers;
  
  {
    if(describerMethods.empty())
    {
      std::cerr << "\nError: describerMethods argument is empty." << std::endl;
      return EXIT_FAILURE;
    }
    std::vector<EImageDescriberType> describerMethodsVec = EImageDescriberType_stringToEnums(describerMethods);

    for(const auto& describerMethod: describerMethodsVec)
    {
      DescriberMethod method;
      method.typeName = EImageDescriberType_enumToString(describerMethod); // TODO: DELI ?
      method.type = describerMethod;
      method.describer = createImageDescriber(method.type);
      method.describer->Set_configuration_preset(featurePreset);
      method.describer->setUpRight(describersAreUpRight);
      imageDescribers.push_back(method);
    }
  }

  using namespace openMVG::features;

  // Feature extraction routines
  // For each View of the SfM_Data container:
  // - if regions file exist continue,
  // - if no file, compute features
  {
    system::Timer timer;
    C_Progress_display my_progress_bar( sfm_data.GetViews().size(),
      std::cout, "\n- EXTRACT FEATURES -\n" );

    Views::const_iterator iterViews = sfm_data.views.begin();
    Views::const_iterator iterViewsEnd = sfm_data.views.end();
    
    if(rangeStart != -1)
    {
      if(rangeStart < 0 || rangeStart > sfm_data.views.size())
      {
        std::cerr << "Bad specific index" << std::endl;
        return EXIT_FAILURE;
      }
      if(rangeSize < 0)
      {
        std::cerr << "Bad range size. " << std::endl;
        return EXIT_FAILURE;
      }
      if(rangeStart + rangeSize > sfm_data.views.size())
        rangeSize = sfm_data.views.size() - rangeStart;

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
      const std::string viewFilename = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);
      std::cout << "Extract features in view: " << viewFilename << std::endl;
      
      std::vector<DescriberComputeMethod> computeMethods;
      
      for(std::size_t i = 0; i < imageDescribers.size(); ++i)
      {
        DescriberComputeMethod computeMethod;
        
        computeMethod.featFilename = stlplus::create_filespec(outDirectory,
              stlplus::basename_part(std::to_string(view->id_view)), imageDescribers[i].typeName + ".feat");
        computeMethod.descFilename = stlplus::create_filespec(outDirectory,
              stlplus::basename_part(std::to_string(view->id_view)), imageDescribers[i].typeName + ".desc");
      
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
            Image<unsigned char> imageGray;
            if (!ReadImage(viewFilename.c_str(), &imageGray))
              return;

            for(auto& compute : computeMethods)
            {
              // Compute features and descriptors and export them to files
              std::cout << "Extracting "<< imageDescribers[compute.methodIndex].typeName  << " features from image " << view->id_view << std::endl;
              std::unique_ptr<Regions> regions;
              imageDescribers[compute.methodIndex].describer->Describe(imageGray, regions);
              imageDescribers[compute.methodIndex].describer->Save(regions.get(), compute.featFilename, compute.descFilename);
            }
        };
        
        if (maxJobs != MAX_JOBS_DEFAULT)
          dispatch(maxJobs, computeFunction);
        else
          computeFunction();
      }
    }

    if (maxJobs != MAX_JOBS_DEFAULT) waitForCompletion();

    std::cout << "Task done in (s): " << timer.elapsed() << std::endl;
  }
  return EXIT_SUCCESS;
}
