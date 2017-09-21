// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/sfm/utils/uid.hpp"
#include <aliceVision/config.hpp>


#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_BOOST)
#include <boost/system/error_code.hpp>
#include <boost/filesystem.hpp>
#endif

#include <string>
#include <vector>

using namespace aliceVision;
using namespace aliceVision::sfm;

// Convert from a SfMData format to another
int main(int argc, char **argv)
{
  CmdLine cmd;

  std::string sSfMData_Filename_In;
  std::string sSfMData_Filename_Out;
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_BOOST)
  std::string matchDir;
#endif

  cmd.add(make_option('i', sSfMData_Filename_In, "input_file"));
  cmd.add(make_switch('V', "VIEWS"));
  cmd.add(make_switch('I', "INTRINSICS"));
  cmd.add(make_switch('E', "EXTRINSICS"));
  cmd.add(make_switch('S', "STRUCTURE"));
  cmd.add(make_switch('O', "OBSERVATIONS"));
  cmd.add(make_switch('C', "CONTROL_POINTS"));
  cmd.add(make_switch('u', "regenerateUID"));
  cmd.add(make_option('o', sSfMData_Filename_Out, "output_file"));
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_BOOST)
  cmd.add(make_option('m', matchDir, "matchDirectory"));
#endif

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--input_file] path to the input SfMData scene\n"
        << "[-o|--output_file] path to the output SfMData scene\n"
        << "\t .json, .bin, .xml, .ply, .baf"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_ALEMBIC)
           ", .abc"
#endif
           "\n"
        << "\n[Options to export partial data (by default all data are exported)]\n"
        << "\nUsable for json/bin/xml format\n"
        << "[-V|--VIEWS] export views\n"
        << "[-I|--INTRINSICS] export intrinsics\n"
        << "[-E|--EXTRINSICS] export extrinsics (view poses)\n"
        << "[-S|--STRUCTURE] export structure\n"
        << "[-O|--OBSERVATIONS] export 2D observations associated with 3D structure\n"
        << "[-C|--CONTROL_POINTS] export control points\n"
        << "[-u|--uid] (re-)compute the unique ID (UID) for the views\n"
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_BOOST)              
        << "[-m|--matchDirectory] the directory containing the features used for the\n"
           "    reconstruction. If provided along the -u option, it creates symbolic\n"
           "    links to the .desc and .feat with the new UID as name. This can be\n"
           "    for legacy reconstructions that were not made using UID"
#endif
        << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  if (sSfMData_Filename_In.empty() || sSfMData_Filename_Out.empty())
  {
    std::cerr << "Invalid input or output filename." << std::endl;
    return EXIT_FAILURE;
  }

  // OptionSwitch is cloned in cmd.add(),
  // so we must use cmd.used() instead of testing OptionSwitch.used
  int flags =
    (cmd.used('V') ? VIEWS      : 0)
  | (cmd.used('I') ? INTRINSICS : 0)
  | (cmd.used('E') ? EXTRINSICS : 0)
  | (cmd.used('O') ? OBSERVATIONS : 0)
  | (cmd.used('S') ? STRUCTURE  : 0);

  flags = (flags) ? flags : ALL;
  
  const bool recomputeUID = cmd.used('u');

  // Load input SfMData scene
  SfMData sfm_data;
  if (!Load(sfm_data, sSfMData_Filename_In, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \"" << sSfMData_Filename_In << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }
  
  if(recomputeUID)
  {
    std::cout << "Recomputing the UID of the views..." << std::endl;
    std::map<std::size_t, std::size_t> oldIdToNew;
    regenerateUID(sfm_data, oldIdToNew);
    
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_BOOST)
    if(!matchDir.empty())
    {
      std::cout << "Generating alias for .feat and .desc with the UIDs" << std::endl;
      for(const auto& iter : oldIdToNew)
      {
        const auto oldID = iter.first;
        const auto newID = iter.second;
        
        // nothing to do if the ids are the same
        if(oldID == newID)
          continue;
        
        const auto oldFeatfilename = stlplus::create_filespec(matchDir, std::to_string(oldID), ".feat");
        const auto newFeatfilename = stlplus::create_filespec(matchDir, std::to_string(newID), ".feat");
        const auto oldDescfilename = stlplus::create_filespec(matchDir, std::to_string(oldID), ".desc");
        const auto newDescfilename = stlplus::create_filespec(matchDir, std::to_string(newID), ".desc");

        if(!(stlplus::is_file(oldFeatfilename) && stlplus::is_file(oldDescfilename)))
        {
          std::cerr << "Cannot find the features file for view ID " << oldID
                      << std::endl;
          return EXIT_FAILURE;
        }
        boost::system::error_code ec;
        boost::filesystem::create_symlink(oldFeatfilename, newFeatfilename, ec);
        if(ec)
        {
          std::cerr << "Error while creating " << newFeatfilename << ": " << ec.message() << std::endl;
          return EXIT_FAILURE;
        }
        boost::filesystem::create_symlink(oldDescfilename, newDescfilename, ec);
        if(ec)
        {
          std::cerr << "Error while creating " << newDescfilename << ": " << ec.message() << std::endl;
          return EXIT_FAILURE;
        }
      }
    }
#endif
    
  }

  // Export the SfMData scene in the expected format
  if (!Save(sfm_data, sSfMData_Filename_Out, ESfMData(flags)))
  {
    std::cerr << std::endl
      << "An error occured while trying to save \"" << sSfMData_Filename_Out << "\"." << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
