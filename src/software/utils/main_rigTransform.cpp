// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include <aliceVision/rig/Rig.hpp>
#include <aliceVision/system/Logger.hpp>

#include <aliceVision/sfm/sfm_data.hpp>
#include <aliceVision/sfm/sfm_data_io.hpp>

#include <aliceVision/sfm/AlembicExporter.hpp>

#include <boost/filesystem.hpp>
#include <boost/progress.hpp>
#include <boost/program_options.hpp> 
#include <iostream>
#include <string>
#include <chrono>
#include <memory>

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

using namespace aliceVision;
using namespace aliceVision::sfm;

static std::vector<double> ReadIntrinsicsFile(const std::string& fname)
{
  std::cout << "reading intrinsics: " << fname << std::endl;

  std::vector<double> v(8);
  std::ifstream ifs(fname);
  if (!(ifs >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5] >> v[6] >> v[7]))
    throw std::runtime_error("failed to read intrinsics file");
  return v;
}

int main(int argc, char** argv)
{
  std::string exportFile = "trackedcameras-rig.abc"; //!< the export file
  std::string importFile = "";
  std::string rigFile = "";
  std::string calibFile = "";
  std::vector<aliceVision::geometry::Pose3> extrinsics;  // the rig subposes
  
  po::options_description desc("If you have localized a single camera from an acquisition with a RIG of cameras, you can use this program to deduce the pose of the other cameras of the RIG.");
  desc.add_options()
        ("help,h", "Print this message")
        ("input,i", po::value<std::string>(&importFile)->required(),
            "The input file containing cameras.")
        ("output,o", po::value<std::string>(&exportFile)->default_value(exportFile),
          "Filename for the SfM_Data export file (where camera poses will be stored)."
          " Only Alembic supported for now. Default: trackedcameras-rig.abc.")
        ("rigFile,e", po::value<std::string>(&rigFile)->required(),
            "Rig calibration file that will be  applied to input.")
        ("calibrationFile,c", po::value<std::string>(&calibFile)->required(),
            "A calibration file for the target camera.")
          ;

  po::variables_map vm;

  try
  {
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if(vm.count("help") || (argc == 1))
    {
      ALICEVISION_COUT(desc);
      return EXIT_SUCCESS;
    }

    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    ALICEVISION_COUT("Usage:\n\n" << desc);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    ALICEVISION_CERR("ERROR: " << e.what() << std::endl);
    ALICEVISION_COUT("Usage:\n\n" << desc);
    return EXIT_FAILURE;
  }
  // just debugging prints
  {
    ALICEVISION_COUT("Program called with the following parameters:");
    ALICEVISION_COUT("\timportFile: " << importFile);
    ALICEVISION_COUT("\texportFile: " << exportFile);
    ALICEVISION_COUT("\trigFile: " << rigFile);
    ALICEVISION_COUT("\tcalibFile: " << calibFile);

  }

  // Load rig calibration file
  if(!rig::loadRigCalibration(rigFile, extrinsics))
  {
    std::cerr << "unable to open " << rigFile << std::endl;
    return EXIT_FAILURE;
  }
  assert(!extrinsics.empty());

  // Import sfm data
  int flags = sfm::ESfM_Data::ALL;
  SfM_Data sfmData;
  Load(sfmData, importFile, ESfM_Data(flags));

  // Load intrinsics
  auto v = ReadIntrinsicsFile(calibFile);
  aliceVision::cameras::Pinhole_Intrinsic_Radial_K3 intrinsics = aliceVision::cameras::Pinhole_Intrinsic_Radial_K3(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);

  // Export to abc
  aliceVision::sfm::AlembicExporter exporter( exportFile );
  exporter.initAnimatedCamera("camera");

  size_t idx = 0;
  for (auto &p : sfmData.GetPoses())
  {
    const aliceVision::geometry::Pose3 rigPose = extrinsics[0].inverse() * p.second;
    exporter.addCameraKeyframe(rigPose, &intrinsics, "", idx, idx);
    ++idx;
  }
  exporter.addPoints(sfmData.GetLandmarks());

  ALICEVISION_COUT("Done.");
  return EXIT_SUCCESS;
}

