#include <openMVG/rig/Rig.hpp>
#include <openMVG/logger.hpp>

#include <openMVG/sfm/sfm_data.hpp>
#include <openMVG/sfm/sfm_data_io.hpp>

#if HAVE_ALEMBIC
#include <openMVG/sfm/AlembicExporter.hpp>
#endif // HAVE_ALEMBIC

#include <boost/filesystem.hpp>
#include <boost/progress.hpp>
#include <boost/program_options.hpp> 
#include <iostream>
#include <string>
#include <chrono>
#include <memory>

namespace bfs = boost::filesystem;
namespace po = boost::program_options;

using namespace openMVG;
using namespace openMVG::sfm;

static std::vector<double> ReadIntrinsicsFile(const std::string& fname)
{
  cout << "reading intrinsics: " << fname << endl;

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
  std::vector<openMVG::geometry::Pose3> extrinsics;  // the rig subposes
  
  po::options_description desc("This program is used to transform cameras localized with a rig file.");
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
      POPART_COUT(desc);
      return EXIT_SUCCESS;
    }

    po::notify(vm);
  }
  catch(boost::program_options::required_option& e)
  {
    POPART_CERR("ERROR: " << e.what() << std::endl);
    POPART_COUT("Usage:\n\n" << desc);
    return EXIT_FAILURE;
  }
  catch(boost::program_options::error& e)
  {
    POPART_CERR("ERROR: " << e.what() << std::endl);
    POPART_COUT("Usage:\n\n" << desc);
    return EXIT_FAILURE;
  }
  // just debugging prints
  {
    POPART_COUT("Program called with the following parameters:");
    POPART_COUT("\timportFile: " << importFile);
    POPART_COUT("\texportFile: " << exportFile);
    POPART_COUT("\trigFile: " << rigFile);
    POPART_COUT("\tcalibFile: " << calibFile);

  }

  // Load rig calibration file
  if(!rig::loadRigCalibration(rigFile, extrinsics))
  {
    cerr << "unable to open " << rigFile << endl;
    exit(1);
  }
  assert(!extrinsics.empty());

  // Import sfm data
  int flags = sfm::ESfM_Data::ALL;
  SfM_Data sfmData;
  Load(sfmData, importFile, ESfM_Data(flags));

  // Load intrinsics
  auto v = ReadIntrinsicsFile(calibFile);
  openMVG::cameras::Pinhole_Intrinsic_Radial_K3 intrinsics = openMVG::cameras::Pinhole_Intrinsic_Radial_K3(v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);

  // Export to abc
#if HAVE_ALEMBIC
  dataio::AlembicExporter exporter( exportFile );
  exporter.initAnimatedCamera("camera");

  size_t idx = 0;
  for (auto &p : sfmData.poses)
  {
    const openMVG::geometry::Pose3 rigPose = extrinsics[0].inverse() * p.second;
    exporter.addCameraKeyframe(rigPose, &intrinsics, "", idx, idx);
    ++idx;
  }
  exporter.addPoints(sfmData.GetLandmarks());
#endif

  POPART_COUT("Done.");
}
