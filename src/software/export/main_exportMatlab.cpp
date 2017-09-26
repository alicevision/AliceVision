// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/image/image.hpp"
#include "aliceVision/image/convertion.hpp"

#include "dependencies/cmdLine/cmdLine.h"

#include <boost/progress.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iterator>
#include <iomanip>
#include <map>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfm;


bool exportToMatlab(
  const SfMData & sfm_data,
  const std::string & outDirectory
  )
{
  // WARNING: Observation::id_feat is used to put the ID of the 3D landmark.
  std::map<IndexT, std::vector<Observation> > observationsPerView;
  
  {
    const std::string landmarksFilename = stlplus::filespec_to_path(outDirectory, "scene.landmarks");
    std::ofstream landmarksFile(landmarksFilename);
    landmarksFile << "# landmarkId X Y Z\n";
    for(const auto& s: sfm_data.structure)
    {
      const IndexT landmarkId = s.first;
      const Landmark& landmark = s.second;
      landmarksFile << landmarkId << " " << landmark.X[0] << " " << landmark.X[1] << " " << landmark.X[2] << "\n";
      for(const auto& obs: landmark.observations)
      {
        const IndexT obsView = obs.first; // The ID of the view that provides this 2D observation.
        observationsPerView[obsView].push_back(Observation(obs.second.x, landmarkId));
      }
    }
    landmarksFile.close();
  }
  
  // Export observations per view
  for(const auto & obsPerView : observationsPerView)
  {
    const std::vector<Observation>& viewObservations = obsPerView.second;
    const IndexT viewId = obsPerView.first;
    
    const std::string viewFeatFilename = stlplus::filespec_to_path(outDirectory, std::to_string(viewId) + ".reconstructedFeatures");
    std::ofstream viewFeatFile(viewFeatFilename);
    viewFeatFile << "# landmarkId x y\n";
    for(const Observation& obs: viewObservations)
    {
      viewFeatFile << obs.id_feat << " " << obs.x[0] << " " << obs.x[1] << "\n";
    }
    viewFeatFile.close();
  }
  
  // Expose camera poses
  {
    const std::string cameraPosesFilename = stlplus::filespec_to_path(outDirectory, "cameras.poses");
    std::ofstream cameraPosesFile(cameraPosesFilename);
    cameraPosesFile << "# viewId R11 R12 R13 R21 R22 R23 R31 R32 R33 C1 C2 C3\n";
    for(const auto& v: sfm_data.views)
    {
      const View& view = *v.second.get();
      if(!sfm_data.IsPoseAndIntrinsicDefined(&view))
        continue;
      const Pose3& pose = sfm_data.GetPoses().at(view.getPoseId());
      cameraPosesFile << view.getViewId()
        << " " << pose.rotation()(0, 0)
        << " " << pose.rotation()(0, 1)
        << " " << pose.rotation()(0, 2)
        << " " << pose.rotation()(1, 0)
        << " " << pose.rotation()(1, 1)
        << " " << pose.rotation()(1, 2)
        << " " << pose.rotation()(2, 0)
        << " " << pose.rotation()(2, 1)
        << " " << pose.rotation()(2, 2)
        << " " << pose.center()(0)
        << " " << pose.center()(1)
        << " " << pose.center()(2)
        << "\n";
    }
    cameraPosesFile.close();
  }
  
  // Expose camera intrinsics
  // Note: we export it per view, It is really redundant but easy to parse.
  {
    const std::string cameraIntrinsicsFilename = stlplus::filespec_to_path(outDirectory, "cameras.intrinsics");
    std::ofstream cameraIntrinsicsFile(cameraIntrinsicsFilename);
    cameraIntrinsicsFile <<
      "# viewId pinhole f u0 v0\n"
      "# viewId radial1 f u0 v0 k1\n"
      "# viewId radial3 f u0 v0 k1 k2 k3\n"
      "# viewId brown f u0 v0 k1 k2 k3 t1 t2\n"
      "# viewId fisheye4 f u0 v0 k1 k2 k3 k4\n"
      "# viewId fisheye1 f u0 v0 k1\n";

    for(const auto& v: sfm_data.views)
    {
      const View& view = *v.second.get();
      if(!sfm_data.IsPoseAndIntrinsicDefined(&view))
        continue;
      const IntrinsicBase& intrinsics = *sfm_data.intrinsics.at(view.getIntrinsicId()).get();
      cameraIntrinsicsFile << view.getViewId() << " " << camera::EINTRINSIC_enumToString(intrinsics.getType());
      for(double p: intrinsics.getParams())
        cameraIntrinsicsFile << " " << p;
      cameraIntrinsicsFile << "\n";
    }
    cameraIntrinsicsFile.close();
  }
  
  return true;
}

int main(int argc, char *argv[])
{
  CmdLine cmd;
  std::string sSfMData_Filename;
  std::string sOutDir = "";

  cmd.add( make_option('i', sSfMData_Filename, "sfmdata") );
  cmd.add( make_option('o', sOutDir, "outdir") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
        << "[-i|--sfmdata] filename, the SfMData file to convert\n"
        << "[-o|--outdir] path\n"
        << std::endl;

      std::cerr << s << std::endl;
      return EXIT_FAILURE;
  }

  sOutDir = stlplus::folder_to_path(sOutDir);

  // Create output dir
  if (!stlplus::folder_exists(sOutDir))
    stlplus::folder_create( sOutDir );

  // Read the input SfM scene
  SfMData sfm_data;
  if (!Load(sfm_data, sSfMData_Filename, ESfMData(ALL)))
  {
    std::cerr << std::endl
      << "The input SfMData file \""<< sSfMData_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if (!exportToMatlab(sfm_data, sOutDir))
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
