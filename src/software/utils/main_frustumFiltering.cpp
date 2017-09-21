// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "aliceVision/sfm/sfm.hpp"
#include "aliceVision/system/Timer.hpp"
#include "aliceVision/matchingImageCollection/pairBuilder.hpp"

#include "dependencies/cmdLine/cmdLine.h"
#include "dependencies/stlplus3/filesystemSimplified/file_system.hpp"

#include <cstdlib>

using namespace aliceVision;
using namespace aliceVision::sfm;

/// Build a list of pair that share visibility content from the SfMData structure
Pair_Set BuildPairsFromStructureObservations(const SfMData & sfm_data)
{
  Pair_Set pairs;

  for (Landmarks::const_iterator itL = sfm_data.GetLandmarks().begin();
    itL != sfm_data.GetLandmarks().end(); ++itL)
  {
    const Landmark & landmark = itL->second;
    for(const auto& iterI : landmark.observations)
    {
      const IndexT id_viewI = iterI.first;
      Observations::const_iterator iterJ = landmark.observations.begin();
      std::advance(iterJ, 1);
      for (; iterJ != landmark.observations.end(); ++iterJ)
      {
        const IndexT id_viewJ = iterJ->first;
        pairs.insert( std::make_pair(id_viewI,id_viewJ));
      }
    }
  }
  return pairs;
}

/// Build a list of pair from the camera frusta intersections
Pair_Set BuildPairsFromFrustumsIntersections(
  const SfMData & sfm_data,
  const double z_near = -1., // default near plane
  const double z_far = -1.,  // default far plane
  const std::string& sOutDirectory = "") // output directory to save frustums as PLY
{
  const FrustumFilter frustum_filter(sfm_data, z_near, z_far);
  if (!sOutDirectory.empty())
    frustum_filter.export_Ply(stlplus::create_filespec(sOutDirectory, "frustums.ply"));
  return frustum_filter.getFrustumIntersectionPairs();
}

int main(int argc, char **argv)
{
  using namespace std;
  std::cout << std::endl
    << "-----------------------------------------------------------\n"
    << "Frustum filtering:\n"
    << "-----------------------------------------------------------\n"
    << "Compute camera cones that share some putative visual content.\n"
    << "------------------------------------------------------------"
    << std::endl;

  CmdLine cmd;

  std::string sSfMData_Filename;
  std::string sOutFile;
  double z_near = -1.;
  double z_far = -1.;

  cmd.add( make_option('i', sSfMData_Filename, "input_file") );
  cmd.add( make_option('o', sOutFile, "output_file") );
  cmd.add( make_option('n', z_near, "z_near") );
  cmd.add( make_option('f', z_far, "z_far") );

  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] path to a SfMData scene\n"
    << "[-o|--output_file] filename of the output pair file\n"
    << "[-n|--z_near] 'optional' distance of the near camera plane\n"
    << "[-f|--z_far] 'optional' distance of the far camera plane\n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  // Assert that we can create the output directory
  if (!stlplus::folder_exists( stlplus::folder_part(sOutFile) ))
    if(!stlplus::folder_create( stlplus::folder_part(sOutFile) ))
      return EXIT_FAILURE;

  // Load input SfMData scene
  SfMData sfm_data;
  if (!Load(sfm_data, sSfMData_Filename, ESfMData(ALL))) {
    std::cerr << std::endl
      << "The input SfMData file \""<< sSfMData_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  aliceVision::system::Timer timer;

  const Pair_Set pairs = BuildPairsFromFrustumsIntersections(sfm_data, z_near, z_far, stlplus::folder_part(sOutFile));
  /*const Pair_Set pairs = BuildPairsFromStructureObservations(sfm_data); */

  std::cout << "#pairs: " << pairs.size() << std::endl;
  std::cout << std::endl << " Pair filtering took (s): " << timer.elapsed() << std::endl;

  // export pairs on disk
  if (savePairs(sOutFile, pairs))
  {
    return EXIT_SUCCESS;
  }
  else
  {
    return EXIT_FAILURE;
  }


}
