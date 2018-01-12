// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfm/sfm.hpp>
#include <aliceVision/image/all.hpp>
#include <aliceVision/image/convertion.hpp>
#include <aliceVision/config.hpp>

#include <boost/program_options.hpp>
#include <boost/progress.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iterator>
#include <iomanip>

using namespace aliceVision;
using namespace aliceVision::camera;
using namespace aliceVision::geometry;
using namespace aliceVision::image;
using namespace aliceVision::sfm;
namespace po = boost::program_options;

class point2d
{
public:
  point2d()
  : x(0), y(0)
  {}
  
    union {
        struct
        {
            double x, y;
        };
        double m[2];
    };
};

class point3d
{
public:
  point3d()
  : x(0), y(0), z(0)
  {}
  
    union {
        struct
        {
            double x, y, z;
        };
        double m[3];
    };
};

struct orientedPoint
{
    point3d p; // 3 * float : 3 * 4 = 12 Bytes : (one float is 4 Bytes : 3.4E +/- 38 (7 digits) )
    point3d n; // 2 * float : 2 * 4 = 8  Bytes
    float sim = 0; // 4-Bytes : 3.4E +/- 38 (7 digits)
    // TOTAL: 12 + 8 + 4 = 24 Bytes
};

struct seed_io_block
{
    orientedPoint op;       // 28 bytes
    point3d xax;            // 12 bytes
    point3d yax;            // 12 bytes
    float pixSize;          // 4 bytes
    uint64_t area;          // 8 bytes
    uint64_t segId;         // 8 bytes
    unsigned short ncams;   // 2 bytes
    unsigned short padding[3];
};

struct Seed
{
    seed_io_block s;
    unsigned short camId = 0;
    point2d shift0;
    point2d shift1;
};

typedef std::vector<Seed> SeedVector;
typedef stl::flat_map< size_t, SeedVector> SeedsPerView;

void retrieveSeedsPerView(
    const SfMData & sfm_data,
    const HashMap<IndexT, IndexT> map_viewIdToContiguous,
    SeedsPerView& outSeedsPerView)
{
  static const double minAngle = 3.0;
  for(const auto& s: sfm_data.structure)
  {
    const IndexT landmarkId = s.first;
    const Landmark& landmark = s.second;
    // For each observation of a 3D landmark, we will export
    // all other observations with an angle > minAngle.
    for(const auto& obsA: landmark.observations)
    {
      const auto& obsACamId_it = map_viewIdToContiguous.find(obsA.first);
      if(obsACamId_it == map_viewIdToContiguous.end())
        continue; // this view cannot be exported to mvs, so we skip the observation
      int obsACamId = obsACamId_it->second;
      const View& viewA = *sfm_data.GetViews().at(obsA.first).get();
      const geometry::Pose3& poseA = sfm_data.GetPoses().at(viewA.getPoseId());
      const Pinhole * intrinsicsA = dynamic_cast<const Pinhole*>(sfm_data.GetIntrinsics().at(viewA.getIntrinsicId()).get());
      
      for(const auto& obsB: landmark.observations)
      {
        // don't export itself
        if(obsA.first == obsB.first)
          continue;
        const auto& obsBCamId_it = map_viewIdToContiguous.find(obsB.first);
        if(obsBCamId_it == map_viewIdToContiguous.end())
          continue; // this view cannot be exported to mvs, so we skip the observation
        const View& viewB = *sfm_data.GetViews().at(obsB.first).get();
        const geometry::Pose3& poseB = sfm_data.GetPoses().at(viewB.getPoseId());
        const Pinhole * intrinsicsB = dynamic_cast<const Pinhole*>(sfm_data.GetIntrinsics().at(viewB.getIntrinsicId()).get());

        const double angle = AngleBetweenRays(
          poseA, intrinsicsA, poseB, intrinsicsB, obsA.second.x, obsB.second.x);
 
        if(angle < minAngle)
          continue;

        Seed seed;
        seed.camId = obsBCamId_it->second - 1; // Get 0-based index this time
        seed.s.ncams = 1;
        seed.s.segId = landmarkId;
        seed.s.op.p.x = landmark.X(0);
        seed.s.op.p.y = landmark.X(1);
        seed.s.op.p.z = landmark.X(2);

        outSeedsPerView[obsACamId].push_back(seed);
      }
    }
  }
}

std::string replaceAll( std::string const& original, std::string const& from, std::string const& to )
{
    std::string results;
    std::string::const_iterator end = original.end();
    std::string::const_iterator current = original.begin();
    std::string::const_iterator next = std::search( current, end, from.begin(), from.end() );
    while ( next != end ) {
        results.append( current, next );
        results.append( to );
        current = next + from.size();
        next = std::search( current, end, from.begin(), from.end() );
    }
    results.append( current, next );
    return results;
}

bool prepareDenseScene(
  const SfMData & sfm_data,
  int scale,
  image::EImageFileType outputFileType,
  const std::string & sOutDirectory)
{
  // As the MVS requires contiguous camera indexes and some views may not have a pose,
  // we reindex the poses to ensure a contiguous pose list.
  HashMap<IndexT, IndexT> map_viewIdToContiguous;
  // Export valid views as Projective Cameras:
  for(const auto &iter : sfm_data.GetViews())
  {
    const View * view = iter.second.get();
    if (!sfm_data.IsPoseAndIntrinsicDefined(view))
      continue;
    Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());
    const IntrinsicBase * cam = iterIntrinsic->second.get();
    // View Id re-indexing
    // Need to start at 1 for MVS
    map_viewIdToContiguous.insert(std::make_pair(view->getViewId(), map_viewIdToContiguous.size() + 1));
  }

  SeedsPerView seedsPerView;
  retrieveSeedsPerView(sfm_data, map_viewIdToContiguous, seedsPerView);
  
  // Export data
  boost::progress_display my_progress_bar(map_viewIdToContiguous.size(),
                                     std::cout, "\n- Exporting Data -\n");

  // Export views:
  //   - 00001_P.txt (Pose of the reconstructed camera)
  //   - 00001.exr (undistorted & scaled colored image)
  //   - 00001_seeds.bin (3d points visible in this image)
  #pragma omp parallel for num_threads(3)
  for(int i = 0; i < map_viewIdToContiguous.size(); ++i)
  {
    auto viewIdToContiguous = map_viewIdToContiguous.cbegin();
    std::advance(viewIdToContiguous, i);
    const IndexT viewId = viewIdToContiguous->first;
    const View * view = sfm_data.GetViews().at(viewId).get();

    assert(view->getViewId() == viewId);
    const IndexT contiguousViewIndex = viewIdToContiguous->second;
    Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->getIntrinsicId());
    // We have a valid view with a corresponding camera & pose
    assert(viewIdToContiguous->second == i + 1);

    std::ostringstream baseFilenameSS;
    baseFilenameSS << std::setw(5) << std::setfill('0') << contiguousViewIndex;
    const std::string baseFilename = baseFilenameSS.str();

    // Export camera pose
    {
      const Pose3 pose = sfm_data.getPose(*view);
      Mat34 P = iterIntrinsic->second.get()->get_projective_equivalent(pose);
      std::ofstream file(
        stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory),
        baseFilename + "_P", "txt").c_str());
      file << std::setprecision(10)
           << P(0, 0) / (double)scale << " " << P(0, 1) / (double)scale << " "  << P(0, 2) / (double)scale << " "  << P(0, 3) / (double)scale << "\n"
           << P(1, 0) / (double)scale << " " << P(1, 1) / (double)scale << " "  << P(1, 2) / (double)scale << " "  << P(1, 3) / (double)scale << "\n"
           << P(2, 0) << " " << P(2, 1) << " "  << P(2, 2) << " "  << P(2, 3) << "\n";
      file.close();
    }
    
    // Export undistort image
    {
      const std::string srcImage = view->getImagePath();
      std::string dstColorImage = stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory), baseFilename, image::EImageFileType_enumToString(outputFileType));

      const IntrinsicBase * cam = iterIntrinsic->second.get();
      Image<RGBfColor> image, image_ud, image_ud_scaled;

      readImage(srcImage, image);
      
      // Undistort
      if (cam->isValid() && cam->have_disto())
      {
        // undistort the image and save it
        UndistortImage(image, cam, image_ud, FBLACK);
      }
      else
      {
        image_ud = image;
      }
      
      // Rescale
      if(scale == 1)
      {
        image_ud_scaled = image_ud;
      }
      else if(scale == 2)
      {
        ImageHalfSample(image_ud, image_ud_scaled);
      }
      else if(scale == 4)
      {
        ImageHalfSample(image_ud, image_ud_scaled); // 2
        image_ud = image_ud_scaled;
        ImageHalfSample(image_ud, image_ud_scaled); // 4
      }
      else if(scale == 8)
      {
        ImageHalfSample(image_ud, image_ud_scaled); // 2
        ImageHalfSample(image_ud_scaled, image_ud); // 4
        ImageHalfSample(image_ud, image_ud_scaled); // 8
      }
      else if(scale == 16)
      {
        ImageHalfSample(image_ud, image_ud_scaled); // 2
        ImageHalfSample(image_ud_scaled, image_ud); // 4
        ImageHalfSample(image_ud, image_ud_scaled); // 8
        image_ud = image_ud_scaled;
        ImageHalfSample(image_ud, image_ud_scaled); // 16
      }
      else
      {
        std::cerr << "Rescale not implemented." << std::endl;
        image_ud_scaled = image_ud;
      }
      writeImage(dstColorImage, image_ud_scaled);
    }
    
    // Export Seeds
    {
      const std::string seedsFilepath = stlplus::create_filespec(
        stlplus::folder_append_separator(sOutDirectory), baseFilename + "_seeds", "bin");
      std::ofstream seedsFile(seedsFilepath, std::ios::binary);
      
      const int nbSeeds = seedsPerView[contiguousViewIndex].size();
      seedsFile.write((char*)&nbSeeds, sizeof(int));
      
      for(const Seed& seed: seedsPerView[contiguousViewIndex])
      {
        seedsFile.write((char*)&seed, sizeof(seed_io_block) + sizeof(unsigned short) + 2 * sizeof(point2d)); //sizeof(Seed));
      }
      seedsFile.close();
    }
   #pragma omp critical
    ++my_progress_bar;
  }

  // Write the mvs ini file
  std::ostringstream os;
  os << "[global]" << os.widen('\n')
  << "outDir=../../meshes" << os.widen('\n')
  << "ncams=" << map_viewIdToContiguous.size() << os.widen('\n')
  << "scale=" << scale << os.widen('\n')
  << "imgExt=" << image::EImageFileType_enumToString(outputFileType) << os.widen('\n')
  << "verbose=TRUE" << os.widen('\n')
  << os.widen('\n')
  << "[imageResolutions]" << os.widen('\n');
  for(const auto& viewIdToContiguous: map_viewIdToContiguous)
  {
    const IndexT viewId = viewIdToContiguous.first;
    const View * view = sfm_data.GetViews().at(viewId).get();
    const IndexT contiguousViewIndex = viewIdToContiguous.second;

    std::ostringstream baseFilenameSS;
    baseFilenameSS << std::setw(5) << std::setfill('0') << contiguousViewIndex;
    const std::string baseFilename = baseFilenameSS.str();

    os << baseFilename << "=" << int(view->getWidth() / (double)scale) << "x" << int(view->getHeight() / (double)scale) << os.widen('\n');
  }

  std::ofstream file2(
    stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory),
    "mvs", "ini").c_str());
  file2 << os.str();
  file2.close();

  return true;
}

int main(int argc, char *argv[])
{
  // command-line parameters

  std::string verboseLevel = system::EVerboseLevel_enumToString(system::Logger::getDefaultVerboseLevel());
  std::string sfmDataFilename;
  std::string outFolder;
  std::string outImageFileTypeName = image::EImageFileType_enumToString(image::EImageFileType::EXR);
  int scale = 2;

  po::options_description allParams("AliceVision prepareDenseScene");

  po::options_description requiredParams("Required parameters");
  requiredParams.add_options()
    ("input,i", po::value<std::string>(&sfmDataFilename)->required(),
      "SfMData file.")
    ("output,o", po::value<std::string>(&outFolder)->required(),
      "Output folder.");

  po::options_description optionalParams("Optional parameters");
  optionalParams.add_options()
    ("scale", po::value<int>(&scale)->default_value(scale),
      "Image downscale factor.")
    ("outputFileType", po::value<std::string>(&outImageFileTypeName)->default_value(outImageFileTypeName),
      image::EImageFileType_informations().c_str());

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

  // set verbose level
  system::Logger::get()->setLogLevel(verboseLevel);

  // set output file type
  image::EImageFileType outputFileType = image::EImageFileType_stringToEnum(outImageFileTypeName);

  // export
  {
    outFolder = stlplus::folder_to_path(outFolder);

    // Create output dir
    if (!stlplus::folder_exists(outFolder))
      stlplus::folder_create(outFolder);

    // Read the input SfM scene
    SfMData sfm_data;
    if (!Load(sfm_data, sfmDataFilename, ESfMData(ALL)))
    {
      std::cerr << std::endl
        << "The input SfMData file \""<< sfmDataFilename << "\" cannot be read." << std::endl;
      return EXIT_FAILURE;
    }

    if (!prepareDenseScene(sfm_data, scale, outputFileType, outFolder))
      return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
