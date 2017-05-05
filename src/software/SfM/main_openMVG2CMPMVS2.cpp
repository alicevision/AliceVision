#include "openMVG/sfm/sfm.hpp"
#include "openMVG/image/image.hpp"
#include "openMVG/image/image_converter.hpp"
#include <openMVG/config.hpp>


#include "third_party/cmdLine/cmdLine.h"
#include "third_party/progress/progress.hpp"

#include <stdlib.h>
#include <stdio.h>
#include <cmath>
#include <iterator>
#include <iomanip>

using namespace openMVG;
using namespace openMVG::cameras;
using namespace openMVG::geometry;
using namespace openMVG::image;
using namespace openMVG::sfm;

class point2d
{
public:
  point2d()
  : x(0), y(0)
  {}
  
    union {
        struct
        {
            float x, y;
        };
        float m[2];
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
            float x, y, z;
        };
        float m[3];
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
    const SfM_Data & sfm_data,
    const Hash_Map<IndexT, IndexT> map_viewIdToContiguous,
    SeedsPerView& outSeedsPerView)
{
  static const double minAngle = 3.0;
  for(const auto& s: sfm_data.structure)
  {
    const IndexT landmarkId = s.first;
    const Landmark& landmark = s.second;
    // For each observation of a 3D landmark, we will export
    // all other observations with an angle > minAngle.
    for(const auto& obsA: landmark.obs)
    {
      const auto& obsACamId_it = map_viewIdToContiguous.find(obsA.first);
      if(obsACamId_it == map_viewIdToContiguous.end())
        continue; // this view cannot be exported to cmpmvs, so we skip the observation
      int obsACamId = obsACamId_it->second;
      const View& viewA = *sfm_data.GetViews().at(obsA.first).get();
      const geometry::Pose3& poseA = sfm_data.GetPoses().at(viewA.id_pose);
      const Pinhole_Intrinsic * intrinsicsA = dynamic_cast<const Pinhole_Intrinsic*>(sfm_data.GetIntrinsics().at(viewA.id_intrinsic).get());
      
      for(const auto& obsB: landmark.obs)
      {
        // don't export itself
        if(obsA.first == obsB.first)
          continue;
        const auto& obsBCamId_it = map_viewIdToContiguous.find(obsB.first);
        if(obsBCamId_it == map_viewIdToContiguous.end())
          continue; // this view cannot be exported to cmpmvs, so we skip the observation
        const View& viewB = *sfm_data.GetViews().at(obsB.first).get();
        const geometry::Pose3& poseB = sfm_data.GetPoses().at(viewB.id_pose);
        const Pinhole_Intrinsic * intrinsicsB = dynamic_cast<const Pinhole_Intrinsic*>(sfm_data.GetIntrinsics().at(viewB.id_intrinsic).get());

        const double angle = AngleBetweenRay(
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

bool exportToCMPMVS2Format(
  const SfM_Data & sfm_data,
  int scale,
  const std::string & sOutDirectory // Output CMPMVS files directory
  )
{
  // Create basis directory structure
  if (!stlplus::is_folder(sOutDirectory))
  {
    stlplus::folder_create(sOutDirectory);
    bool bOk = stlplus::is_folder(sOutDirectory);
    if (!bOk)
    {
      std::cerr << "Cannot access the output directory: " << sOutDirectory << std::endl;
      return false;
    }
  }
  
  // Since CMPMVS requires contiguous camera indexes and some views may not have a pose,
  // we reindex the poses to ensure a contiguous pose list.
  Hash_Map<IndexT, IndexT> map_viewIdToContiguous;
  // Export valid views as Projective Cameras:
  for(const auto &iter : sfm_data.GetViews())
  {
    const View * view = iter.second.get();
    if (!sfm_data.IsPoseAndIntrinsicDefined(view))
      continue;
    Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);
    const IntrinsicBase * cam = iterIntrinsic->second.get();
    // View Id re-indexing
    // Need to start at 1 for CMPMVS
    map_viewIdToContiguous.insert(std::make_pair(view->id_view, map_viewIdToContiguous.size() + 1));
  }

  SeedsPerView seedsPerView;
  retrieveSeedsPerView(sfm_data, map_viewIdToContiguous, seedsPerView);
  
  // Export data
  C_Progress_display my_progress_bar(map_viewIdToContiguous.size(),
                                     std::cout, "\n- Exporting Data -\n");

  // Export views:
  //   - 00001_P.txt (Pose of the reconstructed camera)
  //   - 00001._c.png (undistorted & scaled colored image)
  //   - 00001_seeds.bin (3d points visible in this image)
  #pragma omp parallel for num_threads(3)
  for(int i = 0; i < map_viewIdToContiguous.size(); ++i)
  {
    auto viewIdToContiguous = map_viewIdToContiguous.cbegin();
    std::advance(viewIdToContiguous, i);
    IndexT viewId = viewIdToContiguous->first;
    const View * view = sfm_data.GetViews().at(viewId).get();
    assert(view->id_view == viewId);
    IndexT contiguousViewIndex = map_viewIdToContiguous[view->id_view];
    Intrinsics::const_iterator iterIntrinsic = sfm_data.GetIntrinsics().find(view->id_intrinsic);
    // We have a valid view with a corresponding camera & pose
    assert(map_viewIdToContiguous[view->id_view] == i + 1);

    std::ostringstream baseFilenameSS;
    baseFilenameSS << std::setw(5) << std::setfill('0') << contiguousViewIndex;
    const std::string baseFilename = baseFilenameSS.str();

    // Export camera pose
    {
      const Pose3 pose = sfm_data.GetPoseOrDie(view);
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
      const std::string srcImage = stlplus::create_filespec(sfm_data.s_root_path, view->s_Img_path);

      std::string dstColorImage = stlplus::create_filespec(
        stlplus::folder_append_separator(sOutDirectory), baseFilename + "._c", "png");

      const IntrinsicBase * cam = iterIntrinsic->second.get();
      Image<RGBColor> image;
      ReadImage(srcImage.c_str(), &image);
      
      // Undistort
      Image<RGBColor> image_ud;
      if (cam->isValid() && cam->have_disto())
      {
        // undistort the image and save it
        UndistortImage(image, cam, image_ud, BLACK);
      }
      else
      {
        image_ud = image;
      }
      
      // Rescale
      Image<RGBColor> image_ud_scaled;
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
      WriteImage(dstColorImage.c_str(), image_ud_scaled);
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

  // Write the cmpmvs ini file
  std::ostringstream os;
  os << "[global]" << os.widen('\n')
  << "outDir=\"../../meshes\"" << os.widen('\n')
  << "ncams=" << map_viewIdToContiguous.size() << os.widen('\n')
  << "scale=" << scale << os.widen('\n')
  << "verbose=TRUE" << os.widen('\n')
  << os.widen('\n')

  << "#EOF" << os.widen('\n')
  << os.widen('\n')
  << os.widen('\n');

  std::ofstream file2(
    stlplus::create_filespec(stlplus::folder_append_separator(sOutDirectory),
    "mvs", "ini").c_str());
  file2 << os.str();
  file2.close();

  return true;
}

int main(int argc, char *argv[])
{
  CmdLine cmd;
  std::string sSfM_Data_Filename;
  int scale = 2;
  std::string sOutDir = "";

  cmd.add( make_option('i', sSfM_Data_Filename, "sfmdata") );
  cmd.add( make_option('s', scale, "scale") );
  cmd.add( make_option('o', sOutDir, "outdir") );

  try {
      if (argc == 1) throw std::string("Invalid command line parameter.");
      cmd.process(argc, argv);
  } catch(const std::string& s) {
      std::cerr << "Usage: " << argv[0] << '\n'
      << "[-i|--sfmdata] filename, the SfM_Data file to convert\n"
      << "[-s|--scale] downscale image factor\n"
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
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(ALL)))
  {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  if (!exportToCMPMVS2Format(sfm_data, scale, stlplus::filespec_to_path(sOutDir, "_tmp_scale" + std::to_string(scale))))
    return EXIT_FAILURE;

  return EXIT_SUCCESS;
}
