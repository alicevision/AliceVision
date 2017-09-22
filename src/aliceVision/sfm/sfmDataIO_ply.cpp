// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "sfmDataIO_ply.hpp"

#include <fstream>

namespace aliceVision {
namespace sfm {

/// Save the structure and camera positions of a SfMData container as 3D points in a PLY ASCII file.
 bool Save_PLY(
  const SfMData & sfm_data,
  const std::string & filename,
  ESfMData flags_part)
{
  const bool b_structure = (flags_part & STRUCTURE) == STRUCTURE;
  const bool b_extrinsics = (flags_part & EXTRINSICS) == EXTRINSICS;

  if (!(b_structure || b_extrinsics))
    return false;

  //Create the stream and check it is ok
  std::ofstream stream(filename.c_str());
  if (!stream.is_open())
    return false;

  bool bOk = false;
  {
    // Count how many views having valid poses:
    IndexT view_with_pose_count = 0;
    if (b_extrinsics)
    {
      for (const auto & view : sfm_data.GetViews())
      {
        view_with_pose_count += sfm_data.IsPoseAndIntrinsicDefined(view.second.get());
      }
    }
    stream << "ply"
      << '\n' << "format ascii 1.0"
      << '\n' << "element vertex "
        // Vertex count: (#landmark + #view_with_valid_pose)
        << ((b_structure ? sfm_data.GetLandmarks().size() : 0) +
            view_with_pose_count)
      << '\n' << "property float x"
      << '\n' << "property float y"
      << '\n' << "property float z"
      << '\n' << "property uchar red"
      << '\n' << "property uchar green"
      << '\n' << "property uchar blue"
      << '\n' << "end_header" << std::endl;

      if (b_extrinsics)
      {
        for (const auto & view : sfm_data.GetViews())
        {
          if (sfm_data.IsPoseAndIntrinsicDefined(view.second.get()))
          {
            const geometry::Pose3 pose = sfm_data.getPose(*(view.second.get()));
            stream << pose.center().transpose()
              << " 0 255 0" << "\n";
          }
        }
      }

      if (b_structure)
      {
        const Landmarks & landmarks = sfm_data.GetLandmarks();
        for (Landmarks::const_iterator iterLandmarks = landmarks.begin();
          iterLandmarks != landmarks.end();
          ++iterLandmarks)  {
          stream << iterLandmarks->second.X.transpose() << " "
                 << (int)iterLandmarks->second.rgb.r() << " "
                 << (int)iterLandmarks->second.rgb.g() << " "
                 << (int)iterLandmarks->second.rgb.b() << "\n";
        }
      }
      stream.flush();
      bOk = stream.good();
      stream.close();
  }
  return bOk;
}

} // namespace sfm
} // namespace aliceVision
