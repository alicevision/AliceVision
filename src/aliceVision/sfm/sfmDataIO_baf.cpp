// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#include "sfmDataIO_baf.hpp"

#include <fstream>

namespace aliceVision {
namespace sfm {

/// Save SfMData in an ASCII BAF (Bundle Adjustment File).
// --Header
// #Intrinsics
// #Poses
// #Landmarks
// --Data
// Intrinsic parameters [foc ppx ppy, ...]
// Poses [angle axis, camera center]
// Landmarks [X Y Z #observations id_intrinsic id_pose x y ...]
//--
//- Export also a _imgList.txt file with View filename and id_intrinsic & id_pose.
// filename id_intrinsic id_pose
// The ids allow to establish a link between 3D point observations & the corresponding views
//--
// Export missing poses as Identity pose to keep tracking of the original id_pose indexes
bool Save_BAF(
  const SfMData & sfm_data,
  const std::string & filename,
  ESfMData flags_part)
{
  std::ofstream stream(filename.c_str());
  if (!stream.is_open())
    return false;

  bool bOk = false;
  {
    stream
      << sfm_data.GetIntrinsics().size() << '\n'
      << sfm_data.GetViews().size() << '\n'
      << sfm_data.GetLandmarks().size() << '\n';

    const Intrinsics & intrinsics = sfm_data.GetIntrinsics();
    for (Intrinsics::const_iterator iterIntrinsic = intrinsics.begin();
      iterIntrinsic != intrinsics.end(); ++iterIntrinsic)
    {
      //get params
      const std::vector<double> intrinsicsParams = iterIntrinsic->second.get()->getParams();
      std::copy(intrinsicsParams.begin(), intrinsicsParams.end(),
        std::ostream_iterator<double>(stream, " "));
      stream << '\n';
    }

    const Poses & poses = sfm_data.GetPoses();
    for (Views::const_iterator iterV = sfm_data.GetViews().begin();
      iterV != sfm_data.GetViews().end();
      ++ iterV)
    {
      const View * view = iterV->second.get();
      if (!sfm_data.IsPoseAndIntrinsicDefined(view))
      {
        const Mat3 R = Mat3::Identity();
        const double * rotation = R.data();
        std::copy(rotation, rotation+9, std::ostream_iterator<double>(stream, " "));
        const Vec3 C = Vec3::Zero();
        const double * center = C.data();
        std::copy(center, center+3, std::ostream_iterator<double>(stream, " "));
        stream << '\n';
      }
      else
      {
        // [Rotation col major 3x3; camera center 3x1]
        const double * rotation = poses.at(view->getPoseId()).rotation().data();
        std::copy(rotation, rotation+9, std::ostream_iterator<double>(stream, " "));
        const double * center = poses.at(view->getPoseId()).center().data();
        std::copy(center, center+3, std::ostream_iterator<double>(stream, " "));
        stream << '\n';
      }
    }

    const Landmarks & landmarks = sfm_data.GetLandmarks();
    for (Landmarks::const_iterator iterLandmarks = landmarks.begin();
      iterLandmarks != landmarks.end();
      ++iterLandmarks)
    {
      // Export visibility information
      // X Y Z #observations id_cam id_pose x y ...
      const double * X = iterLandmarks->second.X.data();
      std::copy(X, X+3, std::ostream_iterator<double>(stream, " "));
      const Observations & observations = iterLandmarks->second.observations;
      stream << observations.size() << " ";
      for (Observations::const_iterator iterOb = observations.begin();
        iterOb != observations.end(); ++iterOb)
      {
        const IndexT id_view = iterOb->first;
        const View * v = sfm_data.GetViews().at(id_view).get();
        stream
          << v->getIntrinsicId() << ' '
          << v->getPoseId() << ' '
          << iterOb->second.x(0) << ' ' << iterOb->second.x(1) << ' ';
      }
      stream << '\n';
    }

    stream.flush();
    bOk = stream.good();
    stream.close();
  }

  // Export View filenames & ids as an imgList.txt file
  {
    const std::string sFile = stlplus::create_filespec(
      stlplus::folder_part(filename), stlplus::basename_part(filename) + std::string("_imgList"), "txt");

    stream.open(sFile.c_str());
    if (!stream.is_open())
      return false;
    for (Views::const_iterator iterV = sfm_data.GetViews().begin();
      iterV != sfm_data.GetViews().end();
      ++ iterV)
    {
      const std::string sView_filename = stlplus::create_filespec(sfm_data.s_root_path,
        iterV->second->getImagePath());
      stream
        << sView_filename
        << ' ' << iterV->second->getIntrinsicId()
        << ' ' << iterV->second->getPoseId() << "\n";
    }
    stream.flush();
    bOk = stream.good();
    stream.close();
  }
  return bOk;
}

} // namespace sfm
} // namespace aliceVision
