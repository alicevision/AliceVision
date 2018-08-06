// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "bafIO.hpp"

#include <boost/filesystem.hpp>

#include <fstream>

namespace fs = boost::filesystem;

namespace aliceVision {
namespace sfmDataIO {

bool saveBAF(
  const sfmData::SfMData& sfmData,
  const std::string& filename,
  ESfMData partFlag)
{
  std::ofstream stream(filename.c_str());
  if (!stream.is_open())
    return false;

  bool bOk = false;
  {
    stream
      << sfmData.getIntrinsics().size() << '\n'
      << sfmData.getViews().size() << '\n'
      << sfmData.getLandmarks().size() << '\n';

    const sfmData::Intrinsics& intrinsics = sfmData.getIntrinsics();
    for (sfmData::Intrinsics::const_iterator iterIntrinsic = intrinsics.begin();
      iterIntrinsic != intrinsics.end(); ++iterIntrinsic)
    {
      //get params
      const std::vector<double> intrinsicsParams = iterIntrinsic->second.get()->getParams();
      std::copy(intrinsicsParams.begin(), intrinsicsParams.end(),
        std::ostream_iterator<double>(stream, " "));
      stream << '\n';
    }

    const sfmData::Poses& poses = sfmData.getPoses();
    for (sfmData::Views::const_iterator iterV = sfmData.getViews().begin();
      iterV != sfmData.getViews().end();
      ++ iterV)
    {
      const sfmData::View* view = iterV->second.get();
      if (!sfmData.isPoseAndIntrinsicDefined(view))
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
        const double * rotation = poses.at(view->getPoseId()).getTransform().rotation().data();
        std::copy(rotation, rotation+9, std::ostream_iterator<double>(stream, " "));
        const double * center = poses.at(view->getPoseId()).getTransform().center().data();
        std::copy(center, center+3, std::ostream_iterator<double>(stream, " "));
        stream << '\n';
      }
    }

    const sfmData::Landmarks& landmarks = sfmData.getLandmarks();
    for (sfmData::Landmarks::const_iterator iterLandmarks = landmarks.begin();
      iterLandmarks != landmarks.end();
      ++iterLandmarks)
    {
      // Export visibility information
      // X Y Z #observations id_cam id_pose x y ...
      const double * X = iterLandmarks->second.X.data();
      std::copy(X, X+3, std::ostream_iterator<double>(stream, " "));
      const sfmData::Observations& observations = iterLandmarks->second.observations;
      stream << observations.size() << " ";
      for (sfmData::Observations::const_iterator iterOb = observations.begin();
        iterOb != observations.end(); ++iterOb)
      {
        const IndexT id_view = iterOb->first;
        const sfmData::View * v = sfmData.getViews().at(id_view).get();
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
    const std::string sFile = (fs::path(filename).parent_path() / (fs::path(filename).stem().string() + "_imgList.txt")).string();

    stream.open(sFile.c_str());
    if (!stream.is_open())
      return false;
    for (sfmData::Views::const_iterator iterV = sfmData.getViews().begin();
      iterV != sfmData.getViews().end();
      ++ iterV)
    {
      const std::string sView_filename = iterV->second->getImagePath();
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

} // namespace sfmDataIO
} // namespace aliceVision
