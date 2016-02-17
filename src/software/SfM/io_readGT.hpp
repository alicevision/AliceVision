
// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef IO_READ_GT_HPP
#define IO_READ_GT_HPP

#include "openMVG/geometry/pose3.hpp"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

#include <fstream>
#include <vector>

namespace openMVG
{

using namespace openMVG::cameras;

static bool read_openMVG_Camera(const std::string & camName, Pinhole_Intrinsic & cam, geometry::Pose3 & pose)
{
  std::vector<double> val;
  if (stlplus::extension_part(camName) == "bin")
  {
    std::ifstream in(camName.c_str(), std::ios::in|std::ios::binary);
    if (!in.is_open())
    {
      std::cerr << "Error: failed to open file '" << camName << "' for reading" << std::endl;
      return false;
    }
    val.resize(12);
    in.read((char*)&val[0],(std::streamsize)12*sizeof(double));
    if (in.fail())
    {
      val.clear();
      return false;
    }
  }
  else
  {
    std::ifstream ifs;
    ifs.open( camName.c_str(), std::ifstream::in);
    if (!ifs.is_open()) {
      std::cerr << "Error: failed to open file '" << camName << "' for reading" << std::endl;
      return false;
    }
    while (ifs.good())
    {
      double valT;
      ifs >> valT;
      if (!ifs.fail())
        val.push_back(valT);
    }
  }

  //Update the camera from file value
  Mat34 P;
  if (stlplus::extension_part(camName) == "bin")
  {
    P << val[0], val[3], val[6], val[9],
      val[1], val[4], val[7], val[10],
      val[2], val[5], val[8], val[11];
  }
  else
  {
    P << val[0], val[1], val[2], val[3],
      val[4], val[5], val[6], val[7],
      val[8], val[9], val[10], val[11];
  }
  Mat3 K, R;
  Vec3 t;
  KRt_From_P(P, &K, &R, &t);
  cam = Pinhole_Intrinsic(0,0,K);
  // K.transpose() is applied to give [R t] to the constructor instead of P = K [R t]
  pose = geometry::Pose3(K.transpose() * P);
  return true;
}

static bool read_Strecha_Camera(const std::string & camName, Pinhole_Intrinsic & cam, geometry::Pose3 & pose)
{
  std::ifstream ifs;
  ifs.open( camName.c_str(), std::ifstream::in);
  if (!ifs.is_open()) {
    std::cerr << "Error: failed to open file '" << camName << "' for reading" << std::endl;
    return false;
  }
  std::vector<double> val;
  while (ifs.good() && !ifs.eof())
  {
    double valT;
    ifs >> valT;
    if (!ifs.fail())
      val.push_back(valT);
  }

  if (val.size() == 3*3 +3 +3*3 +3 + 3 || val.size() == 26) //Strecha cam
  {
    Mat3 K, R;
    K << val[0], val[1], val[2],
      val[3], val[4], val[5],
      val[6], val[7], val[8];
    R << val[12], val[13], val[14],
      val[15], val[16], val[17],
      val[18], val[19], val[20];

    Vec3 C (val[21], val[22], val[23]);
    // R need to be transposed
    cam = Pinhole_Intrinsic(0,0,K);
    pose = geometry::Pose3(R.transpose(), C);
  }
  else
  {
    return false;
  }
  return true;
}

/**
@brief Reads a set of Pinhole Cameras and its poses from a ground truth dataset.
@param[in] functorPointer, to the function which can handle the trajectory format. Example: &read_openMVG_Camera
@param[in] sGTPath, the directory where the camera files are located.
@param[in] Suffix: use "bin" for openMVG or "png.camera" for Strechas data.
@param[out] vec_filenames: read cameras names
@param[out] map_Rt_gt: Map of poses
@param[out] map_camerasGT Map of PinholeCameras.
**/
bool readGt(
  bool (*fcnReadCamPtr)(const std::string &, Pinhole_Intrinsic &, geometry::Pose3&), //pointer to the function reading a camera
  std::string sGTPath,
  std::string suffix,
  std::vector<std::string> & vec_filenames,
  sfm::Poses & map_Rt_gt,
  sfm::Intrinsics & map_camerasGT)
{
  // IF GT_Folder exists, perform evaluation of the quality of rotation estimates
  if (!stlplus::is_folder(sGTPath)) {
    std::cout << std::endl << "There is not valid GT data to read from " << sGTPath << std::endl;
    return false;
  }
  
  std::cout << std::endl << "Read rotation and translation estimates" << std::endl;
  // Load GT
  std::map< std::string, Mat3 > map_R_gt;
  //Try to read .suffix camera (parse camera names)
  std::vector<std::string> vec_camfilenames =
    stlplus::folder_wildcard(sGTPath, "*."+suffix, false, true);
  std::sort(vec_camfilenames.begin(), vec_camfilenames.end());
  std::vector<std::string>::const_iterator citerBegin = vec_camfilenames.begin();
  if (!vec_camfilenames.empty())
  {
    IndexT id = 0;
    for (std::vector<std::string>::const_iterator iter = vec_camfilenames.begin();
      iter != vec_camfilenames.end(); ++iter, ++id)
    {
      geometry::Pose3 pose;
      std::shared_ptr<Pinhole_Intrinsic> pinholeIntrinsic = std::make_shared<Pinhole_Intrinsic>();
      bool loaded = fcnReadCamPtr(stlplus::create_filespec(sGTPath, *iter), *pinholeIntrinsic.get(), pose);
      if (!loaded)
      {
        std::cout << "Failed to load: " << *iter << std::endl;
        return false;
      }

      vec_filenames.push_back(stlplus::create_filespec(sGTPath, *iter));
      map_Rt_gt[id] = pose;
      map_camerasGT.emplace(id, pinholeIntrinsic);
    }
  }
  return true;
}

} // namespace openMVG

#endif // IO_READ_GT_HPP
