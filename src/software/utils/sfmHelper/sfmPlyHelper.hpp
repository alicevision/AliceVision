// This file is part of the AliceVision project.
// Copyright (c) 2015 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/numeric/numeric.hpp"

#include <aliceVision/vfs/ostream.hpp>
#include <fstream>
#include <string>
#include <vector>

namespace aliceVision{
namespace plyHelper{

/// Export 3D point vector to PLY format
inline bool exportToPly(const std::vector<Vec3> & vec_points,
  const std::string & sFileName)
{
  vfs::ostream outfile;
  outfile.open(sFileName, std::ios_base::out);
  
  if(!outfile.is_open())
    throw std::runtime_error("Unable to create file "+sFileName);

  outfile << "ply"
    << std::endl << "format ascii 1.0"
    << std::endl << "element vertex " << vec_points.size()
    << std::endl << "property float x"
    << std::endl << "property float y"
    << std::endl << "property float z"
    << std::endl << "property uchar red"
    << std::endl << "property uchar green"
    << std::endl << "property uchar blue"
    << std::endl << "end_header" << std::endl;

  for (size_t i=0; i < vec_points.size(); ++i)
  {
    outfile << vec_points[i].transpose()
      << " 255 255 255" << "\n";
  }
  bool bOk = outfile.good();
  outfile.close();
  return bOk;
}

/// Export 3D point vector and camera position to PLY format
inline bool exportToPly(const std::vector<Vec3> & vec_points,
  const std::vector<Vec3> & vec_camPos,
  const std::string & sFileName,
  const std::vector<Vec3> * vec_coloredPoints = NULL)
{
  vfs::ostream outfile;
  outfile.open(sFileName, std::ios_base::out);
  
  if(!outfile.is_open())
    throw std::runtime_error("Unable to create file "+sFileName);

  outfile << "ply"
    << '\n' << "format ascii 1.0"
    << '\n' << "element vertex " << vec_points.size()+vec_camPos.size()
    << '\n' << "property float x"
    << '\n' << "property float y"
    << '\n' << "property float z"
    << '\n' << "property uchar red"
    << '\n' << "property uchar green"
    << '\n' << "property uchar blue"
    << '\n' << "end_header" << std::endl;

  for (size_t i=0; i < vec_points.size(); ++i)  {
    if (vec_coloredPoints == NULL)
      outfile << vec_points[i].transpose()
        << " 255 255 255" << "\n";
    else
      outfile << vec_points[i].transpose()
        << " " << (*vec_coloredPoints)[i].transpose() << "\n";
  }

  for (size_t i=0; i < vec_camPos.size(); ++i)  {
    outfile << vec_camPos[i].transpose()
      << " 0 255 0" << "\n";
  }
  outfile.flush();
  bool bOk = outfile.good();
  outfile.close();
  return bOk;
}

} // namespace plyHelper
} // namespace aliceVision
