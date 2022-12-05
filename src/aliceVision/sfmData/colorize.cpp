// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "colorize.hpp"
#include <aliceVision/alicevision_omp.hpp>
#include <aliceVision/sfmData/SfMData.hpp>
#include <aliceVision/stl/indexedSort.hpp>
#include <aliceVision/stl/mapUtils.hpp>
#include <aliceVision/image/io.hpp>
#include <aliceVision/system/ProgressDisplay.hpp>

#include <map>
#include <random>
#include <vector>
#include <functional>
namespace aliceVision {
namespace sfmData {

void colorizeTracks(SfMData& sfmData)
{
  auto progressDisplay = system::createConsoleProgressDisplay(sfmData.getLandmarks().size(), std::cout,
                                                              "\nCompute scene structure color\n");

  std::vector<std::reference_wrapper<Landmark>> remainingLandmarksToColor;
  remainingLandmarksToColor.reserve(sfmData.getLandmarks().size());

  for(auto& landmarkPair : sfmData.getLandmarks())
    remainingLandmarksToColor.push_back(landmarkPair.second);

  

  struct ObservationInfo
  {
      ObservationInfo(Observation observation, image::RGBfColor& rgb, double& segment)
          : observation(observation)
          , rgb(rgb)
          , segment(segment)
      {
      }
      std::reference_wrapper<Observation> observation;
      image::RGBfColor rgb;
      double segment; // Inverse norm distance of landmark to center of view
  };
  
  std::map<int, std::vector<ObservationInfo>> landmarkInfo;

  for(const auto& viewPair : sfmData.getViews())
  {
      const IndexT viewId = viewPair.first;
      image::Image<image::RGBColor> image;
      image::readImage(viewPair.second->getImagePath(), image, image::EImageColorSpace::SRGB);

      for(int i = 0; i < remainingLandmarksToColor.size(); ++i)
      {
          Landmark& landmark = remainingLandmarksToColor[i];
          auto it = landmark.observations.find(viewId);
          if(it != landmark.observations.end())
          {
              const Vec3& Tcenter = sfmData.getAbsolutePose(viewId).getTransform().center();
              const Vec3& pt = landmark.X;

              double eucd = 1.0 / (Tcenter - pt).norm();
              Vec2 uv = it->second.x;
              uv.x() = clamp(uv.x(), 0.0, static_cast<double>(image.Width() - 1));
              uv.y() = clamp(uv.y(), 0.0, static_cast<double>(image.Height() - 1));
              image::RGBColor obsColor = image(uv.y(), uv.x());
              landmarkInfo[i].push_back(ObservationInfo(it->second,
                                                        image::RGBfColor(static_cast<float>(obsColor.r()),
                                                                         static_cast<float>(obsColor.g()),
                                                                         static_cast<float>(obsColor.b())),
                                                        eucd)
                                        );

          }
          
      }
          
  }

  for(auto& landmarkInfoPair : landmarkInfo)
  {
      Landmark& landmark = remainingLandmarksToColor[landmarkInfoPair.first];
      std::vector<ObservationInfo> obsInfoVector = landmarkInfoPair.second;
      double sumSegments = 0.0;
      // Compute weight based on Observation distance and assign landmark colors
      for(const ObservationInfo& obsInfo : obsInfoVector)
      {
          sumSegments += obsInfo.segment;
      }
      image::RGBfColor rgbFinal(0.0);
      for(const ObservationInfo& obsinfo : obsInfoVector)
      {
          rgbFinal = rgbFinal + (obsinfo.rgb * (obsinfo.segment / sumSegments));

      }
      landmark.rgb = image::RGBColor(static_cast<char>(rgbFinal.r()), static_cast<char>(rgbFinal.g()), static_cast<char>(rgbFinal.b()));
      progressBar += 1;
  }
}

} // namespace sfm
} // namespace aliceVision
