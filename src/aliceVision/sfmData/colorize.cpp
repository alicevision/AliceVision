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
#include <aliceVision/system/Timer.hpp>

#include <map>
#include <random>
#include <vector>
#include <functional>
namespace aliceVision {
namespace sfmData {

class LMColorAccumulator
{
    public:
        LMColorAccumulator() = default;
        ~LMColorAccumulator() = default;


        image::RGBfColor rgbFinal{0};
        double sumSegments{.0};

        void addRGB(const image::RGBfColor& rgbf, double weight) 
        { 
            rgbFinal = rgbFinal + (rgbf*weight);
            sumSegments += weight; 
        }

        image::RGBfColor getColor() const 
        { 
            return rgbFinal / sumSegments;
        }

        image::RGBColor getRGBChar() const 
        {
            auto temp = getColor();
            return image::RGBColor(static_cast<char>(temp.r()), static_cast<char>(temp.g()),static_cast<char>(temp.b()));
        }

};

void colorizeTracks(SfMData& sfmData)
{
  system::Timer timer;
  auto progressDisplay = system::createConsoleProgressDisplay(sfmData.getViews().size(), std::cout,
                                                              "\nCompute scene structure color\n");

  std::vector<std::reference_wrapper<Landmark>> remainingLandmarksToColor;
  remainingLandmarksToColor.reserve(sfmData.getLandmarks().size());

  for(auto& landmarkPair : sfmData.getLandmarks())
    remainingLandmarksToColor.push_back(landmarkPair.second);

  std::vector<LMColorAccumulator> landmarkInfo(sfmData.getLandmarks().size());
  Views& views = sfmData.getViews();

  #pragma omp parallel for
  for( int viewId = 0; viewId < views.size(); ++viewId)
  {
      image::Image<image::RGBColor> image;
      image::readImage(views.at(viewId)->getImagePath(), image, image::EImageColorSpace::SRGB);

      #pragma omp parallel for
      for(int i = 0; i < remainingLandmarksToColor.size(); ++i)
      {
            const Landmark& landmark = remainingLandmarksToColor.at(i);
            const auto it = landmark.observations.find(viewId);

            if(it != landmark.observations.end())
            {
                const Vec3& Tcenter = sfmData.getAbsolutePose(viewId).getTransform().center();
                const Vec3& pt = landmark.X;
                const double eucd = 1.0 / (Tcenter - pt).norm();
                Vec2 uv = it->second.x;
                uv.x() = clamp(uv.x(), 0.0, static_cast<double>(image.Width() - 1));
                uv.y() = clamp(uv.y(), 0.0, static_cast<double>(image.Height() - 1));
                const image::RGBColor obsColor = image(uv.y(), uv.x());
                image::RGBfColor& rgbf = image::RGBfColor(static_cast<float>(obsColor.r()), static_cast<float>(obsColor.g()), static_cast<float>(obsColor.b()));
                
                #pragma omp critical
                {
                    landmarkInfo.at(i).addRGB(rgbf, eucd);
                }   
            }     
      }
      ++progressDisplay;    
  }

  for(int i = 0; i < remainingLandmarksToColor.size(); ++i)
  {
      Landmark& landmark = remainingLandmarksToColor.at(i);
      landmark.rgb = landmarkInfo.at(i).getRGBChar();
      
  }
  
  std::cout << sfmData.getViews().size() << " views and " << remainingLandmarksToColor.size() << " points Colorize Time: " << timer << std::endl;
}

} // namespace sfm
} // namespace aliceVision
