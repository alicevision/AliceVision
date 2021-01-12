// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "dependencies/vectorGraphics/svgDrawer.hpp"
#include <aliceVision/numeric/numeric.hpp>
#include "LineKernel.hpp"

#include <iostream>
#include <fstream>  
#include <vector>
#include <random>
#include <algorithm>


/**
 * @brief Generate a svg file with the ground truth line, the estimated one, the
 * estimated inliers and outliers.
 * 
 * @param[in] outfile The name of the svg file to generate.
 * @param[in] W The width of the image to generate.
 * @param[in] H The height of the image to generate.
 * @param[in] lineGT The ground truth line.
 * @param[in] lineEst The estimated line.
 * @param[in] points The points from which the lines are generated.
 * @param[in] vec_inliers The inliers that fit the estimated line.
 */
void drawTest(const std::string &outfile,
              int imageWidth, 
              int imageHeight,
              const aliceVision::Vec2 &lineGT,
              const aliceVision::Vec2 &lineEst,
              const aliceVision::Mat &points,
              const std::vector<std::size_t> &vec_inliers)
{
  const std::size_t nbPoints = points.cols();
  svg::svgDrawer svgTest(imageWidth, imageHeight);
  for(std::size_t i = 0; i < nbPoints; ++i)
  {
    std::string sCol = "red";
    float x = points.col(i)[0];
    float y = points.col(i)[1];
    if(std::find(vec_inliers.begin(), vec_inliers.end(), i) != vec_inliers.end())
    {
      sCol = "green";
    }
    svgTest.drawCircle(x, y, 1, svg::svgStyle().fill(sCol).noStroke());
  }
  //draw the found line
  float xa = 0, xb = imageWidth;
  float ya = lineEst[1] * xa + lineEst[0];
  float yb = lineEst[1] * xb + lineEst[0];
  svgTest.drawLine(xa, ya, xb, yb, svg::svgStyle().stroke("blue", 0.5));
  //draw the GT line
  ya = lineGT[1] * xa + lineGT[0];
  yb = lineGT[1] * xb + lineGT[0];
  svgTest.drawLine(xa, ya, xb, yb, svg::svgStyle().stroke("black", 0.5));

  //  ostringstream osSvg;
  //  osSvg << gaussianNoiseLevel << "_line_" << sqrt(errorMax) << ".svg";
  std::ofstream svgFile(outfile);
  svgFile << svgTest.closeSvgFile().str();
  svgFile.close();
}


/**
 * @brief Generate a set of point fitting a line with the given amount of noise
 * and outliers.
 * @param[in] numPoints Number of points to generate.
 * @param[in] outlierRatio Fraction of outliers to generate.
 * @param[in] gaussianNoiseLevel Amount of noise to add to the (inlier) points.
 * @param[in] GTModel The parameters (a, b) of the line, as in y = ax + b.
 * @param[in] gen The random generator.
 * @param[out] xy The 2 x numPoints matrix containing the generated points.
 * @param[out] vec_inliersGT The indices of the inliers.
 */
void generateLine(std::size_t numPoints,
                    double outlierRatio,
                    double gaussianNoiseLevel,
                    const aliceVision::Vec2& GTModel,
                    std::mt19937& gen,
                    aliceVision::Mat2X& outPoints,
                    std::vector<std::size_t>& outInliers)
{
  assert(outlierRatio >= 0 && outlierRatio < 1);
  assert(gaussianNoiseLevel >= 0);
  assert(numPoints == outPoints.cols());
  
  std::normal_distribution<> d(0, gaussianNoiseLevel);
  std::uniform_real_distribution<double> realDist(0, 1.0);
  const bool withNoise = (gaussianNoiseLevel > std::numeric_limits<double>::epsilon());

  //-- Build the point list according the given model adding some noise
  for(std::size_t i = 0; i < numPoints; ++i)
  {
    outPoints.col(i) << i, (double) i * GTModel[1] + GTModel[0];
    if(withNoise)
    {
      const double theta = realDist(gen)*2 * M_PI;
//      std::cout << theta << std::endl;
      const double radius = d(gen);
//      std::cout << radius << std::endl;
      outPoints.col(i) += radius * aliceVision::Vec2(std::cos(theta), std::sin(theta));
    }
  }
  const int W = std::abs(outPoints(0, 0) - outPoints(0, numPoints - 1));
  const int H = (int) std::fabs(outPoints(1, 0) - outPoints(1, numPoints - 1));

  //-- Add some outliers (for the asked percentage amount)
  const std::size_t nbPtToNoise = (std::size_t) numPoints * outlierRatio;
  outInliers.resize(numPoints);
  std::iota(outInliers.begin(), outInliers.end(), 0);

  std::vector<std::size_t> vec_outliers(nbPtToNoise); 
  std::iota(vec_outliers.begin(), vec_outliers.end(), 0);
//  cout << "xy\n" << xy << std::endl;
//  cout << "idx\n";
//  std::copy(vec_outliers.begin(), vec_outliers.end(), std::ostream_iterator<int>(std::cout, " "));
  for(std::size_t i = 0; i < vec_outliers.size(); ++i)
  {
    const std::size_t randomIndex = vec_outliers[i];

    outInliers.erase(std::remove(outInliers.begin(), outInliers.end(), randomIndex), outInliers.end());

    aliceVision::Vec2 pt;
    double distance = 0;
    // try to generate a point that is well far from the line
    std::size_t timeToStop = 0;
    const double minDistance = (withNoise) ? 15 * gaussianNoiseLevel : 15;
    while(distance < minDistance)
    {
      assert(timeToStop < 200);
      pt(0) = realDist(gen) * W;
      pt(1) = realDist(gen) * H;
      distance = aliceVision::robustEstimation::pointToLineError(GTModel, pt);
      ++timeToStop;
    }
//    total += timeToStop;

    outPoints.col(randomIndex) = pt;
  }
//  std::cout << "\nTotal attempts: " << total << std::endl;
  assert(numPoints - nbPtToNoise == outInliers.size());

}

