// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "svgVisualization.hpp"
#include <aliceVision/config.hpp>
#include <aliceVision/system/Logger.hpp>

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
#include <aliceVision/feature/cctag/ImageDescriber_CCTAG.hpp>
#endif
#include <dependencies/vectorGraphics/svgDrawer.hpp>

namespace aliceVision {
namespace matching {


std::string describerTypeColor(feature::EImageDescriberType descType )
{
  switch(descType)
  {
    case feature::EImageDescriberType::SIFT:           return "yellow";
    case feature::EImageDescriberType::SIFT_FLOAT:     return "yellow";
    case feature::EImageDescriberType::SIFT_UPRIGHT:   return "yellow";
    case feature::EImageDescriberType::DSPSIFT:        return "yellow";

    case feature::EImageDescriberType::AKAZE:          return "purple";
    case feature::EImageDescriberType::AKAZE_LIOP:     return "purple";
    case feature::EImageDescriberType::AKAZE_MLDB:     return "purple";
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)
    case feature::EImageDescriberType::CCTAG3:         return "blue";
    case feature::EImageDescriberType::CCTAG4:         return "blue";
#endif
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_APRILTAG)
    case feature::EImageDescriberType::APRILTAG16H5:   return "blue";
#endif

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OPENCV)
#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_OCVSIFT)
    case feature::EImageDescriberType::SIFT_OCV:       return "orange";
#endif
    case feature::EImageDescriberType::AKAZE_OCV:      return "indigo";
#endif

    case feature::EImageDescriberType::UNKNOWN:        return "red";
    case feature::EImageDescriberType::UNINITIALIZED:  return "fuchsia";
  }
  return "magenta";
}

float getRadiusEstimate(const std::pair<size_t,size_t> & imgSize)
{
  // heuristic for the radius of the feature according to the size of the image
  // the larger the distance the larger the minimum radius should be in order to be visible
  // We consider a minimum of 2 pixel and we increment it linearly according to 
  // the image size
  return std::max(std::max(imgSize.first, imgSize.second) / float(600), 2.0f);
}

float getStrokeEstimate(const std::pair<size_t,size_t> & imgSize)
{
  return std::max(std::max(imgSize.first, imgSize.second) / float(2200), 2.0f);
}

inline void drawMatchesSideBySide(svg::svgDrawer& svgStream,
                           const feature::PointFeature &L,
                           const feature::PointFeature &R,
                           std::size_t offset,
                           float radiusLeft,
                           float radiusRight,
                           float strokeLeft,
                           float strokeRight,
                           const svg::svgStyle& lineStyle,
                           const svg::svgStyle& leftStyle,
                           const svg::svgStyle& rightStyle)
{
    const float xRight = R.x() + offset;
    svgStream.drawLine(L.x(), L.y(), xRight, R.y(), lineStyle);
    svgStream.drawCircle(L.x(), L.y(), radiusLeft, leftStyle);
    svgStream.drawCircle(xRight, R.y(), radiusRight, rightStyle);
}

inline void drawMatchesSideBySide(svg::svgDrawer& svgStream,
                           const std::vector<feature::PointFeature>& keypointsLeft,
                           const std::vector<feature::PointFeature>& keypointsRight,
                           std::size_t offset,
                           float radiusLeft,
                           float radiusRight,
                           float strokeLeft,
                           float strokeRight,
                           const svg::svgStyle& lineStyle,
                           const svg::svgStyle& leftStyle,
                           const svg::svgStyle& rightStyle,
                           const matching::IndMatches& matches)
{
    for (const matching::IndMatch &m : matches)
    {
        const feature::PointFeature &L = keypointsLeft[m._i];
        const feature::PointFeature &R = keypointsRight[m._j];
        drawMatchesSideBySide(svgStream, L, R,  offset, radiusLeft, radiusRight, strokeLeft, strokeRight, lineStyle, leftStyle, rightStyle);
    }
}

inline void drawInliersSideBySide(svg::svgDrawer& svgStream,
                           const std::vector<feature::PointFeature>& keypointsLeft,
                           const std::vector<feature::PointFeature>& keypointsRight,
                           std::size_t offset,
                           float radiusLeft,
                           float radiusRight,
                           float strokeLeft,
                           float strokeRight,
                           const svg::svgStyle& lineStyle,
                           const svg::svgStyle& leftStyle,
                           const svg::svgStyle& rightStyle,
                           const matching::IndMatches& matches,
                           std::vector<size_t>& inliers)
{
    for (const auto &idx : inliers)
    {
        const auto currMatch = matches[idx];
        const feature::PointFeature &L = keypointsLeft[currMatch._i];
        const feature::PointFeature &R = keypointsRight[currMatch._j];

        drawMatchesSideBySide(svgStream, L, R,  offset, radiusLeft, radiusRight, strokeLeft, strokeRight, lineStyle, leftStyle, rightStyle);
    }
}

void drawMatchesSideBySide(const std::string& imagePathLeft,
                           const std::pair<size_t,size_t>& imageSizeLeft,
                           const std::vector<feature::PointFeature>& keypointsLeft,
                           const std::string& imagePathRight,
                           const std::pair<size_t,size_t>& imageSizeRight,
                           const std::vector<feature::PointFeature>& keypointsRight,
                           const matching::IndMatches& matches,
                           const std::string &outputSVGPath)
{
  svg::svgDrawer svgStream(imageSizeLeft.first + imageSizeRight.first,
                           std::max(imageSizeLeft.second, imageSizeRight.second));
  const std::size_t offset = imageSizeLeft.first;
  svgStream.drawImage(imagePathLeft, imageSizeLeft.first, imageSizeLeft.second);
  svgStream.drawImage(imagePathRight, imageSizeRight.first, imageSizeRight.second, offset);

  const float radiusLeft = getRadiusEstimate(imageSizeLeft);
  const float radiusRight = getRadiusEstimate(imageSizeRight);
  const float strokeLeft = getStrokeEstimate(imageSizeLeft);
  const float strokeRight = getStrokeEstimate(imageSizeRight);
  const std::string color = "green";
  const svg::svgStyle lineStyle = svg::svgStyle().stroke(color, std::min(strokeRight, strokeLeft));
  const svg::svgStyle leftStyle = svg::svgStyle().stroke(color, strokeLeft);
  const svg::svgStyle rightStyle = svg::svgStyle().stroke(color, strokeRight);


  for (const matching::IndMatch &m : matches)
  {
    //Get back linked feature, draw a circle and link them by a line
    const feature::PointFeature &L = keypointsLeft[m._i];
    const feature::PointFeature &R = keypointsRight[m._j];
    const float xRight = R.x() + offset;

    svgStream.drawLine(L.x(), L.y(), xRight, R.y(), lineStyle);
    svgStream.drawCircle(L.x(), L.y(), radiusLeft, leftStyle);
    svgStream.drawCircle(xRight, R.y(), radiusRight, rightStyle);
  }


  std::ofstream svgFile(outputSVGPath);
  if (!svgFile.is_open())
  {
    ALICEVISION_CERR("Unable to open file " + outputSVGPath);
    return;
  }
  svgFile << svgStream.closeSvgFile().str();
  if (!svgFile.good())
  {
    ALICEVISION_CERR("Something wrong while writing file " + outputSVGPath);
    return;
  }
  svgFile.close();
}

void drawHomographyMatches(const std::string& imagePathLeft,
                           const std::pair<size_t,size_t>& imageSizeLeft,
                           const std::vector<feature::PointFeature>& features_I,
                           const std::string& imagePathRight,
                           const std::pair<size_t,size_t>& imageSizeRight,
                           const std::vector<feature::PointFeature>& features_J,
                           const std::vector<std::pair<Mat3, matching::IndMatches>>& homographiesAndMatches,
                           const matching::IndMatches& putativeMatches,
                           const std::string& outFilename)
{
  const auto& colors = sixteenColors;

  svg::svgDrawer svgStream(imageSizeLeft.first + imageSizeRight.first,
                           std::max(imageSizeLeft.second, imageSizeRight.second));
  const std::size_t offset = imageSizeLeft.first;

  svgStream.drawImage(imagePathLeft, imageSizeLeft.first, imageSizeLeft.second);
  svgStream.drawImage(imagePathRight, imageSizeRight.first, imageSizeRight.second, offset);

  // draw little white dots representing putative matches
  for (const auto& match : putativeMatches)
  {
    const float radius{1.f};
    const float strokeSize{2.f};
    const feature::PointFeature &fI = features_I.at(match._i);
    const feature::PointFeature &fJ = features_J.at(match._j);
    const svg::svgStyle style = svg::svgStyle().stroke("white", strokeSize);

    svgStream.drawCircle(fI.x(), fI.y(), radius, style);
    svgStream.drawCircle(fJ.x() + offset, fJ.y(), radius, style);
  }

  {
    // for each homography draw the associated matches in a different color
    std::size_t iH{0};
    const float radius{5.f};
    const float strokeSize{5.f};
    for (const auto &currRes : homographiesAndMatches)
    {
      const auto &bestMatchesId = currRes.second;
      // 0 < iH <= 8: colored; iH > 8  are white (not enough colors)
      const std::string color = (iH < colors.size()) ? colors.at(iH) : "grey";

      for (const auto &match : bestMatchesId)
      {
        const feature::PointFeature &fI = features_I.at(match._i);
        const feature::PointFeature &fJ = features_J.at(match._j);

        const svg::svgStyle style = svg::svgStyle().stroke(color, strokeSize);

        svgStream.drawCircle(fI.x(), fI.y(), radius, style);
        svgStream.drawCircle(fJ.x() + offset, fJ.y(), radius, style);
      }
      ++iH;
    }
  }

  std::ofstream svgFile(outFilename);
  if(!svgFile.is_open())
  {
    ALICEVISION_CERR("Unable to open file "+outFilename);
    return;
  }
  svgFile << svgStream.closeSvgFile().str();
  if(!svgFile.good())
  {
    ALICEVISION_CERR("Something wrong happened while writing file "+outFilename);
    return;
  }
  svgFile.close();
}

void saveMatches2SVG(const std::string &imagePathLeft,
                     const std::pair<size_t,size_t> & imageSizeLeft,
                     const feature::MapRegionsPerDesc &keypointsLeft,
                     const std::string &imagePathRight,
                     const std::pair<size_t,size_t> & imageSizeRight,
                     const feature::MapRegionsPerDesc &keypointsRight,
                     const matching::MatchesPerDescType & matches,
                     const std::string &outputSVGPath)
{
  svg::svgDrawer svgStream( imageSizeLeft.first + imageSizeRight.first, std::max(imageSizeLeft.second, imageSizeRight.second));
  svgStream.drawImage(imagePathLeft, imageSizeLeft.first, imageSizeLeft.second);
  svgStream.drawImage(imagePathRight, imageSizeRight.first, imageSizeRight.second, imageSizeLeft.first);
  
  // heuristic for the radius of the feature according to the size of the image
  // the larger the distance the larger the minimum radius should be in order to be visible
  // We consider a minimum of 2 pixel and we increment it linearly according to 
  // the image size
  const float radiusLeft = getRadiusEstimate(imageSizeLeft);
  const float radiusRight = getRadiusEstimate(imageSizeRight);
  const float strokeLeft = getStrokeEstimate(imageSizeLeft);
  const float strokeRight = getStrokeEstimate(imageSizeRight);

  for(const auto& descMatches : matches)
  {
    feature::EImageDescriberType descType = descMatches.first;
    const std::string descColor = describerTypeColor(descType);
    for(const matching::IndMatch &m : descMatches.second)
    {
      //Get back linked feature, draw a circle and link them by a line
      const feature::PointFeature & L = keypointsLeft.at(descType)->GetRegionsPositions()[m._i];
      const feature::PointFeature & R = keypointsRight.at(descType)->GetRegionsPositions()[m._j];

      svgStream.drawLine(L.x(), L.y(), R.x()+imageSizeLeft.first, R.y(), svg::svgStyle().stroke("green", std::min(strokeRight,strokeLeft)));
      svgStream.drawCircle(L.x(), L.y(), radiusLeft, svg::svgStyle().stroke(descColor, strokeLeft));
      svgStream.drawCircle(R.x()+imageSizeLeft.first, R.y(), radiusRight, svg::svgStyle().stroke(descColor, strokeRight));
    }
  }
 
  std::ofstream svgFile( outputSVGPath.c_str() );
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();
}


void saveKeypoints2SVG(const std::string &inputImagePath,
                       const std::pair<size_t,size_t> & imageSize,
                       const std::vector<feature::PointFeature> &keypoints,
                       const std::string &outputSVGPath,
                       bool richKeypoint /*= true*/)
{
  svg::svgDrawer svgStream( imageSize.first, imageSize.second);
  svgStream.drawImage(inputImagePath, imageSize.first, imageSize.second);
  // heuristic for the radius of the feature according to the size of the image
  // the larger the distance the larger the minimum radius should be in order to be visible
  // We consider a minimum of 2 pixel and we increment it linearly according to 
  // the image size
  const float radius = getRadiusEstimate(imageSize);
  const float strokeWidth = getStrokeEstimate(imageSize);
  
  for(const feature::PointFeature &kpt : keypoints)
  {
    svgStream.drawCircle(kpt.x(), kpt.y(), (richKeypoint) ? kpt.scale()*radius : radius, svg::svgStyle().stroke("yellow", strokeWidth));
    if(richKeypoint)
    {
      // compute the coordinate of the point on the circle used to draw the orientation line
      const Vec2f point = kpt.coords() + kpt.getScaledOrientationVector() * radius;
      svgStream.drawLine(kpt.x(), kpt.y(), 
                         point(0), point(1),
                         svg::svgStyle().stroke("yellow", strokeWidth));
    }
  }
 
  std::ofstream svgFile( outputSVGPath );
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();  
}

/**
 * @brief It saves a svg file containing an image (as linked image) and its detected
 * features.
 *
 * @param[in] imagePathLeft The full path to the left image. The image is only
 * saved as a link, no image data is stored in the svg.
 * @param[in] imageSizeLeft The size of the image <width,height>.
 * @param[in] keypointsLeft The keypoints of the left image.
 * @param[in] imagePathRight The full path to the left image. The image is only
 * saved as a link, no image data is stored in the svg.
 * @param[in] imageSizeRight The size of the image <width,height>.
 * @param[in] keypointsRight The keypoints of the right image.
 * @param[in] richKeypoint Draw rich keypoints with a circle proportional to the
 * octave in which the point has been detected.
 */
void drawKeypointsSideBySide(const std::string&imagePathLeft,
                             const std::pair<size_t,size_t>& imageSizeLeft,
                             const std::vector<feature::PointFeature>& keypointsLeft,
                             const std::string &imagePathRight,
                             const std::pair<size_t,size_t>& imageSizeRight,
                             const std::vector<feature::PointFeature>& keypointsRight,
                             const std::string &outputSVGPath,
                             bool richKeypoint)
{
  svg::svgDrawer svgStream( imageSizeLeft.first + imageSizeRight.first, std::max(imageSizeLeft.second, imageSizeRight.second));
  const std::size_t offset = imageSizeLeft.first;
  svgStream.drawImage(imagePathLeft, imageSizeLeft.first, imageSizeLeft.second);
  svgStream.drawImage(imagePathRight, imageSizeRight.first, imageSizeRight.second, offset);

  const float radiusLeft = getRadiusEstimate(imageSizeLeft);
  const float radiusRight = getRadiusEstimate(imageSizeRight);
  const float strokeLeft = getStrokeEstimate(imageSizeLeft);
  const float strokeRight = getStrokeEstimate(imageSizeRight);
  const svg::svgStyle styleLeft = svg::svgStyle().stroke("green", strokeLeft);
  const svg::svgStyle styleRight = svg::svgStyle().stroke("green", strokeRight);

  for(const auto& kpt : keypointsLeft)
  {
    svgStream.drawCircle(kpt.x(), kpt.y(), (richKeypoint) ? kpt.scale()*radiusLeft : radiusLeft, styleLeft);
  }
  for(const auto& kpt : keypointsRight)
  {
    svgStream.drawCircle(kpt.x()+offset, kpt.y(), (richKeypoint) ? kpt.scale()*radiusRight : radiusRight, styleRight);
  }

  std::ofstream svgFile(outputSVGPath);
  if(!svgFile.is_open())
  {
    ALICEVISION_CERR("Unable to open file " + outputSVGPath);
    return;
  }
  svgFile << svgStream.closeSvgFile().str();
  if(!svgFile.good())
  {
    ALICEVISION_CERR("Something wrong while writing file " + outputSVGPath);
    return;
  }
  svgFile.close();
}

void saveFeatures2SVG(const std::string &inputImagePath,
                      const std::pair<size_t,size_t> & imageSize,
                      const feature::MapFeaturesPerDesc & keypoints,
                      const std::string & outputSVGPath)
{
  svg::svgDrawer svgStream( imageSize.first, imageSize.second);
  svgStream.drawImage(inputImagePath, imageSize.first, imageSize.second);
  // heuristic for the radius of the feature according to the size of the image
  // the larger the distance the larger the minimum radius should be in order to be visible
  // We consider a minimum of 2 pixel and we increment it linearly according to 
  // the image size
  const float radius = getRadiusEstimate(imageSize);
  const float strokeWidth = getStrokeEstimate(imageSize);

  for(const auto& keypointPerDesc: keypoints)
  {
    const std::string featColor = describerTypeColor(keypointPerDesc.first);
    for(const feature::PointFeature &kpt : keypointPerDesc.second)
    {
      svgStream.drawCircle(kpt.x(), kpt.y(), radius, svg::svgStyle().stroke(featColor, strokeWidth));
    }
  }
 
  std::ofstream svgFile( outputSVGPath );
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();
}

/**
 * @brief
 * 
 * @param[in] inputImagePath The full path to the image file. The image is only 
 * saved as a link, no image data is stored in the svg.
 * @param[in] imageSize The size of the image <width,height>.
 * @param[in] points A vector containing the points to draw.
 * @param[in] outputSVGPath The name of the svg file to generate.
 */
void saveFeatures2SVG(const std::string &inputImagePath,
                      const std::pair<size_t,size_t> & imageSize,
                      const Mat &points,
                      const std::string &outputSVGPath,
                      const std::vector<size_t> *inliers /*=nullptr*/)
{
  assert(points.rows()>=2);
  svg::svgDrawer svgStream( imageSize.first, imageSize.second);
  svgStream.drawImage(inputImagePath, imageSize.first, imageSize.second);
  // heuristic for the radius of the feature according to the size of the image
  // the larger the distance the larger the minimum radius should be in order to be visible
  // We consider a minimum of 2 pixel and we increment it linearly according to 
  // the image size
  const float radius = getRadiusEstimate(imageSize);
  const float strokeWidth = getStrokeEstimate(imageSize);
  
  if(!inliers)
  {
    for(std::size_t i=0; i < points.cols(); ++i) 
    {
      svgStream.drawCircle(points(0,i), points(1,i), radius, svg::svgStyle().stroke("yellow", strokeWidth));
    }
  }
  else
  {
    for(std::size_t i=0; i < points.cols(); ++i) 
    {
      if(std::find(inliers->begin(), inliers->end(), i) != inliers->end())
        svgStream.drawCircle(points(0,i), points(1,i), radius, svg::svgStyle().stroke("green", strokeWidth));
      else
        svgStream.drawCircle(points(0,i), points(1,i), radius, svg::svgStyle().stroke("yellow", strokeWidth));
    }   
  }
 
  std::ofstream svgFile( outputSVGPath );
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();
}

bool lineToBorderPoints(const Vec3 &epiLine, const std::size_t imgW, const std::size_t imgH, std::vector<Vec2> &intersectionPts)
{
  intersectionPts.clear();
  intersectionPts.reserve(2);
  // @TODO check special case of epiline coincident with the border lines
  
  // intersect epiline with x=0
  //y = -(a*0+c)/b
  double p = - epiLine(2)/epiLine(1);
  if(p >= 0 && p <= imgH)
    intersectionPts.emplace_back(0, p);
  
  // intersect epiline with x=imgW
  //y = -(a*imgW+c)/b
  p = - (imgW*epiLine(0) + epiLine(2))/epiLine(1);
  if(p >= 0 && p <= imgH)
    intersectionPts.emplace_back(imgW, p);
  
  if(intersectionPts.size()==2)
    return true;
  
  // intersect epiline with y=0
  //x = -(b*0+c)/a
  p = - epiLine(2)/epiLine(0);
  if(p >= 0 && p <= imgW)
    intersectionPts.emplace_back(p, 0);
  
  if(intersectionPts.size()==2)
    return true;
  
  // intersect epiline with y=imgH
  //y = -(a*imgW+c)/b
  p = - (imgH*epiLine(1) + epiLine(2))/epiLine(0);
  if(p >= 0 && p <= imgW)
    intersectionPts.emplace_back(p, imgH);
  
  return (intersectionPts.size()==2);
  
}

void saveEpipolarGeometry2SVG(const std::string &imagePath,
                              const std::pair<size_t, size_t> & imageSize,
                              const std::vector<feature::PointFeature> &keypoints,
                              const std::vector<feature::PointFeature> &otherKeypoints,
                              const matching::IndMatches &matches,
                              const Mat3 &Fmat,
                              const std::string &outputSVGPath,
                              bool left)
{
  svg::svgDrawer svgStream(imageSize.first, imageSize.second);
  svgStream.drawImage(imagePath, imageSize.first, imageSize.second);
  std::size_t count = 0;
  // heuristic for the radius of the feature according to the size of the image
  // the larger the distance the larger the minimum radius should be in order to be visible
  // We consider a minimum of 2 pixel and we increment it linearly according to 
  // the image size
  const float radius = getRadiusEstimate(imageSize);
  const float strokeWidth = getStrokeEstimate(imageSize);
  for(const matching::IndMatch &m : matches)
  {
    //Get back linked feature, draw a circle and link them by a line
    feature::PointFeature p;
    feature::PointFeature other;
    if(left)
    {
      p = keypoints[m._i];
      other = otherKeypoints[m._j];
    }
    else
    {
      p = keypoints[m._j];
      other = otherKeypoints[m._i];
    }
    if(count > 7)
      svgStream.drawCircle(p.x(), p.y(), radius, svg::svgStyle().stroke("yellow", strokeWidth));
    else
      svgStream.drawCircle(p.x(), p.y(), radius, svg::svgStyle().stroke("red", strokeWidth).fill("red"));

    Vec3 epiLine;
    if(left)
    {
      epiLine = Fmat.transpose() * Vec3(other.x(), other.y(), 1.0);
    }
    else
    {
      epiLine = Fmat * Vec3(other.x(), other.y(), 1.0);
    }

    //    ALICEVISION_LOG_DEBUG("test 1 o*F*p " << (Fmat*Vec3(p.x(), p.y(), 1.0)).transpose()*Vec3(other.x(), other.y(), 1.0));
    //    ALICEVISION_LOG_DEBUG("test 2 p*F*o " << (Fmat.transpose()*Vec3(p.x(), p.y(), 1.0)).transpose()*Vec3(other.x(), other.y(), 1.0));
    //    ALICEVISION_LOG_DEBUG("epiline\n" << epiLine << " dotprod " << (epiLine.dot(Vec3(p.x(), p.y(), 1.0))));
    std::vector<Vec2> pts;
    if(lineToBorderPoints(epiLine, imageSize.first, imageSize.second, pts))
    {
      //      ALICEVISION_LOG_DEBUG("pt1*epiline " << (epiLine.transpose()*Vec3(pts[0](0), pts[0](1), 1)));
      //      ALICEVISION_LOG_DEBUG("pt1 " << pts[0]);
      //      ALICEVISION_LOG_DEBUG("pt2*epiline " << (epiLine.transpose()*Vec3(pts[1](0), pts[1](1), 1)));
      //      ALICEVISION_LOG_DEBUG("pt2 " << pts[1]);
      if(count > 7)
        svgStream.drawLine(pts[0](0), pts[0](1), pts[1](0), pts[1](1), svg::svgStyle().stroke("green", strokeWidth));
      else
        svgStream.drawLine(pts[0](0), pts[0](1), pts[1](0), pts[1](1), svg::svgStyle().stroke("red", strokeWidth));
    }
    else
    {
      ALICEVISION_LOG_DEBUG("********** pts size: " << pts.size() << " epiline " << epiLine << " out of image");
      if(pts.size() > 0)
      {
        svgStream.drawLine(pts[0](0), pts[0](1), p.x(), p.y(), svg::svgStyle().stroke("red", 10 * strokeWidth));
        ALICEVISION_LOG_DEBUG("********** pts: " << pts[0].transpose());
      }
    }
    ++count;
    //    if(count > 7) break;

  }

  //draw the epipole
  Mat epipole;
  if(left)
    epipole = Fmat.fullPivLu().kernel();
  else
    epipole = Fmat.transpose().fullPivLu().kernel();

  if(epipole.cols() > 1)
  {
    ALICEVISION_LOG_WARNING("F has kernel of size " << epipole.cols() << ":\n" << epipole);
  }
  else
  {
    // normalize coordinates
    Vec3 point = epipole.col(0);
    ALICEVISION_LOG_DEBUG("epipole:\n" << point);
    //@todo check 0
    point /= point(2);
    ALICEVISION_LOG_DEBUG("epipole normalized:\n" << point);
    // check if the point is inside the image
    if(!((point(0) > 0 && point(0) < imageSize.first) &&
            (point(1) > 0 && point(1) < imageSize.second)))
    {
      ALICEVISION_LOG_DEBUG("epipole outside the image:\n" << point);
      // point outside the image, clamp it to the borders
      if(point(0) < 0) point(0) = 0;
      if(point(0) > imageSize.first) point(0) = imageSize.first;
      if(point(1) < 0) point(1) = 0;
      if(point(1) > imageSize.second) point(0) = imageSize.second;
      ALICEVISION_LOG_DEBUG("clamped epipole:\n" << point);
    }
    svgStream.drawCircle(point(0), point(1), 3 * radius, svg::svgStyle().stroke("red", strokeWidth).fill("red"));
  }

  std::ofstream svgFile(outputSVGPath.c_str());
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();
}

void saveMatchesAsMotion(const std::string &imagePath,
                         const std::pair<size_t, size_t> & imageSize,
                         const std::vector<feature::PointFeature> &keypoints,
                         const std::vector<feature::PointFeature> &otherKeypoints,
                         const matching::IndMatches &matches,
                         const std::string &outputSVGPath,
                         bool left,
                         bool richKeypoint)
{
  svg::svgDrawer svgStream(imageSize.first, imageSize.second);
  svgStream.drawImage(imagePath, imageSize.first, imageSize.second);

  const float radius = getRadiusEstimate(imageSize);
  const float strokeWidth = getStrokeEstimate(imageSize);
  for(size_t i = 0; i < matches.size(); ++i)
  {
    //Get back linked feature, draw a circle and link them by a line
    const auto L = keypoints[matches[i]._i];
    const auto R = otherKeypoints[matches[i]._j];
    if(left)
    {
      svgStream.drawLine(L.x(), L.y(), R.x(), R.y(), svg::svgStyle().stroke("green", strokeWidth));
      svgStream.drawCircle(L.x(), L.y(), (richKeypoint) ? L.scale()*radius : radius, svg::svgStyle().stroke("yellow", 2.0));
      svgStream.drawCircle(R.x(), R.y(), (richKeypoint) ? R.scale()*radius : radius, svg::svgStyle().stroke("red", strokeWidth));
    }
    else
    {
      svgStream.drawLine(L.x(), L.y(), R.x(), R.y(), svg::svgStyle().stroke("green", strokeWidth));
      svgStream.drawCircle(R.x(), R.y(), (richKeypoint) ? R.scale()*radius : radius, svg::svgStyle().stroke("yellow", 2.0));
      svgStream.drawCircle(L.x(), L.y(), (richKeypoint) ? L.scale()*radius : radius, svg::svgStyle().stroke("red", strokeWidth));

    }
  }
  std::ofstream svgFile(outputSVGPath.c_str());
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();
}

void saveMatchesAsMotion(const std::string &imagePath,
                         const std::pair<size_t, size_t> & imageSize,
                         const std::vector<feature::PointFeature> &keypoints,
                         const std::vector<feature::PointFeature> &otherKeypoints,
                         const std::vector<std::pair<Mat3, matching::IndMatches>>& homographiesAndMatches,
                         const std::string &outputSVGPath,
                         bool left,
                         bool richKeypoint)
{
  svg::svgDrawer svgStream(imageSize.first, imageSize.second);
  svgStream.drawImage(imagePath, imageSize.first, imageSize.second);

  const float radius = getRadiusEstimate(imageSize);
  const float strokeWidth = getStrokeEstimate(imageSize);

  std::size_t count{0};
  const std::size_t numMaxColor{sixteenColors.size()};

  for(const auto& h : homographiesAndMatches)
  {
    const auto& matches = h.second;
    const auto strokeColor = sixteenColors[count++ % numMaxColor];
    const svg::svgStyle lineStyle = svg::svgStyle().stroke(strokeColor, strokeWidth);
    const svg::svgStyle otherStyle = svg::svgStyle().stroke(strokeColor, 2.f);

    for (const auto& match : matches)
    {
      //Get back linked feature, draw a circle and link them by a line
      const auto L = keypoints[match._i];
      const auto R = otherKeypoints[match._j];
      if(left)
      {
        svgStream.drawLine(L.x(), L.y(), R.x(), R.y(), lineStyle);
        svgStream.drawCircle(L.x(), L.y(), (richKeypoint) ? L.scale()*radius : radius, otherStyle);
        svgStream.drawCircle(R.x(), R.y(), (richKeypoint) ? R.scale()*radius : radius, lineStyle);
      }
      else
      {
        svgStream.drawLine(L.x(), L.y(), R.x(), R.y(), lineStyle);
        svgStream.drawCircle(R.x(), R.y(), (richKeypoint) ? R.scale()*radius : radius, otherStyle);
        svgStream.drawCircle(L.x(), L.y(), (richKeypoint) ? L.scale()*radius : radius, lineStyle);

      }
    }
  }
  std::ofstream svgFile(outputSVGPath);
  if(!svgFile.is_open())
  {
    ALICEVISION_CERR("Unable to open file " + outputSVGPath);
    return;
  }
  svgFile << svgStream.closeSvgFile().str();
  if(!svgFile.good())
  {
    ALICEVISION_CERR("Something wrong while writing file " + outputSVGPath);
    return;
  }
  svgFile.close();
}

#if ALICEVISION_IS_DEFINED(ALICEVISION_HAVE_CCTAG)

void saveCCTag2SVG(const std::string &inputImagePath,
                      const std::pair<size_t,size_t> & imageSize,
                      const feature::CCTAG_Regions &cctags,
                      const std::string &outputSVGPath)
{
  // set the text size to 5% if the image heigth
  const float textSize = 0.05*imageSize.second;
  
  svg::svgDrawer svgStream( imageSize.first, imageSize.second);
  svgStream.drawImage(inputImagePath, imageSize.first, imageSize.second);
  
  const float radius = getRadiusEstimate(imageSize);
  const float strokeWidth = getStrokeEstimate(imageSize);
    
  const auto &feat = cctags.Features();
  const auto &desc = cctags.Descriptors();
  
  for(std::size_t i = 0; i < desc.size(); ++i) 
  {
    const IndexT cctagId = feature::getCCTagId(desc[i]);
    if ( cctagId == UndefinedIndexT)
    {
      continue;
    }
    const auto &kpt = feat[i];
    svgStream.drawCircle(kpt.x(), kpt.y(), radius, svg::svgStyle().stroke("yellow", strokeWidth));
    svgStream.drawText(kpt.x(), kpt.y(), textSize, std::to_string(cctagId), "yellow");
  }
 
  std::ofstream svgFile( outputSVGPath );
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();
}

void saveCCTagMatches2SVG(const std::string &imagePathLeft,
                     const std::pair<size_t,size_t> & imageSizeLeft,
                     const feature::CCTAG_Regions &cctagLeft,
                     const std::string &imagePathRight,
                     const std::pair<size_t,size_t> & imageSizeRight,
                     const feature::CCTAG_Regions &cctagRight,
                     const matching::IndMatches &matches,
                     const std::string &outputSVGPath, 
                     bool showNotMatched)
{
  // set the text size to 5% if the image heigth
  const float textSize = 0.05*std::min(imageSizeRight.second, imageSizeLeft.second);
  
  svg::svgDrawer svgStream(imageSizeLeft.first + imageSizeRight.first, std::max(imageSizeLeft.second, imageSizeRight.second));
  svgStream.drawImage(imagePathLeft, imageSizeLeft.first, imageSizeLeft.second);
  svgStream.drawImage(imagePathRight, imageSizeRight.first, imageSizeRight.second, imageSizeLeft.first);

  const auto &keypointsLeft = cctagLeft.Features();
  const auto &keypointsRight = cctagRight.Features();
  const auto &descLeft = cctagLeft.Descriptors();
  const auto &descRight = cctagRight.Descriptors();
  
  //just to be sure...
  assert(keypointsLeft.size() == descLeft.size());
  assert(keypointsRight.size() == descRight.size());
  
  const float radiusLeft = getRadiusEstimate(imageSizeLeft);
  const float radiusRight = getRadiusEstimate(imageSizeRight);
  const float strokeLeft = getStrokeEstimate(imageSizeLeft);
  const float strokeRight = getStrokeEstimate(imageSizeRight);
  
  for(const matching::IndMatch &m : matches)
  {
    //Get back linked feature, draw a circle and link them by a line
    const feature::PointFeature & L = keypointsLeft[m._i];
    const feature::PointFeature & R = keypointsRight[m._j];
    const IndexT cctagIdLeft = feature::getCCTagId(descLeft[m._i]);
    const IndexT cctagIdRight = feature::getCCTagId(descRight[m._j]);
    if ( cctagIdLeft == UndefinedIndexT || cctagIdRight == UndefinedIndexT )
    {
      ALICEVISION_LOG_WARNING("[svg]\tWarning! cctagIdLeft " << cctagIdLeft << " " << "cctagIdRight " << cctagIdRight);
      continue;
    }
    
    svgStream.drawLine(L.x(), L.y(), R.x() + imageSizeLeft.first, R.y(), svg::svgStyle().stroke("green", std::min(strokeLeft, strokeRight)));
    svgStream.drawCircle(L.x(), L.y(), radiusLeft, svg::svgStyle().stroke("yellow", strokeLeft));
    svgStream.drawCircle(R.x() + imageSizeLeft.first, R.y(), radiusRight, svg::svgStyle().stroke("yellow", std::min(strokeLeft, strokeRight)));
    
    svgStream.drawText(L.x(), L.y(), textSize, std::to_string(cctagIdLeft), "yellow");
    svgStream.drawText(R.x() + imageSizeLeft.first, R.y(), textSize, std::to_string(cctagIdRight), "yellow");
  }
  
  if(showNotMatched)
  {
    const float textSizeSmaller = 0.75*textSize;
    
    // for the left side
    for(std::size_t i = 0; i < keypointsLeft.size(); ++i)
    {
      // look if the index is not already in matches
      bool found = false;
      for(const matching::IndMatch &m : matches)
      {
        found = m._i == i;
        if(found)
          break;
      }
      if(found)
        continue;
      
      assert(i < descLeft.size());
      // find the cctag id
      const IndexT cctagIdLeft = feature::getCCTagId(descLeft[i]);
      if(cctagIdLeft == UndefinedIndexT)
        continue;
      
      // draw the center
      const feature::PointFeature & L = keypointsLeft[i];
      svgStream.drawCircle(L.x(), L.y(), radiusLeft, svg::svgStyle().stroke("red", strokeLeft));
      // print the id
      svgStream.drawText(L.x(), L.y(), textSizeSmaller, std::to_string(cctagIdLeft), "red");
    }
    
    // for the right side
    for(std::size_t i = 0; i < keypointsRight.size(); ++i)
    {
      // look if the index is not already in matches
      bool found = false;
      for(const matching::IndMatch &m : matches)
      {
        found = m._j == i;
        if(found)
          break;
      }
      if(found)
        continue;
      
      assert(i < descRight.size());
      // find the cctag id
      const IndexT cctagIdRight = feature::getCCTagId(descRight[i]);
      if(cctagIdRight == UndefinedIndexT)
        continue;
      
      // draw the center
      const feature::PointFeature & R = keypointsRight[i];
      svgStream.drawCircle(R.x() + imageSizeLeft.first, R.y(), radiusRight, svg::svgStyle().stroke("red", strokeRight));
      // print the id
      svgStream.drawText(R.x() + imageSizeLeft.first, R.y(), textSizeSmaller, std::to_string(cctagIdRight), "red");

    }
  }

  std::ofstream svgFile(outputSVGPath.c_str());
  svgFile << svgStream.closeSvgFile().str();
  svgFile.close();
}
#endif

} // namespace feature
} // namespace aliceVision

