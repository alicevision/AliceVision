#include "bestImages.hpp"
#include "openMVG/logger.hpp"

#include <limits>
#include <numeric>
#include <iostream>
#include <assert.h>

namespace openMVG{
namespace calibration{

void precomputeCellIndexes(const std::vector<std::vector<cv::Point2f> >& imagePoints,
                           const cv::Size& imageSize,
                           const std::size_t calibGridSize,
                           std::vector<std::vector<std::size_t> >& cellIndexesPerImage)
{
  float cellWidth = float(imageSize.width) / float(calibGridSize);
  float cellHeight = float(imageSize.height) / float(calibGridSize);

  for (const auto& pointbuf : imagePoints)
  {
    std::vector<std::size_t> imageCellIndexes;
    // Points repartition in image
    for (cv::Point2f point : pointbuf)
    {
      // Compute the index of the point
      std::size_t cellPointX = std::floor(point.x / cellWidth);
      std::size_t cellPointY = std::floor(point.y / cellHeight);
      std::size_t cellIndex = cellPointY * calibGridSize + cellPointX;
      imageCellIndexes.push_back(cellIndex);
    }
    cellIndexesPerImage.push_back(imageCellIndexes);
  }
}

void computeCellsWeight(const std::vector<std::size_t>& imagesIndexes,
                        const std::vector<std::vector<std::size_t> >& cellIndexesPerImage,
                        const std::size_t calibGridSize,
                        std::map<std::size_t, std::size_t>& cellsWeight)
{
  //Init cell's weight to 0
  for (std::size_t i = 0; i < calibGridSize * calibGridSize; ++i)
    cellsWeight[i] = 0;

  // Add weight into cells
  for (std::size_t i = 0; i < imagesIndexes.size(); ++i)
  {
    std::vector<std::size_t> uniqueCellIndexes = cellIndexesPerImage[imagesIndexes[i]];
    std::sort(uniqueCellIndexes.begin(), uniqueCellIndexes.end());
    auto last = std::unique(uniqueCellIndexes.begin(), uniqueCellIndexes.end());
    uniqueCellIndexes.erase(last, uniqueCellIndexes.end());

    for (std::size_t cellIndex : uniqueCellIndexes)
    {
      ++cellsWeight[cellIndex];
    }
  }
}

void computeImageScores(const std::vector<std::size_t>& inputImagesIndexes,
                        const std::vector<std::vector<std::size_t> >& cellIndexesPerImage,
                        const std::map<std::size_t, std::size_t>& cellsWeight,
                        std::vector<std::pair<float, std::size_t> >& imageScores)
{
  // Compute the score of each image
  for (std::size_t i = 0; i < inputImagesIndexes.size(); ++i)
  {
    const std::vector<std::size_t>& imageCellIndexes = cellIndexesPerImage[inputImagesIndexes[i]];
    float imageScore = 0;
    for (std::size_t cellIndex : imageCellIndexes)
    {
      imageScore += cellsWeight.at(cellIndex);
    }
    // Normalize by the number of checker items.
    // If the detector support occlusions of the checker the number of items may vary.
    imageScore /= float(imageCellIndexes.size());
    imageScores.emplace_back(imageScore, inputImagesIndexes[i]);
  }
}

void selectBestImages(const std::vector<std::vector<cv::Point2f> >& imagePoints,
                      const cv::Size& imageSize,
                      const std::size_t& maxCalibFrames,
                      const std::size_t calibGridSize,
                      std::vector<float>& calibImageScore,
                      std::vector<std::size_t>& calibInputFrames,
                      std::vector<std::vector<cv::Point2f> >& calibImagePoints,
                      std::vector<std::size_t>& remainingImagesIndexes)
{
  std::vector<std::vector<std::size_t> > cellIndexesPerImage;

  // Precompute cell indexes per image
  precomputeCellIndexes(imagePoints, imageSize, calibGridSize, cellIndexesPerImage);

  // Init with 0, 1, 2, ...
  remainingImagesIndexes.resize(imagePoints.size());
  std::iota(remainingImagesIndexes.begin(), remainingImagesIndexes.end(), 0);

  std::vector<std::size_t> bestImagesIndexes;
  if (maxCalibFrames < imagePoints.size())
  {
    while (bestImagesIndexes.size() < maxCalibFrames )
    {
      std::map<std::size_t, std::size_t> cellsWeight;
      std::vector<std::pair<float, std::size_t> > imageScores;
      // Count points in each cell of the grid
      if (bestImagesIndexes.empty())
        computeCellsWeight(remainingImagesIndexes, cellIndexesPerImage, calibGridSize, cellsWeight);
      else
        computeCellsWeight(bestImagesIndexes, cellIndexesPerImage, calibGridSize, cellsWeight);

      computeImageScores(remainingImagesIndexes, cellIndexesPerImage, cellsWeight, imageScores);

      // Find best score
      std::size_t bestImageIndex = std::numeric_limits<std::size_t>::max();
      float bestScore = std::numeric_limits<float>::max();
      for (const auto& imageScore: imageScores)
      {
        if (imageScore.first < bestScore)
        {
          bestScore = imageScore.first;
          bestImageIndex = imageScore.second;
        }
      }
      auto eraseIt = std::find(remainingImagesIndexes.begin(), remainingImagesIndexes.end(), bestImageIndex);
      assert(bestScore != std::numeric_limits<float>::max());
      assert(eraseIt != remainingImagesIndexes.end());
      remainingImagesIndexes.erase(eraseIt);
      bestImagesIndexes.push_back(bestImageIndex);
      calibImageScore.push_back(bestScore);
    }
  }
  else
  {
    OPENMVG_LOG_DEBUG("Info: Less valid frames (" << imagePoints.size() << ") than specified maxCalibFrames (" << maxCalibFrames << ").");
    bestImagesIndexes.resize(imagePoints.size());
    std::iota(bestImagesIndexes.begin(), bestImagesIndexes.end(), 0);
    
    std::map<std::size_t, std::size_t> cellsWeight;
    computeCellsWeight(remainingImagesIndexes, cellIndexesPerImage, calibGridSize, cellsWeight);
    
    std::vector<std::pair<float, std::size_t> > imageScores;
    computeImageScores(remainingImagesIndexes, cellIndexesPerImage, cellsWeight, imageScores);
   
    for(auto imgScore: imageScores)
    {
      calibImageScore.push_back(imgScore.first);
    }
  }

  assert(bestImagesIndexes.size() == std::min(maxCalibFrames, imagePoints.size()));

  for(std::size_t i = 0; i < bestImagesIndexes.size(); ++i)
  {
    const std::size_t origI = bestImagesIndexes[i];
    calibImagePoints.push_back(imagePoints[origI]);
    calibInputFrames.push_back(origI);
  }
}

}//namespace calibration
}//namespace openMVG
