// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef OPENMVG_COLOR_HARMONIZATION_ENGINE_GLOBAL_H
#define OPENMVG_COLOR_HARMONIZATION_ENGINE_GLOBAL_H

#include <aliceVision/numeric/numeric.h>
#include <aliceVision/features/features.hpp>
#include <aliceVision/features/RegionsPerView.hpp>
#include <aliceVision/tracks/tracks.hpp>

#include <memory>

namespace aliceVision {

enum EHistogramSelectionMethod
{
    eHistogramHarmonizeFullFrame     = 0,
    eHistogramHarmonizeMatchedPoints = 1,
    eHistogramHarmonizeVLDSegment    = 2,
};

/**
 * @brief The ColorHarmonizationEngineGlobal class
 *
 * [1] "Global Multiple-View Color Consistency."
 * Authors: Pierre MOULON, Bruno DUISIT and Pascal MONASSE
 * Date: November 2013
 * Conference: CVMP
 */
class ColorHarmonizationEngineGlobal
{
public:
  ColorHarmonizationEngineGlobal(
    const std::string & sSfM_Data_Filename,
    const std::string & sMatchesPath,
    const std::string & sMatchesGeometricModel,
    const std::string & sOutDirectory,
    const std::vector<features::EImageDescriberType>& descTypes,
    int selectionMethod = -1,
    int imgRef = -1);

  ~ColorHarmonizationEngineGlobal();

  virtual bool Process();

private:

  bool CleanGraph();

  /// Read input data (point correspondences)
  bool ReadInputData();

public:

  const std::vector< std::string > & getFilenamesVector() const { return _vec_fileNames; }

  const std::vector< std::pair< size_t, size_t > > & getImagesSize() const { return _vec_imageSize; }

private:

  EHistogramSelectionMethod _selectionMethod;
  int _imgRef;
  std::string _sMatchesGeometricModel;

  // -----
  // Input data
  // ----

  std::vector< std::string > _vec_fileNames; // considered images

  features::RegionsPerView _regionsPerView;
  std::vector< std::pair< size_t, size_t > > _vec_imageSize; // Size of each image

  aliceVision::matching::PairwiseMatches _pairwiseMatches; // pairwise geometric matches

  std::vector<features::EImageDescriberType> _descTypes; //< describer type use for color harmonizations

  //
  std::string _sSfM_Data_Path;// Path to the Sfm_Scene
  std::string _sMatchesPath;  // Path to correspondences and features
  std::string _sOutDirectory; // Output path where outputs will be stored
};


} // namespace aliceVision

#endif // OPENMVG_COLOR_HARMONIZATION_ENGINE_GLOBAL_H
