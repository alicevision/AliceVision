// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#ifdef _MSC_VER
#pragma warning(once:4244)
#endif

#include <aliceVision/image/all.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/MathTrait.hpp>
#include <aliceVision/feature/PointFeature.hpp>
#include <aliceVision/feature/Descriptor.hpp>

namespace aliceVision {
namespace feature {

/**
 * Bibliography:
 * [1] "Fast Explicit Diffusion for Accelerated Features in Nonlinear Scale Spaces."
 * Conference : BMVC.
 *
 * @authors Pablo F. Alcantarilla, Jesús Nuevo and Adrien Bartoli
 *
 * @date September 2013
 *
 * @note This implementation is done for AliceVision:
 *  - code look the same but less memory are allocated
 *  - only Perona and Malik G2 diffusion (PM_G2) is implemented.
 *
 * @see Reference implementation (OpenCV based)
 *      by Pablo F. Alcantarilla (1), Jesus Nuevo (2)
 *  - https://github.com/pablofdezalc/akaze
 *  Toshiba Research Europe Ltd (1)
 *  TrueVision Solutions (2)
 */

struct AKAZEOptions
{
  /// octave to process
  int nbOctaves = 4;
  /// levels per octave
  int nbSlicePerOctave = 4;
  /// initial sigma offset (used to suppress low level noise)
  float sigma0 = 1.6f;
  /// hessian determinant threshold
  float threshold = 0.0008f;
  /// magnifier used to describe an interest point
  float descFactor = 1.0f;
  /// grid size for filtering
  std::size_t gridSize = 4;
  /// maximum number of keypoints
  std::size_t maxTotalKeypoints = 1000;
};

struct AKAZEKeypoint
{
  /// coordinate x of the keypoints
  float x = 0.f;
  /// coordinate y of the keypoints
  float y = 0.f;
  /// diameter of the meaningful keypoint neighborhood
  float size = 0.f;
  /// computed orientation of the keypoint (-1 if not applicable);
  /// it's in [0,360) degrees and measured relative to
  /// image coordinate system, ie in clockwise.
  float angle = 0.f;
  /// the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
  float response = 0.f;
  /// octave (pyramid layer) from which the keypoint has been extracted
  unsigned char octave = 0;
  /// object class (if the keypoints need to be clustered by an object they belong to)
  unsigned char class_id = 0;
};

/**
 * @brief AKAZE Class Declaration
 */
class AKAZE
{
public:

  struct TEvolution
  {
    /// current gaussian image
    image::Image<float> cur;
    /// current x derivatives
    image::Image<float> Lx;
    /// current y derivatives
    image::Image<float> Ly;
    /// current Determinant of Hessian
    image::Image<float> Lhess;
  };

  /**
   * @brief Constructor
   * @param[in] image Input image
   * @param[in] options AKAZE configuration options
   */
  AKAZE(const image::Image<float>& image, const AKAZEOptions& options);

  /**
   * @brief Compute the AKAZE non linear diffusion scale space per slice
   */
  void computeScaleSpace();

  /**
   * @brief Detect feature in the AKAZE scale space
   * @param[in,out] keypoints AKAZE keypoints
   */
  void featureDetection(std::vector<AKAZEKeypoint>& keypoints) const;

  /**
   * @brief Grid filtering of the detected keypoints
   * @param[in,out] keypoints AKAZE keypoints
   */
  void gridFiltering(std::vector<AKAZEKeypoint>& keypoints) const;

  /**
   * @brief Sub-pixel refinement of the detected keypoints
   * @param[in,out] keypoints AKAZE keypoints
   */
  void subpixelRefinement(std::vector<AKAZEKeypoint>& keypoints) const;

  /**
   * @brief Sub-pixel refinement of the detected keypoint
   * @param[in,out] keypoint AKAZE keypoint
   */
  bool subpixelRefinement(AKAZEKeypoint& keypoint, const image::Image<float>& Ldet) const;

  /**
   * @brief This method computes the main orientation for a given keypoint
   * @param[in,out] keypoint Input keypoint
   * @param[in] Lx X derivatives
   * @param[in] Ly Y derivatives
   * @note The orientation is computed using a similar approach as described in the
   * original SURF method. See Bay et al., Speeded Up Robust Features, ECCV 2006
  */
  void computeMainOrientation(AKAZEKeypoint& keypoint,
                              const image::Image<float>& Lx,
                              const image::Image<float>& Ly) const;

  /**
   * @brief Get scale space slices
   * @return slices
   */
  inline const std::vector<TEvolution>& getSlices() const
  {
    return _evolution;
  }

private:
  /// configuration options for AKAZE
  AKAZEOptions _options;
  /// vector of nonlinear diffusion evolution (Scale Space)
  std::vector<TEvolution> _evolution;
  /// input image
  image::Image<float> _input;
};

} // namespace feature
} // namespace aliceVision
