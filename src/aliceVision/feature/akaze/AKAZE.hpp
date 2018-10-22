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

//------------------
//-- Bibliography --
//------------------
//- [1] "Fast Explicit Diffusion for Accelerated Features in Nonlinear Scale Spaces."
//- Authors: Pablo F. Alcantarilla, Jesús Nuevo and Adrien Bartoli.
//- Date: September 2013.
//- Conference : BMVC.
//------
// Notes:
// This implementation is done for AliceVision:
//  - code look the same but less memory are allocated,
//  - only Perona and Malik G2 diffusion (PM_G2) is implemented.
//------
// To see the reference implementation (OpenCV based):
// please visit https://github.com/pablofdezalc/akaze
// Authors: Pablo F. Alcantarilla (1), Jesus Nuevo (2)
// Institutions:
//  Toshiba Research Europe Ltd (1)
//  TrueVision Solutions (2)
//------

#include <aliceVision/image/all.hpp>
#include <aliceVision/numeric/numeric.hpp>
#include <aliceVision/numeric/MathTrait.hpp>

#include <aliceVision/feature/PointFeature.hpp>
#include <aliceVision/feature/Descriptor.hpp>

namespace aliceVision {
namespace feature {

struct AKAZEConfig
{
  int iNbOctave = 4;          ///< Octave to process
  int iNbSlicePerOctave = 4;  ///< Levels per octave
  float fSigma0 = 1.6f;       ///< Initial sigma offset (used to suppress low level noise)
  float fThreshold = 0.0008f; ///< Hessian determinant threshold
  float fDesc_factor = 1.f;   ///< Magnifier used to describe an interest point
  std::size_t gridSize = 4;
  std::size_t maxTotalKeypoints = 1000;
};

struct AKAZEKeypoint{

  AKAZEKeypoint()
  {
    x = y = size = angle = response = 0.f;
    class_id = octave = 0;
  }

  float x,y;      //!< coordinates of the keypoints
  float size;     //!< diameter of the meaningful keypoint neighborhood
  float angle;    //!< computed orientation of the keypoint (-1 if not applicable);
                  //!< it's in [0,360) degrees and measured relative to
                  //!< image coordinate system, ie in clockwise.
  float response; //!< the response by which the most strong keypoints have been selected. Can be used for the further sorting or subsampling
  unsigned char octave; //!< octave (pyramid layer) from which the keypoint has been extracted
  unsigned char class_id; //!< object class (if the keypoints need to be clustered by an object they belong to)
};

struct TEvolution
{
  image::Image<float>
    cur,    ///< Current gaussian image
    Lx,     ///< Current x derivatives
    Ly,     ///< Current y derivatives
    Lhess;  ///< Current Determinant of Hessian
};

// AKAZE Class Declaration
class AKAZE {

private:

  AKAZEConfig options_;               ///< Configuration options for AKAZE
  std::vector<TEvolution> evolution_;	///< Vector of nonlinear diffusion evolution (Scale Space)
  image::Image<float> in_;            ///< Input image

public:

  /// Constructor
  AKAZE(const image::Image<float> & in, const AKAZEConfig & options);

  /// Compute the AKAZE non linear diffusion scale space per slice
  void Compute_AKAZEScaleSpace(void);

  /// Detect AKAZE feature in the AKAZE scale space
  void Feature_Detection(std::vector<AKAZEKeypoint>& kpts) const;

  /// Grid filtering of the detected keypoints
  void gridFiltering(std::vector<AKAZEKeypoint>& kpts) const;

  /// Sub pixel refinement of the detected keypoints
  void Do_Subpixel_Refinement(std::vector<AKAZEKeypoint>& kpts) const;

  /// Sub pixel refinement of a keypoint
  bool Do_Subpixel_Refinement(AKAZEKeypoint & kpts, const image::Image<float> & Ldet) const;

  /// Scale Space accessor
  const std::vector<TEvolution> & getSlices() const {return evolution_;}

  /**
   * @brief This method computes the main orientation for a given keypoint
   * @param kpt Input keypoint
   * @note The orientation is computed using a similar approach as described in the
   * original SURF method. See Bay et al., Speeded Up Robust Features, ECCV 2006
  */
  void Compute_Main_Orientation(
    AKAZEKeypoint& kpt,
    const image::Image<float> & Lx,
    const image::Image<float> & Ly) const;

  /// Compute an AKAZE slice
  static void ComputeAKAZESlice(
    const image::Image<float> & src, // Input image for the given octave
    const int p , // octave index
    const int q , // slice index
    const int nbSlice , // slices per octave
    const float sigma0 , // first octave initial scale
    const float contrast_factor ,
    image::Image<float> & Li, // Diffusion image
    image::Image<float> & Lx, // X derivatives
    image::Image<float> & Ly, // Y derivatives
    image::Image<float> & Lhess // Det(Hessian)
    );

  /// Compute Contrast Factor
  static float ComputeAutomaticContrastFactor(
    const image::Image<float> & src,
    const float percentile );
};

} // namespace feature
} // namespace aliceVision
