// This file is part of the AliceVision project and is made available under
// the terms of the MPL2 license (see the COPYING.md file).

#ifndef ALICEVISION_MULTIVIEW_CONDITIONNING_H_
#define ALICEVISION_MULTIVIEW_CONDITIONNING_H_

#include "aliceVision/numeric/numeric.h"

//-- Implementation of normalized coordinates.
// Normalization improve accuracy of results and provide benefits
//  that make scale and coordinate origin invariant.
// The implementation follows Algorithm 4.2 from HZ page 109.

namespace aliceVision {

// Point conditioning :
void PreconditionerFromPoints(const Mat &points, Mat3 *T);

/// Normalize input point for a given T transform matrix
void ApplyTransformationToPoints(const Mat &points,
                                 const Mat3 &T,
                                 Mat *transformed_points);

// Normalize point in [-.5, .5] and return transformation matrix
void NormalizePoints(const Mat &points,
                     Mat *normalized_points,
                     Mat3 *T);

/// Point conditioning (compute Transformation matrix)
void PreconditionerFromImageSize(int width, int height, Mat3 *T);

///  Normalize point rom image coordinates to [-.5, .5]
void NormalizePointsFromImageSize(const Mat &points,
                     Mat *normalized_points,
                     Mat3 *T, int width, int height);


/// Unnormalize using Inverse
struct UnnormalizerI {
  // Denormalize the results. See HZ page 109.
  static void Unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H);
};

/// Unnormalize using Transpose
struct UnnormalizerT {
  // Denormalize the results. See HZ page 109.
  static void Unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H);
};

} //namespace aliceVision


#endif // ALICEVISION_MULTIVIEW_CONDITIONNING_H_
