// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// Copyright (c) 2010 libmv contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "Unnormalizer.hpp"

namespace aliceVision {
namespace multiview {

// Denormalize the results. See HZ page 109.
void UnnormalizerT::unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H)
{
  *H = T2.transpose() * (*H) * T1;
}

// Denormalize the results. See HZ page 109.
void UnnormalizerI::unnormalize(const Mat3 &T1, const Mat3 &T2, Mat3 *H)
{
  *H = T2.inverse() * (*H) * T1;
}

void UnnormalizerResection::unnormalize(const Mat &T, const Mat &U, Mat34 *P)
{
  *P = T.inverse() * (*P);
}

} // namespace multiview
} // namespace aliceVision
