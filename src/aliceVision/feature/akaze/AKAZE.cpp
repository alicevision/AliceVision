// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/feature/akaze/AKAZE.hpp>
#include <aliceVision/feature/imageStats.hpp>
#include <aliceVision/system/Logger.hpp>
#include <aliceVision/config.hpp>

namespace aliceVision {
namespace feature {

/// lookup table for 2d gaussian (sigma = 2.5) where (0,0) is top left and (6,6) is bottom right
const float gauss25[7][7] = {
  {0.02546481f,	0.02350698f,	0.01849125f,	0.01239505f,	0.00708017f,	0.00344629f,	0.00142946f},
  {0.02350698f,	0.02169968f,	0.01706957f,	0.01144208f,	0.00653582f,	0.00318132f,	0.00131956f},
  {0.01849125f,	0.01706957f,	0.01342740f,	0.00900066f,	0.00514126f,	0.00250252f,	0.00103800f},
  {0.01239505f,	0.01144208f,	0.00900066f,	0.00603332f,	0.00344629f,	0.00167749f,	0.00069579f},
  {0.00708017f,	0.00653582f,	0.00514126f,	0.00344629f,	0.00196855f,	0.00095820f,	0.00039744f},
  {0.00344629f,	0.00318132f,	0.00250252f,	0.00167749f,	0.00095820f,	0.00046640f,	0.00019346f},
  {0.00142946f,	0.00131956f,	0.00103800f,	0.00069579f,	0.00039744f,	0.00019346f,	0.00008024f}
};

/// factor for the multiscale derivatives
const float derivativeFactor = 1.5f;

/**
 * @brief Compute slice scale
 * @param[in] sigma0 First octave initial scale
 * @param[in] p Octave index
 * @param[in] q Slice index
 * @param[in] Q nbSlice
 */
inline float sigma(const float sigma0 , const int p , const int q , const int Q)
{
  if(p == 0 && q == 0)
    return sigma0;
  else
    return sigma0 * powf(2.f, p + static_cast<float>(q) / static_cast<float>(Q)) ;
}

/**
 * @brief Compute an AKAZE slice
 * @param[in] src Input image for the given octave
 * @param[in] p Octave index
 * @param[in] q Slice index
 * @param[in] nbSlice Slices per octave
 * @param[in] sigma0 First octave initial scale
 * @param[in] contrastFactor
 * @param Li Diffusion image
 * @param Lx X derivatives
 * @param Ly Y derivatives
 * @param Lhess Det(Hessian)
 */
void computeAKAZESlice(const image::Image<float>& src,
                       const int p,
                       const int q,
                       const int nbSlice,
                       const float sigma0,
                       const float contrastFactor,
                       image::Image<float>& Li,
                       image::Image<float>& Lx,
                       image::Image<float>& Ly,
                       image::Image<float>& Lhess)
{
  const float sigmaCur = sigma(sigma0, p, q, nbSlice);
  const float ratio = 1 << p; //pow(2,p);
  const int sigmaScale = MathTrait<float>::round(sigmaCur * derivativeFactor / ratio);

  image::Image<float> smoothed;

  if(p == 0 && q == 0)
  {
    // compute new image
    image::ImageGaussianFilter(src , sigma0 , Li, 0, 0);
  }
  else
  {
    // general case
    image::Image<float> in;
    if( q == 0 )
    {
      image::ImageHalfSample(src , in);
    }
    else
    {
      in = src;
    }

    const float sigmaPrev = ( q == 0 ) ? sigma(sigma0, p - 1, nbSlice - 1, nbSlice) : sigma(sigma0, p, q - 1, nbSlice);

    // compute non linear timing between two consecutive slices
    const float t_prev = 0.5f * (sigmaPrev * sigmaPrev);
    const float t_cur  = 0.5f * (sigmaCur * sigmaCur);
    const float total_cycle_time = t_cur - t_prev;

    // compute first derivatives (Scharr scale 1, non normalized) for diffusion coef
    image::ImageGaussianFilter(in , 1.f , smoothed, 0, 0 );
    image::ImageScharrXDerivative(smoothed, Lx, false);
    image::ImageScharrYDerivative(smoothed, Ly, false);

    // compute diffusion coefficient
    image::Image<float> & diff = smoothed; // diffusivity image (reuse existing memory)
    image::ImagePeronaMalikG2DiffusionCoef(Lx, Ly, contrastFactor, diff) ;

    // compute FED cycles
    std::vector<float> tau ;
    image::FEDCycleTimings(total_cycle_time, 0.25f, tau);
    image::ImageFEDCycle(in, diff, tau);
    Li = in ; // evolution image
  }

  // compute Hessian response
  if(p == 0 && q == 0)
  {
    smoothed = Li ;
  }
  else
  {
    // add a little smooth to image (for robustness of Scharr derivatives)
    image::ImageGaussianFilter(Li, 1.f, smoothed, 0, 0);
  }

  // compute true first derivatives
  image::ImageScaledScharrXDerivative(smoothed, Lx, sigmaScale);
  image::ImageScaledScharrYDerivative(smoothed, Ly, sigmaScale);

  // second order spatial derivatives
  image::Image<float> Lxx, Lyy, Lxy;
  image::ImageScaledScharrXDerivative(Lx, Lxx, sigmaScale);
  image::ImageScaledScharrYDerivative(Lx, Lxy, sigmaScale);
  image::ImageScaledScharrYDerivative(Ly, Lyy, sigmaScale);

  Lx *= static_cast<float>(sigmaScale);
  Ly *= static_cast<float>(sigmaScale);

  // compute Determinant of the Hessian
  Lhess.resize(Li.Width(), Li.Height());
  const float sigmaSizeQuad = Square(sigmaScale) * Square(sigmaScale);
  Lhess.array() = (Lxx.array() * Lyy.array() - Lxy.array().square()) * sigmaSizeQuad;
}

#if DEBUG_OCTAVE
template <typename Image>
void convertScale(Image& src)
{
   typename image::Image::Tpixel min_val = src.minCoeff(), max_val = src.maxCoeff();
   src = src.array() - min_val;
   src /= max_val;
}
#endif // DEBUG_OCTAVE

AKAZE::AKAZE(const image::Image<float>& image, const AKAZEOptions& options):
    _input(image),
    _options(options)
{
  _options.descFactor = std::max(6.f * sqrtf(2.f), _options.descFactor);

  // safety check to limit the computable octave count
  const int nbOctaveMax = ceil(std::log2( std::min(_input.Width(), _input.Height())));
  _options.nbOctaves = std::min(_options.nbOctaves, nbOctaveMax);
}

void AKAZE::computeScaleSpace()
{
  float contrastFactor = computeAutomaticContrastFactor( _input, 0.7f);
  image::Image<float> input = _input;

  // octave computation
  for(int p = 0; p < _options.nbOctaves; ++p)
  {
    contrastFactor *= (p == 0) ? 1.f : 0.75f;

    for(int q = 0; q < _options.nbSlicePerOctave; ++q)
    {
      _evolution.emplace_back(TEvolution());
      TEvolution& evo = _evolution.back();

      // compute Slice at (p,q) index
      computeAKAZESlice(input, p, q, _options.nbSlicePerOctave, _options.sigma0, contrastFactor,
        evo.cur, evo.Lx, evo.Ly, evo.Lhess);

      // Prepare inputs for next slice
      input = evo.cur;

      // DEBUG octave image
#if DEBUG_OCTAVE
      std::stringstream str ;
      str << "./" << "_oct_" << p << "_" << q << ".png" ;
      image::Image<float> tmp = evo.cur;
      convertScale(tmp);
      image::Image< unsigned char > tmp2 ((tmp*255).cast<unsigned char>());
      image::writeImage(str.str(), tmp2, image::EImageColorSpace::NO_CONVERSION);
#endif // DEBUG_OCTAVE
    }
  }
}

void detectDuplicates(std::vector<std::pair<AKAZEKeypoint, bool>>& previous,
                      std::vector<std::pair<AKAZEKeypoint, bool>>& current)
{
  // mark duplicates, using a full search algorithm
  for(std::vector<std::pair<AKAZEKeypoint, bool> >::iterator p1=previous.begin(); p1<previous.end(); ++p1)
  {
    for(std::vector<std::pair<AKAZEKeypoint, bool> >::iterator p2 = current.begin(); p2<current.end(); ++p2)
    {
      if(p2->second == true)
        continue;

      // check spatial distance
      const float dist = Square(p1->first.x-p2->first.x)+Square(p1->first.y-p2->first.y);
      if(dist <= Square(p1->first.size) && dist != 0.f)
      {
        if (p1->first.response < p2->first.response)
          p1->second = true; // mark as duplicate key point
        else
          p2->second = true; // mark as duplicate key point
        break; // no other point can be so close, so skip to the next iteration
      }
    }
  }
}

void AKAZE::featureDetection(std::vector<AKAZEKeypoint>& keypoints) const
{
  std::vector<std::vector<std::pair<AKAZEKeypoint, bool>>> ptsPerSlice(_options.nbOctaves * _options.nbSlicePerOctave);

  #pragma omp parallel for schedule(dynamic)
  for(int p = 0 ; p < _options.nbOctaves ; ++p)
  {
    const float ratio = static_cast<float>(1 << p);

    for(int q = 0 ; q < _options.nbSlicePerOctave ; ++q)
    {
      const float sigma_cur = sigma( _options.sigma0 , p , q , _options.nbSlicePerOctave );
      const image::Image<float>& LDetHess = _evolution[_options.nbOctaves * p + q].Lhess;

      // check that the point is under the image limits for the descriptor computation
      const float borderLimit =
        MathTrait<float>::round(_options.descFactor * sigma_cur * derivativeFactor / ratio) + 1;

      for(int jx = borderLimit; jx < LDetHess.Height()-borderLimit; ++jx)
      {
        for(int ix = borderLimit; ix < LDetHess.Width()-borderLimit; ++ix)
        {
          const float value = LDetHess(jx, ix);

          // filter the points with the detector threshold
          if(value > _options.threshold &&
             value > LDetHess(jx-1, ix)   &&
             value > LDetHess(jx-1, ix+1) &&
             value > LDetHess(jx-1, ix-1) &&
             value > LDetHess(jx  , ix-1) &&
             value > LDetHess(jx  , ix+1) &&
             value > LDetHess(jx+1, ix-1) &&
             value > LDetHess(jx+1, ix)   &&
             value > LDetHess(jx+1, ix+1))
          {
            AKAZEKeypoint point;
            point.size = sigma_cur * derivativeFactor ;
            point.octave = p;
            point.response = fabs(value);
            point.x = ix * ratio + 0.5 * (ratio-1);
            point.y = jx * ratio + 0.5 * (ratio-1);
            point.angle = 0.0f;
            point.class_id = p * _options.nbSlicePerOctave + q;
            ptsPerSlice[_options.nbOctaves * p + q].emplace_back(point, false);
          }
        }
      }
    }
  }

  // filter duplicates
  detectDuplicates(ptsPerSlice[0], ptsPerSlice[0]);
  for (int k = 1; k < ptsPerSlice.size(); ++k)
  {
    detectDuplicates(ptsPerSlice[k], ptsPerSlice[k]);    // detect inter scale duplicates
    detectDuplicates(ptsPerSlice[k-1], ptsPerSlice[k]);  // detect duplicates using previous octave
  }

  for(std::size_t k = 0; k < ptsPerSlice.size(); ++k)
  {
    const std::vector< std::pair<AKAZEKeypoint, bool> >& vec_kp = ptsPerSlice[k];
    for(std::size_t i = 0; i < vec_kp.size(); ++i)
    {
      const auto& kp = vec_kp[i];

      // keep only the one marked as not duplicated
      if(!kp.second)
        keypoints.emplace_back(kp.first);
    }
  }
}

void AKAZE::gridFiltering(std::vector<AKAZEKeypoint>& keypoints) const
{
  if(_options.maxTotalKeypoints == 0 || keypoints.size() <= _options.maxTotalKeypoints)
    return;

  // sort keypoints by size to guarantee best points are kept
  // note: reordering keypoints also helps preventing grid-filtering from creating a 
  //       non-uniform distribution within cells when input keypoints are sorted by spatial coordinates
  std::sort(keypoints.begin(), keypoints.end(), [](const AKAZEKeypoint& a, const AKAZEKeypoint& b){ return a.size > b.size; });

  std::vector<AKAZEKeypoint> out_keypoints;
  out_keypoints.reserve(keypoints.size());

  const std::size_t sizeMat = _options.gridSize * _options.gridSize;
  const std::size_t keypointsPerCell = _options.maxTotalKeypoints / sizeMat;
  const double regionWidth = _input.Width() / static_cast<double>(_options.gridSize);
  const double regionHeight = _input.Height() / static_cast<double>(_options.gridSize);

  std::vector<std::size_t> countFeatPerCell(sizeMat, 0);
  std::vector<std::size_t> rejectedIndexes;

  for(std::size_t i = 0; i < keypoints.size(); ++i)
  {
    const AKAZEKeypoint& point = keypoints.at(i);
    const std::size_t cellX = std::min(std::size_t(point.x / regionWidth ), _options.gridSize);
    const std::size_t cellY = std::min(std::size_t(point.y / regionHeight), _options.gridSize);

    std::size_t& count = countFeatPerCell.at(cellX * _options.gridSize + cellY);
    ++count;

    if(count > keypointsPerCell)
    {
      rejectedIndexes.emplace_back(i);
      continue;
    }

    out_keypoints.emplace_back(point);
  }

  // if we don't have enough features (less than maxTotalKeypoints) after the grid filtering (empty regions in the grid for example).
  // we add the best other ones, without repartition constraint.
  if(out_keypoints.size() < _options.maxTotalKeypoints)
  {
    const std::size_t remainingElements = std::min(rejectedIndexes.size(), _options.maxTotalKeypoints - out_keypoints.size());
    ALICEVISION_LOG_TRACE("Grid filtering copy " << remainingElements << " remaining points.");

    for(std::size_t i = 0; i < remainingElements; ++i)
      out_keypoints.emplace_back(keypoints.at(rejectedIndexes.at(i)));
  }

  out_keypoints.swap(keypoints);
}

bool AKAZE::subpixelRefinement(AKAZEKeypoint& keypoint, const image::Image<float>& Ldet) const
{
  const unsigned int ratio = (1 << keypoint.octave);
  const int x = MathTrait<float>::round(keypoint.x / ratio);
  const int y = MathTrait<float>::round(keypoint.y / ratio);

  // compute the gradient
  const float Dx = 0.5f * (Ldet(y,x+1)  - Ldet(y,x-1));
  const float Dy = 0.5f * (Ldet(y+1, x) - Ldet(y-1, x));

  // compute the Hessian
  const float Dxx = Ldet(y, x+1) + Ldet(y, x-1) - 2.0f * Ldet(y, x);
  const float Dyy = Ldet(y+1, x) + Ldet(y-1, x) -2.0f * Ldet(y, x);
  const float Dxy = 0.25f * (Ldet(y+1, x+1) + Ldet(y-1, x-1)) - 0.25f * (Ldet(y-1, x+1) + Ldet(y+1, x-1));

  // solve the linear system
  Eigen::Matrix<double, 2, 2> A;
  Vec2 b;
  A << Dxx, Dxy, Dxy, Dyy;
  b << -Dx, -Dy;

  const Vec2 dst = A.fullPivLu().solve(b);

  if(fabs(dst(0)) <= 1.0 && fabs(dst(1)) <= 1.0)
  {
    keypoint.x += dst(0) * ratio + 0.5 * (ratio-1);
    keypoint.y += dst(1) * ratio + 0.5 * (ratio-1);
    return true;
  }

  // delete the point since its not stable
  return false;
}

void AKAZE::subpixelRefinement(std::vector<AKAZEKeypoint>& keypoints) const
{
  std::vector<AKAZEKeypoint> in_keypoints;
  in_keypoints.swap(keypoints);
  keypoints.reserve(in_keypoints.size());

  #pragma omp parallel for schedule(dynamic)
  for(int i = 0; i < static_cast<int>(in_keypoints.size()); ++i)
  {
    AKAZEKeypoint& point = in_keypoints[i];
    if(subpixelRefinement(point, this->_evolution[point.class_id].Lhess))
    {
      #pragma omp critical
      keypoints.emplace_back(point);
    }
  }
}

/// This function computes the angle from the vector given by (X Y). From 0 to 2*Pi
inline float getAngle(float x, float y)
{
  const float angle = atan2(y,x);
  // output angle between 0 and 2Pi
  return angle > 0.0f ? angle : 2.f * M_PI + angle;
}

void AKAZE::computeMainOrientation(AKAZEKeypoint& keypoint,
                                   const image::Image<float> & Lx,
                                   const image::Image<float> & Ly) const
{
  int ix = 0, iy = 0, idx = 0;
  const int TABSIZE = 109;
  float resX[TABSIZE], resY[TABSIZE], Ang[TABSIZE];
  const short id[] = {6,5,4,3,2,1,0,1,2,3,4,5,6};

  // variables for computing the dominant direction
  float sumX = 0.0f, sumY = 0.0f, max = 0.0f, ang1 = 0.0f, ang2 = 0.0f;

  // get the information from the keypoint
  const unsigned int ratio = (1 << keypoint.octave);
  const int s = MathTrait<float>::round(keypoint.size/ratio);
  const float xf = keypoint.x/ratio;
  const float yf = keypoint.y/ratio;

  // calculate derivatives responses for points within radius of 6*scale
  for(int i = -6; i <= 6; ++i)
  {
    for(int j = -6; j <= 6; ++j)
    {
      if(i*i + j*j < 36)
      {
        iy = MathTrait<float>::round(yf + j * s);
        ix = MathTrait<float>::round(xf + i * s);

        const float gweight = gauss25[id[i+6]][id[j+6]];
        resX[idx] = gweight * Lx(iy, ix);
        resY[idx] = gweight * Ly(iy, ix);

        Ang[idx] = getAngle(resX[idx],resY[idx]);
        ++idx;
      }
    }
  }

  // loop slides pi/3 window around feature point
  for (ang1 = 0.f; ang1 < 2.0f * M_PI;  ang1+=0.15f)
  {
    ang2 =(ang1 + M_PI / 3.0f > 2.0f * M_PI ?
      ang1 - 5.0f * M_PI / 3.0f :
      ang1 + M_PI / 3.0f);
    sumX = sumY = 0.f;

    for(std::size_t k = 0; k < idx; ++k)
    {
      // get angle from the x-axis of the sample point
      const float& ang = Ang[k];

      // determine whether the point is within the window
      if (ang1 < ang2 && ang1 < ang && ang < ang2)
      {
        sumX += resX[k];
        sumY += resY[k];
      }
      else if (ang2 < ang1 &&
               ((ang > 0 && ang < ang2) || (ang > ang1 && ang < 2.0f * M_PI) ))
      {
        sumX += resX[k];
        sumY += resY[k];
      }
    }

    // if the vector produced from this window is longer than all
    // previous vectors then this forms the new dominant direction
    if (sumX * sumX + sumY * sumY > max)
    {
      // store largest orientation
      max = sumX * sumX + sumY * sumY;
      keypoint.angle = getAngle(sumX, sumY);
    }
  }
}

} // namespace feature
} // namespace aliceVision

