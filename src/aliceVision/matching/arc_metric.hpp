
// Copyright (c) 2012, 2013 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#ifndef OPENMVG_MATCHING_METRIC_H
#define OPENMVG_MATCHING_METRIC_H

#include "openMVG/matching/metric_hamming.hpp"
#include "openMVG/numeric/accumulator_trait.hpp"
#include <openMVG/config.hpp>

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_SSE)
#include <openMVG/system/Logger.hpp>
#include <xmmintrin.h>
#endif

#include <cstddef>

#include <typeinfo>
#include <math.h>
#include <cmath>

namespace openMVG {
namespace matching {

/// Squared Euclidean distance functor.
template<class T>
struct L2_Simple
{
  typedef T ElementType;
  typedef typename Accumulator<T>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
  inline ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const
  {
    ResultType result = ResultType();
    ResultType diff;
    for(size_t i = 0; i < size; ++i ) {
      diff = *a++ - *b++;
      result += diff*diff;
    }
    return result;
  }
};

/// Squared Euclidean distance functor (vectorized version)
template<class T>
struct L2_Vectorized
{
  typedef T ElementType;
  typedef typename Accumulator<T>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
  inline ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const
  {
    ResultType result = ResultType();


    //Using int is faster, but will it give us calculation errors in the long run?
    //ResultType diff0, diff1, diff2, diff3;
    int diff0, diff1, diff2, diff3;


    
    Iterator1 last = a + size;
    Iterator1 lastgroup = last - 3;
    
    /**********************************************************************/
    /**********************************************************************/
    
    //ResultType dot, dota, dotb, magnitude;

    //int works on a+=4, but not on a+=2 or a+=8, use double if so
    unsigned int dot = 0;// dota = 0, dotb = 0;
    //double magnitude = 257556.0;
    unsigned int magnitude = 257556; //507*508. can be tweeked

    //calculating arc length between the vectors
    while(a < lastgroup) {
                   
      diff0 = a[0] * b[0];
      diff1 = a[1] * b[1];
      diff2 = a[2] * b[2];
      diff3 = a[3] * b[3];
      
      dot += diff0 + diff1 + diff2 + diff3;      

/*      
      diff0 = a[0] * a[0];
      diff1 = a[1] * a[1];
      diff2 = a[2] * a[2];
      diff3 = a[3] * a[3];
      
      dota += diff0 + diff1 + diff2 + diff3;      
      
      diff0 = b[0] * b[0];
      diff1 = b[1] * b[1];
      diff2 = b[2] * b[2];
      diff3 = b[3] * b[3];
      
      dotb += diff0 + diff1 + diff2 + diff3;
      
*/   
      a+=4;
      b+=4;
      }
    
  // while(a < last) {
  //   diff0 = *a++ - *b++;
  //  result += diff0 * diff0;
  //  }

//    std::cout << "dota: " << sqrt(dota) << " dotb: " << sqrt(dotb) << std::endl;
    
    float pi = 2*M_PI; //get full circle

    //std::cout << "original dota: " << dota << std::endl;
    //std::cout << "original dotb: " << dotb << std::endl;
    //std::cout << "original dot: " << dot << std::endl;
    // std::cout << dotb << std::endl;
    //std::cout << dot << std::endl;

    //std::exit(1);
    //magnitude = sqrt(dota)*sqrt(dotb);
    //std::cout << sqrt(sqrt(dota)*sqrt(dotb)) << std::endl;
    //magnitude = 257556.0;  //507*508 most common number if doing the lenght calculation, should be 512, change threshold if using this number

    //double test = (double)257556;
    //float tmp = dot/ (float)magnitude;   
    
    //   std::cout << dot << "   " << test << "   " << dot/test << std::endl;
//    result = (acos(dot/257556)/360)*pi;

    //float tmp2 = (-0.69813170079773212 * tmp * tmp - 0.87266462599716477) * tmp + 1.5707963267948966;
    //float tmp2 = (-0.69813170 * tmp * tmp - 0.87266462) * tmp + 1.57079632;
    //result = tmp2/360*pi;
    result = (acosf(dot/(float)magnitude)/360)*pi;
    //result = (acosh(dot/magnitude)/360)*pi;
    
    //result = dot/dota;
    //std::cout << magnitude << std::endl;
    //result = dot/magnitude;
    /*
    std::cout << "**********************" << std::endl;
    std::cout << (float) a[0] << std::endl;
    std::cout << (float) b[0] << std::endl;
    std::cout << (float) a[0]/512 << std::endl;
    std::cout << (float) b[0]/512 << std::endl;
    std::cout << (((float)(a[0]))/512) * (((float)b[0])/512) << std::endl;
    std::cout << "**********************" << std::endl;    
    //std::cout << a.size() << std::endl;
    */
    
    /**********************************************************************/
    /**********************************************************************/
    /*
    // Process 4 items with each loop for efficiency.
     while (a < lastgroup) {
      diff0 = a[0] - b[0];
      diff1 = a[1] - b[1];
      diff2 = a[2] - b[2];
      diff3 = a[3] - b[3];
      result += diff0 * diff0 + diff1 * diff1 + diff2 * diff2 + diff3 * diff3;
      a += 4;
      b += 4;
    }
    // Process last 0-3 pixels.  Not needed for standard vector lengths.
    while (a < last) {
      diff0 = *a++ - *b++;
      result += diff0 * diff0;
    }
    
    //std::cout << typeid(a).name() << std::endl;
    
    */
    return result;
    
  }
};

#if OPENMVG_IS_DEFINED(OPENMVG_HAVE_SSE)

namespace optim_ss2{

  /// Union to switch between SSE and float array
  union sseRegisterHelper
  {
      __m128 m;
      float f[4];
  };

  // Euclidean distance (SSE method) (squared result)
  inline float l2_sse(const float * b1, const float * b2, int size)
  {
    float* b1Pt = (float*)b1;
    float* b2Pt = (float*)b2;
    if(size%4 == 0)
    {
      __m128 srcA, srcB, temp, cumSum;
      float zeros[4] = {0.f,0.f,0.f,0.f};
      cumSum = _mm_load_ps( zeros );
      for(int i = 0 ; i < size; i+=4)
      {
        srcA = _mm_load_ps(b1Pt+i);
        srcB = _mm_load_ps(b2Pt+i);
        //-- Subtract
        temp = _mm_sub_ps( srcA, srcB );
        //-- Multiply
        temp =  _mm_mul_ps( temp, temp );
        //-- sum
        cumSum = _mm_add_ps( cumSum, temp );
      }
      sseRegisterHelper res;
      res.m = cumSum;
      return (res.f[0]+res.f[1]+res.f[2]+res.f[3]);
    }
    else
    {
      OPENMVG_LOG_WARNING("/!\\ size is not modulus 4, distance cannot be performed in SSE");
      return 0.0f;
    }
  }
} // namespace optim_ss2

// Template specification to run SSE L2 squared distance
//  on float vector
template<>
struct L2_Vectorized<float>
{
  typedef float ElementType;
  typedef Accumulator<float>::Type ResultType;

  template <typename Iterator1, typename Iterator2>
  inline ResultType operator()(Iterator1 a, Iterator2 b, size_t size) const
  {
    return optim_ss2::l2_sse(a,b,size);
  }
};

#endif // OPENMVG_HAVE_SSE

}  // namespace matching
}  // namespace openMVG

#endif // OPENMVG_MATCHING_METRIC_H
