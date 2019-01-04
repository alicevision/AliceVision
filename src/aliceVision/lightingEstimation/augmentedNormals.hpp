// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/image/Image.hpp>
#include <aliceVision/image/pixelTypes.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>


namespace aliceVision {
namespace lightingEstimation {

using Eigen::MatrixXf;

/**
 * @brief Augmented normals for augmented Lambert's law (using Spherical Harmonics model)
 * Composed of 9 coefficients evaluated from normal coordinates (nx, ny, nz):
 * New coefficients are: [nx; ny; nz; 1; nx*ny; nx*nz; ny*nz; nx^2 * ny^2; 3nz^2-1]
 */
class AugmentedNormal : public Eigen::Matrix<float, 9, 1, 0, 9, 1>
{
  using T = float;
  typedef Eigen::Matrix<T, 9, 1, 0, 9, 1> BaseMatrix;
  typedef T TBase;
public:

  AugmentedNormal() = default;

  /**
  * @brief Constructor from a normal value
  */
  inline AugmentedNormal(T nx, T ny, T nz)
  {
      (*this)(0) = nx;
      (*this)(1) = ny;
      (*this)(2) = nz;
      (*this)(3) = 1;
      (*this)(4) = nx*ny;
      (*this)(5) = nx*nz;
      (*this)(6) = ny*nz;
      (*this)(7) = nx*nx - ny*ny;
      (*this)(8) = 3 * nz * nz - 1;
  }

  /**
  * @brief Constructor from eigen Matrix
  */
  explicit inline AugmentedNormal(const BaseMatrix& m)
    : BaseMatrix(m)
  {
  }

  explicit inline AugmentedNormal(const Eigen::Matrix<T, 3, 1, 0, 3, 1>& m)
    : AugmentedNormal(m(0), m(1), m(2))
  {
  }

  inline const T& nx() const
  {
    return ( *this )( 0 );
  }
  inline T& nx()
  {
    return ( *this )( 0 );
  }

  inline const T& ny() const
  {
    return ( *this )( 1 );
  }
  inline T& ny()
  {
    return ( *this )( 1 );
  }

  inline const T& nz() const
  {
    return ( *this )( 2 );
  }
  inline T& nz()
  {
    return ( *this )( 2 );
  }

  inline const T& nambiant() const
  {
    return ( *this )( 3 );
  }
  inline T& nambiant()
  {
    return ( *this )( 3 );
  }

  inline const T& nx_ny() const
  {
    return ( *this )( 4 );
  }
  inline T& nx_ny()
  {
    return ( *this )( 4 );
  }

  inline const T& nx_nz() const
  {
    return ( *this )( 5 );
  }
  inline T& nx_nz()
  {
    return ( *this )( 5 );
  }

  inline const T& ny_nz() const
  {
    return ( *this )( 6 );
  }
  inline T& ny_nz()
  {
    return ( *this )( 6 );
  }

  inline const T& nx2_ny2() const
  {
    return ( *this )( 7 );
  }
  inline T& nx2_ny2()
  {
    return ( *this )( 7 );
  }

  inline const T& nz2() const
  {
    return ( *this )( 8 );
  }
  inline T& nz2()
  {
    return ( *this )( 8 );
  }

};


}
}
