// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "aliceVision/numeric/numeric.hpp"
#include <memory>

namespace aliceVision
{
  namespace image
  {

    /* There is no use for extended stride for the moment */
    using EigenStride = Eigen::Stride<0, 0>;

    /* An image is always a dynamic array with row major pixel ordering */
    template <typename T>
    using EigenRowMatrixT = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

    /* Define a map to an image container */
    template <typename T>
    using EigenMapTypeT = Eigen::Map<EigenRowMatrixT<T>, Eigen::AlignmentType::Unaligned, EigenStride>;

    /*A container defines the pixel storage in memory */
    template <class T>
    class BaseContainer {
    public:
      virtual ~BaseContainer() = default;
      virtual BaseContainer * clone() = 0; 
      virtual T * getDataPtr() = 0;
      virtual bool resize( int width, int height, bool fInit = true, const T val = T( 0 )) = 0;
    };

    /**
     * Classic Dense in-core container
     * Using Eigen for management
     */
    template <class T>
    class EigenContainer : public BaseContainer<T> {
    public:
      EigenContainer(int height, int width) : _contained(height, width){ 
      }

      EigenContainer(const EigenContainer & other) : _contained(other._contained) {
      }

      virtual BaseContainer<T> * clone() {
        
        return new EigenContainer(*this);
      }

      EigenRowMatrixT<T> & getContained() {
        return _contained;
      }

      virtual T * getDataPtr() {
        return _contained.data();
      }

      virtual bool resize( int width, int height, bool fInit = true, const T val = T( 0 )) {

        _contained.resize(height, width);
        if (fInit) {
          _contained.fill(val);
        }

        return true;
      }

      virtual ~EigenContainer() = default;

    protected:
      EigenRowMatrixT<T> _contained;
    };

    template <typename T>
    class Image : public EigenMapTypeT<T>
    {
    public:
      using Tpixel = T;
      using EigenRowMatrix = EigenRowMatrixT<T>;
      using EigenMapType = EigenMapTypeT<T>;

      /**
       * @brief Default constructor
       * @note This create an empty image
       */
      inline Image() : EigenMapType(nullptr, 0, 0)
      {
        
        std::shared_ptr<EigenContainer<T>> container = std::make_shared<EigenContainer<T>>(1, 1);        
        _container = container;        
        new ((EigenMapType*)this) EigenMapType(_container->getDataPtr(), 1, 1);
      }

      /**
      * @brief Full constructor
      * @param width Width of the image (ie number of column)
      * @param height Height of the image (ie number of row)
      * @param fInit Tell if the image should be initialized
      * @param val If fInit is true, set all pixel to the specified value
      */
      inline Image(int width, int height, bool fInit = false, const T val = T()) 
      :  EigenMapType(nullptr, 0, 0)
      {
        
        std::shared_ptr<EigenContainer<T>> container = std::make_shared<EigenContainer<T>>(height, width);
        
        if (fInit) {
          container->getContained().fill(val);
        }
        
        _container = container;   

        new ((EigenMapType*)this) EigenMapType(_container->getDataPtr(), height, width);
      };

      /**
      * @brief Copy constructor
      * @param I Source image
      */
      inline Image( const Image & I )
      : EigenMapType(nullptr, 0, 0)
      {

        size_t width = I.cols();
        size_t height = I.rows();

        _container = std::shared_ptr<BaseContainer<T>>(I._container->clone());

        new ((EigenMapType*)this) EigenMapType(_container->getDataPtr(), height, width);
      }

      /**
      * @brief Copy constructor
      * @param I Source matrix
      */
      inline Image( const EigenRowMatrix & I )
      : EigenMapType(nullptr, 0, 0)
      {        
        size_t width = I.cols();
        size_t height = I.rows();

        std::shared_ptr<EigenContainer<T>> container = std::make_shared<EigenContainer<T>>(height, width);
        container->getContained() = I;

        _container = container;   

        new ((EigenMapType*)this) EigenMapType(_container->getDataPtr(), height, width);
      }

      /**
      * @brief Move constructor
      * @param src Source image
      */
      inline Image(Image && src)
      : EigenMapType(nullptr, 0, 0)
      {
         _container = std::move(src._container);
        new ((EigenMapType*)this) EigenMapType(_container->getDataPtr(), src.rows(), src.cols());
      }

      /**
      * @brief Assignment operator
      * @param I Source image
      * @return Image after assignment
      */
      inline Image& operator=(const Image & I)
      {
        size_t width = I.cols();
        size_t height = I.rows();

        _container = std::shared_ptr<BaseContainer<T>>(I._container->clone());

        new ((EigenMapType*)this) EigenMapType(_container->getDataPtr(), height, width);

        return *this;
      }

      /**
      * @brief Assignment operator
      * @param I Source image
      * @return Image after assignment
      */
      inline Image& operator=(const EigenRowMatrix & M)
      {
        size_t width = M.cols();
        size_t height = M.rows();

        std::shared_ptr<EigenContainer<T>> container = std::make_shared<EigenContainer<T>>(height, width);
        container->getContained() = M;

        _container = container;   

        new ((EigenMapType*)this) EigenMapType(_container->getDataPtr(), height, width);

        return *this;
      }

      /**
      * @brief destructor
      */
      virtual ~Image() = default;
  
      /**
      * @brief Change geometry of image
      * @param width New width of image
      * @param height New height of image
      * @param fInit Indicate if new image should be initialized
      * @param val if fInit is true all pixel in the new image are set to this value
      */
      inline void resize( int width, int height, bool fInit = true, const T val = T( 0 ))
      {
        if (_container) {
          if (_container->resize(width, height, fInit, val)) {
            new ((EigenMapType*)this) EigenMapType(_container->getDataPtr(), height, width);
          }
        }
      }

      /**
       * @brief Retrieve the width of the image
       * @return Width of image
       */
      inline int Width()  const
      {
        return static_cast<int>(this->cols());
      }

      /**
       * @brief Retrieve the height of the image
       * @return Height of the image
       */
      inline int Height() const
      {
        
        return static_cast<int>( this->rows() );
      }

      /**
      * @brief Return the depth in byte of the pixel
      * @return depth of the pixel (in byte)
      * @note (T=unsigned char will return 1)
      */
      inline int Depth() const
      {
        return sizeof(Tpixel);
      }

      /**
      * @brief constant random pixel access
      * @param y Index of the row
      * @param x Index of the column
      * @return Constant pixel reference at position (y,x)
      */
      inline const T& operator()(int y, int x) const
      {
        
        return EigenMapType::operator()(y, x);
      }

      /**
       * @brief random pixel access
       * @param y Index of the row
       * @param x Index of the column
       * @return Pixel reference at position (y,x)
       */
      inline T& operator()(int y, int x)
      {
        return EigenMapType::operator()(y, x);
      }

      /**
       * @brief random pixel access
       * @param i position of the pixel in row order
       * @return Pixel reference at position i
       */
      [[deprecated]] const inline T& operator()(int i) const
      {
        int y = i / this->cols();
        int x = i % this->cols();
        
        return EigenMapType::operator()(y, x);
      }

      /**
       * @brief random pixel access
       * @param i position of the pixel in row order
       * @return Pixel reference at position i
       */
      [[deprecated]] inline T& operator()(int i)
      {
        int y = i / this->cols();
        int x = i % this->cols();
        
        return EigenMapType::operator()(y, x);
      }


      /**
      * @brief Get low level access to the internal pixel data
      * @return const reference to internal matrix data
      */
      inline const EigenMapType & GetMat() const
      {
        return *this;
      }

      inline EigenMapType & GetMat()
      {
        return *this;
      }
      
      /**
      * @brief Tell if a point is inside the image.
      * @param y Index of the row
      * @param x Index of the column
      * @retval true If pixel (y,x) is inside the image
      * @retval false If pixel (y,x) is outside the image
      */
      inline bool Contains( int y, int x ) const
      {
        return 0 <= x && x < this->cols() && 0 <= y && y < this->rows();
      }

      /**
       * Get data pointer
       * @return data pointer
       */
      Tpixel * data() const {
        if (!_container) {
          return nullptr;
        }

        return _container->getDataPtr();
      }

      void fill(const T & value) {
        EigenMapType::fill(value);
      }

    protected :
      std::shared_ptr<BaseContainer<T>> _container = nullptr;
    };
  } // namespace image
} // namespace aliceVision
