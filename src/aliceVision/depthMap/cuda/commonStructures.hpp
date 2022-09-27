// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <cuda_runtime.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <assert.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <cstring>


#define THROW_ON_CUDA_ERROR(rcode, message) \
  if (rcode != cudaSuccess) {  \
    std::stringstream s; s << message << ": " << cudaGetErrorString(err);  \
    throw std::runtime_error(s.str());  \
  }


namespace aliceVision {
namespace depthMap {

#define MAX_CONSTANT_CAMERA_PARAM_SETS   10



/*********************************************************************************
 * forward declarations
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaDeviceMemoryPitched;

/*********************************************************************************
 * CudaSizeBase
 *********************************************************************************/

template <unsigned Dim> class CudaSizeBase
{
public:
  CudaSizeBase()
  {
    #pragma unroll
    for(int i = Dim; i--;)
      size[i] = 0;
  }
  inline size_t operator[](size_t i) const { return size[i]; }
  inline size_t &operator[](size_t i) { return size[i]; }
  inline CudaSizeBase operator+(const CudaSizeBase<Dim> &s) const {
    CudaSizeBase<Dim> r;

    #pragma unroll
    for(size_t i = Dim; i--;)
      r[i] = (*this)[i] + s[i];

    return r;
  }
  inline CudaSizeBase operator-(const CudaSizeBase<Dim> &s) const {
    CudaSizeBase<Dim> r;

    #pragma unroll
    for(size_t i = Dim; i--;)
      r[i] = (*this)[i] - s[i];

    return r;
  }

#if 0
  inline size_t getSize() const {
    size_t s = 1;

    #pragma unroll
    for(int i = Dim; i--;)
      s *= size[i];

    return s;
  }
#endif

protected:
  size_t size[Dim];
};

template <unsigned Dim>
bool operator==(const CudaSizeBase<Dim> &s1, const CudaSizeBase<Dim> &s2)
{
  for(int i = Dim; i--;)
    if(s1[i] != s2[i])
      return false;

  return true;
}

template <unsigned Dim>
bool operator!=(const CudaSizeBase<Dim> &s1, const CudaSizeBase<Dim> &s2)
{
  for(size_t i = Dim; i--;)
    if(s1[i] != s2[i])
      return true;

  return false;
}

/*********************************************************************************
 * CudaSize
 *********************************************************************************/

template <unsigned Dim>
class CudaSize: public CudaSizeBase<Dim>
{
    CudaSize() {}
};

template <>
class CudaSize<1>: public CudaSizeBase<1>
{
public:
    CudaSize() {}
    explicit CudaSize(size_t s0) { size[0] = s0; }
};

template <>
class CudaSize<2>: public CudaSizeBase<2>
{
public:
    CudaSize() {}
    CudaSize(size_t s0, size_t s1) { size[0] = s0; size[1] = s1; }

    inline size_t x() const { return size[0]; }
    inline size_t y() const { return size[1]; }
};

template <>
class CudaSize<3>: public CudaSizeBase<3>
{
public:
    CudaSize() {}
    CudaSize(size_t s0, size_t s1, size_t s2) { size[0] = s0; size[1] = s1; size[2] = s2; }

    inline size_t x() const { return size[0]; }
    inline size_t y() const { return size[1]; }
    inline size_t z() const { return size[2]; }
};

template <unsigned Dim>
CudaSize<Dim> operator/(const CudaSize<Dim> &lhs, const float &rhs) {
  if (rhs == 0)
    fprintf(stderr, "Division by zero!!\n");
  CudaSize<Dim> out = lhs;
  for(size_t i = 0; i < Dim; ++i)
    out[i] /= rhs;

  return out;
}

template <unsigned Dim>
CudaSize<Dim> operator-(const CudaSize<Dim> &lhs, const CudaSize<Dim> &rhs) {
  CudaSize<Dim> out = lhs;
  for(size_t i = Dim; i--;)
    out[i]-= rhs[i];
  return out;
}

/*********************************************************************************
 * CudaMemorySizeBase
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaMemorySizeBase
{
    CudaSize<Dim> _size;
    size_t        _pitch;
public:
    CudaMemorySizeBase( )
    { }

    explicit CudaMemorySizeBase( const CudaSize<Dim> &size )
        : _size( size )
        , _pitch( size[0] * sizeof(Type) )
    { }

    /* Initialize or change the contained _size value. As a
     * convenience for the many subclasses whose pitch is always
     * size[0] * sizeof(Type), true can be passed as second
     * parameter.
     */
    void setSize( const CudaSize<Dim> &size, bool computePitch )
    {
        _size = size;
        if( computePitch )
        {
            _pitch = size[0] * sizeof(Type);
        }
    }

    /* Return the Size struct.
     * It is best to use this as an opaque type.
     */
    inline const CudaSize<Dim>& getSize() const
    {
        return _size;
    }

    /* Return the byte size of dimension 0 with padding.
     * This function may return useless info until the
     * actual pitch has been initiated by the subclass.
     */
    inline size_t getPitch() const
    {
        return _pitch;
    }

    /* Return the number of bytes that are required by the data
     * contained in the subclass. The pitch may be different.
     * For many subclasses, getBytesUnpadded() == getBytesPadded()
     */
    inline size_t getBytesUnpadded() const
    {
        size_t prod = _size[0] * sizeof(Type);
        for( int i=1; i<Dim; i++ ) prod *= _size[i];
        return prod;
    }

    /* Return the number of bytes that are required to contain
     * the data of the subclass. This considers the pitch of the
     * first dimension.
     * For many subclasses, getBytesUnpadded() == getBytesPadded()
     */
    inline size_t getBytesPadded() const
    {
        size_t prod = _pitch;
        for( int i=1; i<Dim; i++ ) prod *= _size[i];
        return prod;
    }

    /* Returns the number of items that have been allocated,
     * ignoring padding.
     */
    inline size_t getUnitsTotal( ) const
    {
        size_t prod = _size[0];
        for( int i=1; i<Dim; i++ ) prod *= _size[i];
        return prod;
    }

    /* Returns the number of items of class Type that is contained
     * in the given dimension. For dimensions >= Dim, return 1.
     */
    inline size_t getUnitsInDim( int dim ) const
    {
        return ( dim < Dim ? _size[dim] : 1 );
    }

    /* For dim 0, return the pitch.
     * For all other dimensions, return the number of units in that dimension.
     */
    inline size_t getPaddedBytesInRow( ) const
    {
        return getPitch();
    }

    /* For dim 0, return the number of meaning bytes.
     * For all other dimensions, return the number of units in that dimension.
     */
    inline size_t getUnpaddedBytesInRow( ) const
    {
        return _size[0] * sizeof(Type);
    }

    /* Return the number of bytes that are required for an n-dimensional
     * slice of the subclass, always starting at dimension 0.
     *
     * Note that "dim" itself is included in the computation.
     */
    inline size_t getBytesPaddedUpToDim( int dim ) const
    {
        size_t prod = _pitch;
        for( int i=1; i<=dim; i++ ) prod *= getUnitsInDim(i);
        return prod;
    }

protected:
    /* Use to set the pitch when it has been returned by an allocation
     * function.
     */
    inline void setPitch( size_t pitch )
    {
        _pitch = pitch;
    }

    /* Allows the child class to pass the pitch to functions such as
     * CudaMallocPitched for initialization to the true value.
     */
    inline size_t& getPitchRef()
    {
        return _pitch;
    }
};

/*********************************************************************************
 * CudaHostMemoryHeap
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaHostMemoryHeap : public CudaMemorySizeBase<Type,Dim>
{
    Type* buffer = nullptr;
public:
    CudaHostMemoryHeap( )
        : buffer( nullptr )
    { }

    explicit CudaHostMemoryHeap(const CudaSize<Dim> &size )
        : buffer( nullptr )
    {
        allocate( size );
    }

    CudaHostMemoryHeap<Type,Dim>& operator=(const CudaHostMemoryHeap<Type,Dim>& rhs)
    {
        if( buffer != nullptr )
        {
            allocate( rhs.getSize() );
        }
        else if( this->getSize() != rhs.getSize() )
        {
            deallocate();
            allocate( rhs.getSize() );
        }

        memcpy(buffer, rhs.buffer, rhs.getBytesPadded() );
        return *this;
    }

    ~CudaHostMemoryHeap()
    {
        deallocate();
    }

    void initBuffer()
    {
        memset(buffer, 0, this->getBytesPadded());
    }

    // see below with copy() functions
    void copyFrom( const CudaDeviceMemoryPitched<Type, Dim>& src );

    inline Type *getBuffer()
    {
        return buffer;
    }
    inline const Type *getBuffer() const
    {
        return buffer;
    }
    inline Type& operator()(size_t x)
    {
        return buffer[x];
    }
    inline const Type& operator()(size_t x) const
    {
        return buffer[x];
    }
    inline Type& operator()(size_t x, size_t y)
    {
        return getRow(y)[x];
    }
    inline const Type& operator()(size_t x, size_t y) const
    {
        return getRow(y)[x];
    }

    inline unsigned char* getBytePtr()
    {
        return (unsigned char*)buffer;
    }
    inline const unsigned char* getBytePtr() const
    {
        return (unsigned char*)buffer;
    }

private:
    inline Type* getRow(size_t row)
    {
        unsigned char* ptr = getBytePtr();
        ptr += row * this->getPitch();
        return (Type*)ptr;
    }
    inline const Type* getRow(size_t row) const
    {
        const unsigned char* ptr = getBytePtr();
        ptr += row * this->getPitch();
        return (Type*)ptr;
    }

public:
    void allocate( const CudaSize<Dim> &size )
    {
        this->setSize( size, true );

        cudaError_t err = cudaMallocHost( &buffer, this->getBytesUnpadded() );

        THROW_ON_CUDA_ERROR( err, "Could not allocate pinned host memory in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err) );
    }

    void deallocate( )
    {
        if( buffer == nullptr ) return;
        cudaFreeHost(buffer);
        buffer = nullptr;
    }
};

/*********************************************************************************
 * CudaDeviceMemoryPitched
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaDeviceMemoryPitched : public CudaMemorySizeBase<Type,Dim>
{
    Type* buffer = nullptr;
public:
    CudaDeviceMemoryPitched( )
        : buffer( nullptr )
    { }

    explicit CudaDeviceMemoryPitched(const CudaSize<Dim> &size)
    {
        allocate( size );
    }

    explicit CudaDeviceMemoryPitched(const CudaHostMemoryHeap<Type, Dim> &rhs)
    {
        allocate( rhs.getSize() );
        copyFrom( rhs );
    }

    ~CudaDeviceMemoryPitched()
    {
        deallocate();
    }

    CudaDeviceMemoryPitched<Type,Dim>& operator=(const CudaDeviceMemoryPitched<Type,Dim> & rhs)
    {
        if( buffer == nullptr )
        {
            allocate( rhs.size );
        }
        else if( this->getSize() != rhs.getSize() )
        {
            deallocate( );
            allocate( rhs.getSize() );
        }
        copyFrom( rhs );
        return *this;
    }

    template<typename texturetype>
    void bindToTexture( texturetype& texref )
    {
        cudaError_t err = cudaBindTexture2D( 0, // offset
                                             texref,
                                             this->getBuffer(),
                                             cudaCreateChannelDesc<Type>(),
                                             this->getUnitsInDim(0),
                                             this->getUnitsInDim(1),
                                             this->getPitch() );
        THROW_ON_CUDA_ERROR( err, "Failed to bind texture reference to pitched memory, " << cudaGetErrorString( err ) );
    }

    // see below with copy() functions
    void copyFrom( const CudaHostMemoryHeap<Type, Dim>& src, cudaStream_t stream = 0 );
    void copyFrom( const Type* src, size_t sx, size_t sy );
    void copyFrom( const CudaDeviceMemoryPitched<Type, Dim>& src );

    void copyTo( Type* dst, size_t sx, size_t sy ) const;

    Type* getBuffer()
    {
        return buffer;
    }

    const Type* getBuffer() const
    {
        return buffer;
    }

    Type& operator()(size_t x)
    {
        return buffer[x];
    }

    Type& operator()(size_t x, size_t y)
    {
        Type* row = getRow( y );
        return row[x];
    }

    inline unsigned char* getBytePtr()
    {
        return (unsigned char*)buffer;
    }

    inline const unsigned char* getBytePtr() const
    {
        return (unsigned char*)buffer;
    }

    inline Type* getRow( size_t row )
    {
        unsigned char* ptr = getBytePtr();
        ptr += row * this->getPitch();
        return (Type*)ptr;
    }

    void allocate( const CudaSize<Dim>& size )
    {
        this->setSize( size, false );

        if(Dim == 2)
        {
            cudaError_t err = cudaMallocPitch<Type>(&buffer,
                                                    &this->getPitchRef(),
                                                    this->getUnpaddedBytesInRow(),
                                                    this->getUnitsInDim(1) );
            if( err != cudaSuccess )
            {
                int devid;
                cudaGetDevice( &devid );
                std::stringstream ss;
                ss << "Could not allocate pitched device memory.\n"
                   << "Device " << devid << " alloc " << this->getBytesUnpadded() << " bytes failed in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err);
                throw std::runtime_error(ss.str());
            }
        }
        else if(Dim == 3)
        {
            cudaExtent extent;
            extent.width  = this->getUnpaddedBytesInRow();
            extent.height = this->getUnitsInDim(1);
            extent.depth  = this->getUnitsInDim(2);
            cudaPitchedPtr pitchDevPtr;
            cudaError_t err = cudaMalloc3D(&pitchDevPtr, extent);
            if( err != cudaSuccess )
            {
                int devid;
                cudaGetDevice( &devid );
                size_t bytes = this->getBytesUnpadded();
                size_t sx    = this->getUnitsInDim(0);
                size_t sy    = this->getUnitsInDim(1);
                size_t sz    = this->getUnitsInDim(2);
                std::stringstream ss;
                ss << "Could not allocate 3D device memory.\n"
                   << "Device " << devid << " alloc "
                                    << sx << "x" << sy << "x" << sz << "x" << sizeof(Type) << " = "
                                    << bytes << " bytes ("
                << (int)(bytes/1024.0f/1024.0f) << " MB) failed in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err);
                throw std::runtime_error(ss.str());
            }

            buffer = (Type*)pitchDevPtr.ptr;
            this->setPitch( pitchDevPtr.pitch );
        }
        else
        {
            throw std::runtime_error("CudaDeviceMemoryPitched does not support " + std::to_string(Dim) + " dimensions.");
        }
    }

    void deallocate()
    {
        if( buffer == nullptr ) return;

        cudaError_t err = cudaFree(buffer);
        if( err != cudaSuccess )
        {
            std::stringstream ss;
            ss << "CudaDeviceMemoryPitched: Device free failed, " << cudaGetErrorString(err);
            throw std::runtime_error(ss.str());
        }

        buffer = nullptr;
    }
};

/*********************************************************************************
 * CudaDeviceMemory
 *********************************************************************************/

template <class Type> class CudaDeviceMemory : public CudaMemorySizeBase<Type,1>
{
    Type* buffer = nullptr;
public:
    explicit CudaDeviceMemory(const size_t size)
    {
        allocate( size );
    }

    explicit inline CudaDeviceMemory(const CudaHostMemoryHeap<Type,1> &rhs)
    {
        allocate( rhs.getSize() );
        copy(*this, rhs);
    }

    // constructor with synchronous copy
    CudaDeviceMemory(const Type* inbuf, const size_t size )
    {
        allocate( size );
        copyFrom( inbuf, size );
    }

    // constructor with asynchronous copy
    CudaDeviceMemory(const Type* inbuf, const size_t size, cudaStream_t stream )
    {
        allocate( size );
        copyFrom( inbuf, size, stream );
    }

    ~CudaDeviceMemory()
    {
        deallocate( );
    }

    CudaDeviceMemory<Type> & operator=(const CudaDeviceMemory<Type> & rhs)
    {
        if( buffer == nullptr )
        {
            allocate( rhs.getSize() );
        }
        else if( this->getSize() != rhs.getSize() )
        {
            deallocate( );
            allocate( rhs.getSize() );
        }
        copy(*this, rhs);
        return *this;
    }

    Type *getBuffer()
    {
        return buffer;
    }
    const Type *getBuffer() const
    {
        return buffer;
    }

    unsigned char* getBytePtr()
    {
        return (unsigned char*)buffer;
    }
    const unsigned char* getBytePtr() const
    {
        return (unsigned char*)buffer;
    }

    void allocate( const CudaSize<1> &size )
    {
        this->setSize( size, true );

        cudaError_t err = cudaMalloc(&buffer, this->getBytesUnpadded() );

        THROW_ON_CUDA_ERROR( err, "Could not allocate pinned host memory in " << __FILE__ << ":" << __LINE__ << ", " << cudaGetErrorString(err) );
    }
    void allocate( const size_t size )
    {
        allocate(CudaSize<1>(size));
    }

    void deallocate()
    {
        if( buffer == nullptr ) return;

        cudaError_t err = cudaFree(buffer);
        if( err != cudaSuccess )
        {
            std::stringstream ss;
            ss << "CudaDeviceMemory: Device free failed, " << cudaGetErrorString(err);
            throw std::runtime_error(ss.str());
        }

        buffer = nullptr;
    }

    void copyFrom( const Type* inbuf, const size_t num )
    {
        cudaMemcpyKind kind = cudaMemcpyHostToDevice;
        cudaError_t err = cudaMemcpy( buffer,
                                      inbuf,
                                      num * sizeof(Type),
                                      kind );

        THROW_ON_CUDA_ERROR( err, "Failed to copy from flat host buffer to CudaDeviceMemory in " << __FILE__ << ":" << __LINE__ << ": " << cudaGetErrorString(err) );
    }

    void copyFrom( const Type* inbuf, const size_t num, cudaStream_t stream )
    {
        cudaMemcpyKind kind = cudaMemcpyHostToDevice;
        cudaError_t err = cudaMemcpyAsync( buffer,
                                           inbuf,
                                           num * sizeof(Type),
                                           kind,
                                           stream );

        THROW_ON_CUDA_ERROR( err, "Failed to copy from flat host buffer to CudaDeviceMemory in " << __FILE__ << ":" << __LINE__ << ": " << cudaGetErrorString(err) );
    }
};

/*********************************************************************************
 * CudaArray
 *********************************************************************************/

template <class Type, unsigned Dim> class CudaArray : public CudaMemorySizeBase<Type,Dim>
{
    cudaArray *array;
public:
    explicit CudaArray(const CudaSize<Dim> &size)
    {
        allocate( size );
    }

    explicit inline CudaArray(const CudaDeviceMemoryPitched<Type, Dim> &rhs)
    {
        allocate( rhs.getSize() );
        copy(*this, rhs);
    }

    explicit inline CudaArray(const CudaHostMemoryHeap<Type, Dim> &rhs)
    {
        allocate( rhs.getSize() );
        copy(*this, rhs);
    }

    virtual ~CudaArray()
    {
        cudaFreeArray(array);
    }

    cudaArray *getArray()
    {
        return array;
    }

    const cudaArray *getArray() const
    {
        return array;
    }

    void allocate( const CudaSize<Dim> &size )
    {
        this->setSize( size, true );

        cudaChannelFormatDesc channelDesc = cudaCreateChannelDesc<Type>();
        if(Dim == 1)
        {
            cudaError_t err = cudaMallocArray(&array,
                                              &channelDesc,
                                              this->getUnitsInDim(0),
                                              1,
                                              cudaArraySurfaceLoadStore);
            THROW_ON_CUDA_ERROR(err, "Device alloc 1D array failed");
        }
        else if(Dim == 2)
        {
            cudaError_t err = cudaMallocArray(&array,
                                              &channelDesc,
                                              this->getUnitsInDim(0),
                                              this->getUnitsInDim(1),
                                              cudaArraySurfaceLoadStore);
            THROW_ON_CUDA_ERROR(err, "Device alloc 2D array failed");
        }
        else
        {
            cudaExtent extent;
            extent.width  = this->getUnitsInDim(0);
            extent.height = this->getUnitsInDim(1);
            extent.depth  = this->getUnitsInDim(2);
            for(unsigned i = 3; i < Dim; ++i)
                extent.depth *= this->getUnitsInDim(i);
            cudaError_t err = cudaMalloc3DArray(&array, &channelDesc, extent);
            THROW_ON_CUDA_ERROR(err, "Device alloc 3D array failed");
        }
    }
};

/*********************************************************************************
 * copyFrom member functions
 *********************************************************************************/

template<class Type, unsigned Dim>
void CudaDeviceMemoryPitched<Type, Dim>::copyFrom( const CudaHostMemoryHeap<Type, Dim>& src, cudaStream_t stream )
{
    const cudaMemcpyKind kind = cudaMemcpyHostToDevice;
    cudaError_t err;
    if(Dim == 1)
    {
        if( stream == 0 )
            err = cudaMemcpy( this->getBytePtr(),
                              src.getBytePtr(),
                              src.getBytesUnpadded(),
                              kind );
        else
            err = cudaMemcpyAsync( this->getBytePtr(),
                                   src.getBytePtr(),
                                   src.getBytesUnpadded(),
                                   kind,
                                   stream );
        THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
    else if(Dim >= 2)
    {
        size_t number_of_rows = 1;
        for( int i=1; i<Dim; i++ ) number_of_rows *= src.getUnitsInDim(i);

        if( stream == 0 )
            err = cudaMemcpy2D( this->getBytePtr(),
                                this->getPitch(),
                                src.getBytePtr(),
                                src.getPitch(),
                                src.getUnpaddedBytesInRow(),
                                number_of_rows,
                                kind );
        else
            err = cudaMemcpy2DAsync( this->getBytePtr(),
                                     this->getPitch(),
                                     src.getBytePtr(),
                                     src.getPitch(),
                                     src.getUnpaddedBytesInRow(),
                                     number_of_rows,
                                     kind,
                                     stream );
        THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
}

template<class Type, unsigned Dim>
void CudaDeviceMemoryPitched<Type, Dim>::copyFrom( const Type* src, size_t sx, size_t sy )
{
    if(Dim == 2)
    {
        const size_t src_pitch  = sx * sizeof(Type);
        const size_t src_width  = sx * sizeof(Type);
        const size_t src_height = sy;
        cudaError_t err = cudaMemcpy2D(this->getBytePtr(),
                                       this->getPitch(),
                                       src,
                                       src_pitch,
                                       src_width,
                                       src_height,
                                       cudaMemcpyHostToDevice);

        THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
}

template<class Type, unsigned Dim>
void CudaDeviceMemoryPitched<Type, Dim>::copyFrom(const CudaDeviceMemoryPitched<Type, Dim>& src)
{
    const cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
    if(Dim == 1)
    {
        cudaError_t err = cudaMemcpy(this->getBytePtr(),
                                     src.getBytePtr(),
                                     src.getUnpaddedBytesInRow(),
                                     kind);
        THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
    else if(Dim >= 2)
    {
        size_t number_of_rows = 1;
        for( int i=1; i<Dim; i++ ) number_of_rows *= src.getUnitsInDim(i);

        cudaError_t err = cudaMemcpy2D( this->getBytePtr(),
                                        this->getPitch(),
                                        src.getBytePtr(),
                                        src.getPitch(),
                                        src.getUnpaddedBytesInRow(),
                                        number_of_rows,
                                        kind);
        THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
}

template<class Type, unsigned Dim>
void CudaHostMemoryHeap<Type, Dim>::copyFrom( const CudaDeviceMemoryPitched<Type, Dim>& src )
{
    const cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
    if(Dim == 1)
    {
        cudaError_t err = cudaMemcpy( this->getBytePtr(),
                                      src.getBytePtr(),
                                      this->getUnpaddedBytesInRow(),
                                      kind);
        THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
    else if(Dim >= 2)
    {
        size_t number_of_rows = 1;
        for( int i=1; i<Dim; i++ ) number_of_rows *= this->getUnitsInDim(i);
    
        cudaError_t err = cudaMemcpy2D( this->getBytePtr(),
                                        this->getPitch(),
                                        src.getBytePtr(),
                                        src.getPitch(),
                                        this->getUnpaddedBytesInRow(),
                                        number_of_rows,
                                        kind);

        THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
}

template<class Type, unsigned Dim>
void CudaDeviceMemoryPitched<Type, Dim>::copyTo( Type* dst, size_t sx, size_t sy ) const
{
    if(Dim == 2)
    {
        const size_t dst_pitch  = sx * sizeof(Type);
        const size_t dst_width  = sx * sizeof(Type);
        const size_t dst_height = sy;
        cudaError_t err = cudaMemcpy2D(dst,
                                       dst_pitch,
                                       this->getBytePtr(),
                                       this->getPitch(),
                                       dst_width,
                                       dst_height,
                                       cudaMemcpyDeviceToHost);

        THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
}

/*********************************************************************************
 * copy functions
 *********************************************************************************/

template<class Type, unsigned Dim> void copy(CudaHostMemoryHeap<Type, Dim>& dst, const CudaDeviceMemoryPitched<Type, Dim>& src)
{
    dst.copyFrom( src );
}

template<class Type> void copy(CudaHostMemoryHeap<Type,1>& _dst, const CudaDeviceMemory<Type>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
  cudaError_t err = cudaMemcpy(_dst.getBytePtr(),
                               _src.getBytePtr(),
                               _dst.getUnpaddedBytesInRow(),
                               kind);
  THROW_ON_CUDA_ERROR(err, "Failed to copy from CudaHostMemoryHeap to CudaDeviceMemory");
}

template<class Type, unsigned Dim> void copy(CudaHostMemoryHeap<Type, Dim>& _dst, const CudaArray<Type, Dim>& _src)
{
  cudaMemcpyKind kind = cudaMemcpyDeviceToHost;
  if(Dim == 1) {
    cudaError_t err = cudaMemcpyFromArray(_dst.getBytePtr(),
                                          _src.getArray(),
                                          0, 0,
                                          _dst.getUnpaddedBytesInRow(),
                                          kind);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
  else if(Dim == 2)
  {
    cudaError_t err = cudaMemcpy2DFromArray(_dst.getBytePtr(),
                                            _dst.getPitch(),
                                            _src.getArray(),
                                            0,
                                            0,
                                            _dst.getUnpaddedBytesInRow(),
                                            _dst.getUnitsInDim(1),
                                            kind);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
  else if(Dim >= 3)
  {
    size_t number_of_rows = 1;
    for( int i=1; i<Dim; i++ ) number_of_rows *= _dst.getUnitsInDim(i);

    cudaMemcpy3DParms p = { 0 };
    p.srcArray = const_cast<cudaArray *>(_src.getArray());

    p.dstPtr.ptr = (void *)_dst.getBytePtr();
    p.dstPtr.pitch  = _dst.getPitch();
    p.dstPtr.xsize  = _dst.getUnitsInDim(0);
    p.dstPtr.ysize  = number_of_rows;

    p.extent.width  = _dst.getUnitsInDim(0);
    p.extent.height = _dst.getUnitsInDim(1);
    p.extent.depth  = _dst.getUnitsInDim(2);
    for(unsigned i = 3; i < Dim; ++i)
      p.extent.depth *= _src.getUnitsInDim(i);
    p.kind = kind;
    cudaError_t err = cudaMemcpy3D(&p);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaHostMemoryHeap<Type, Dim>& _src)
{
    _dst.copyFrom( _src );
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
    _dst.copyFrom( _src );
}

template<class Type> void copy(CudaDeviceMemory<Type>& _dst, const CudaHostMemoryHeap<Type,1>& _src)
{
  const cudaMemcpyKind kind = cudaMemcpyHostToDevice;
  cudaError_t err = cudaMemcpy(_dst.getBytePtr(),
                               _src.getBytePtr(),
                               _src.getUnpaddedBytesInRow(),
                               kind);
  THROW_ON_CUDA_ERROR(err, "Failed to copy from CudaHostMemoryHeap to CudaDeviceMemory");
}

template<class Type> void copy(CudaDeviceMemory<Type>& _dst, const Type* buffer, const size_t numelems )
{
    _dst.copyFrom( buffer, numelems );
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const CudaArray<Type, Dim>& _src)
{
  const cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
  if(Dim == 1) {
    cudaError_t err = cudaMemcpyFromArray(_dst.getBytePtr(),
                                          _src.getArray(),
                                          0, 0,
                                          _src.getUnpaddedBytesInRow(),
                                          kind);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
  else if(Dim == 2)
  {
    cudaError_t err = cudaMemcpy2DFromArray(_dst.getBytePtr(),
                                            _dst.getPitch(),
                                            _src.getArray(),
                                            0,
                                            0,
                                            _src.getUnpaddedBytesInRow(),
                                            _src.getUnitsInDim(1),
                                            kind);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
  else if(Dim == 3) {
    cudaMemcpy3DParms p = { 0 };
    p.srcArray = const_cast<cudaArray *>(_src.getArray());
    p.srcPtr.pitch = _src.getPitch();
    p.srcPtr.xsize = _src.getSize()[0];
    p.srcPtr.ysize = _src.getSize()[1];
    p.dstPtr.ptr = (void *)_dst.getBytePtr();
    p.dstPtr.pitch = _dst.getPitch();
    p.dstPtr.xsize = _dst.getSize()[0];
    p.dstPtr.ysize = _dst.getSize()[1];
    p.extent.width = _src.getSize()[0];
    p.extent.height = _src.getSize()[1];
    p.extent.depth = _src.getSize()[2];
    for(unsigned i = 3; i < Dim; ++i)
      p.extent.depth *= _src.getSize()[i];
    p.kind = kind;
    cudaError_t err = cudaMemcpy3D(&p);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
}

template<class Type, unsigned Dim> void copy(CudaArray<Type, Dim>& _dst, const CudaHostMemoryHeap<Type, Dim>& _src)
{
  const cudaMemcpyKind kind = cudaMemcpyHostToDevice;
  if(Dim == 1) {
    cudaError_t err = cudaMemcpyToArray(_dst.getArray(),
                                        0, 0,
                                        _src.getBytePtr(),
                                          _src.getUnpaddedBytesInRow(),
                                        kind);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
  else if(Dim == 2) {
    cudaError_t err = cudaMemcpy2DToArray(_dst.getArray(),
                                          0,
                                          0,
                                          _src.getBytePtr(),
                                          _src.getPitch(),
                                          _src.getUnpaddedBytesInRow(),
                                          _src.getUnitsInDim(1),
                                          kind);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
  else if(Dim >= 3) {
    size_t number_of_rows = 1;
    for( int i=1; i<Dim; i++ ) number_of_rows *= _src.getUnitsInDim(i);

    cudaMemcpy3DParms p = { 0 };
    p.srcPtr.ptr = (void *)_src.getBytePtr();
    p.srcPtr.pitch = _src.getPitch();
    p.srcPtr.xsize = _src.getUnitsInDim(0);
    p.srcPtr.ysize = number_of_rows;

    p.dstArray = _dst.getArray();

    p.extent.width  = _src.getUnitsInDim(0);
    p.extent.height = _src.getUnitsInDim(1);
    p.extent.depth  = _src.getUnitsInDim(2);
    for(unsigned i = 3; i < Dim; ++i)
      p.extent.depth *= _src.getUnitsInDim(i);
    p.kind = kind;
    cudaError_t err = cudaMemcpy3D(&p);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
}

template<class Type, unsigned Dim> void copy(CudaArray<Type, Dim>& _dst, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  const cudaMemcpyKind kind = cudaMemcpyDeviceToDevice;
  if(Dim == 1) {
    cudaError_t err = cudaMemcpyToArray(_dst.getArray(),
                                        0, 0,
                                        _src.getBytePtr(),
                                        _src.getUnpaddedBytesInRow(),
                                        kind);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
  else if(Dim == 2) {
    cudaError_t err = cudaMemcpy2DToArray(_dst.getArray(),
                                          0,
                                          0,
                                          _src.getBytePtr(),
                                          _src.getPitch(),
                                          _src.getUnpaddedBytesInRow(),
                                          _src.getUnitsInDim(1),
                                          kind);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
  else if(Dim >= 3) {
    size_t number_of_rows = 1;
    for( int i=1; i<Dim; i++ ) number_of_rows *= _src.getUnitsInDim(i);

    cudaMemcpy3DParms p = { 0 };
    p.srcPtr.ptr = (void *)_src.getBytePtr();
    p.srcPtr.pitch  = _src.getPitch();
    p.srcPtr.xsize  = _src.getUnitsInDim(0);
    p.srcPtr.ysize  = number_of_rows;

    p.dstArray = _dst.getArray();

    /* From the documentation:
     * The extent field defines the dimensions of the transferred area in elements.
     * If a CUDA array is participating in the copy, the extent is defined in terms
     * of that array's elements. If no CUDA array is participating in the copy then
     * the extents are defined in elements of unsigned char.
     */
    p.extent.width  = _src.getUnitsInDim(0);
    p.extent.height = _src.getUnitsInDim(1);
    p.extent.depth  = _src.getUnitsInDim(2);
    for(unsigned i = 3; i < Dim; ++i)
      p.extent.depth *= _src.getUnitsInDim(i);
    p.kind = kind;
    cudaError_t err = cudaMemcpy3D(&p);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
}

template<class Type, unsigned Dim> void copy(Type* _dst, size_t sx, size_t sy, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
  if(Dim == 2) {
    cudaError_t err = cudaMemcpy2D(_dst,
                                   sx * sizeof (Type),
                                   _src.getBytePtr(),
                                   _src.getPitch(),
                                   sx * sizeof (Type),
                                   sy,
                                   cudaMemcpyDeviceToHost);
    THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
  }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& dst, const Type* src, size_t sx, size_t sy)
{
    dst.copyFrom( src, sx, sy );
}

template<class Type, unsigned Dim> void copy(Type* _dst, size_t sx, size_t sy, size_t sz, const CudaDeviceMemoryPitched<Type, Dim>& _src)
{
    if(Dim >= 3)
    {
      cudaError_t err = cudaMemcpy2D( _dst,
                                      sx * sizeof (Type),
                                      _src.getBytePtr(),
                                      _src.getPitch(),
                                      sx * sizeof (Type),
                                      sy * sz,
                                      cudaMemcpyDeviceToHost);
      THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
}

template<class Type, unsigned Dim> void copy(CudaDeviceMemoryPitched<Type, Dim>& _dst, const Type* _src, size_t sx, size_t sy, size_t sz)
{
    if(Dim >= 3)
    {
      const size_t src_pitch      = sx * sizeof(Type);
      const size_t width_in_bytes = sx * sizeof(Type);
      const size_t height         = sy * sz;
      cudaError_t err = cudaMemcpy2D( _dst.getBytePtr(),
                                      _dst.getPitch(),
                                      _src,
                                      src_pitch,
                                      width_in_bytes,
                                      height,
                                      cudaMemcpyHostToDevice);
      THROW_ON_CUDA_ERROR(err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ")");
    }
}

template<class Type> void copy2D( Type* dst, size_t sx, size_t sy,
                                  Type* src, size_t src_pitch,
                                  cudaStream_t stream )
{
    const size_t dst_pitch      = sx * sizeof(Type);
    const size_t width_in_bytes = sx * sizeof(Type);
    const size_t height         = sy;
    cudaError_t err = cudaMemcpy2DAsync( dst,
                                         dst_pitch,
                                         src,
                                         src_pitch,
                                         width_in_bytes,
                                         height,
                                         cudaMemcpyDeviceToHost,
                                         stream );
    THROW_ON_CUDA_ERROR( err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ", " << cudaGetErrorString(err) << ")" );
}

template<class Type> void copy2D( Type* dst, size_t sx, size_t sy,
                                  Type* src, size_t src_pitch )
{
    const size_t dst_pitch      = sx * sizeof(Type);
    const size_t width_in_bytes = sx * sizeof(Type);
    const size_t height         = sy;
    cudaError_t err = cudaMemcpy2D( dst,
                                    dst_pitch,
                                    src,
                                    src_pitch,
                                    width_in_bytes,
                                    height,
                                    cudaMemcpyDeviceToHost );
    THROW_ON_CUDA_ERROR( err, "Failed to copy (" << __FILE__ << " " << __LINE__ << ", " << cudaGetErrorString(err) << ")" );
}

struct CameraStructBase
{
    float  P[12];
    float  iP[9];
    float  R[9];
    float  iR[9];
    float  K[9];
    float  iK[9];
    float3 C;
    float3 XVect;
    float3 YVect;
    float3 ZVect;
};

// #define ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
using CudaColorBaseType = unsigned char;
using CudaRGBA = uchar4;

#else
using CudaColorBaseType = float;
using CudaRGBA = float4;

#endif


struct TexturedArray
{
    CudaDeviceMemoryPitched<CudaRGBA, 2>* arr = nullptr;
    cudaTextureObject_t tex;
};

struct CamCacheIdx
{
    int i = 0;

    CamCacheIdx() = default;
    explicit CamCacheIdx( int val ) : i(val) { }
};

typedef std::vector<TexturedArray> Pyramid;

struct CameraStruct
{
    CamCacheIdx  param_dev;
    Pyramid*     pyramid = nullptr;
    int          camId = -1;
    cudaStream_t stream = 0; // allow async work on cameras used in parallel
};

/**
* @notes: use normalized coordinates
*/
template <class Type>
struct CudaTexture
{
    cudaTextureObject_t textureObj = 0;
    CudaTexture(CudaDeviceMemoryPitched<Type, 2>& buffer_dmp)
    {
        cudaTextureDesc  tex_desc;
        memset(&tex_desc, 0, sizeof(cudaTextureDesc));
        tex_desc.normalizedCoords = 0; // addressed (x,y) in [width,height]
        tex_desc.addressMode[0] = cudaAddressModeClamp;
        tex_desc.addressMode[1] = cudaAddressModeClamp;
        tex_desc.addressMode[2] = cudaAddressModeClamp;
        tex_desc.readMode = cudaReadModeElementType;
        tex_desc.filterMode = cudaFilterModePoint;

        cudaResourceDesc res_desc;
        res_desc.resType = cudaResourceTypePitch2D;
        res_desc.res.pitch2D.desc = cudaCreateChannelDesc<Type>();
        res_desc.res.pitch2D.devPtr = buffer_dmp.getBuffer();
        res_desc.res.pitch2D.width = buffer_dmp.getSize()[0];
        res_desc.res.pitch2D.height = buffer_dmp.getSize()[1];
        res_desc.res.pitch2D.pitchInBytes = buffer_dmp.getPitch();

        // create texture object: we only have to do this once!
        cudaCreateTextureObject(&textureObj, &res_desc, &tex_desc, NULL);
    }
    ~CudaTexture()
    {
        cudaDestroyTextureObject(textureObj);
    }
};

} // namespace depthMap
} // namespace aliceVision

