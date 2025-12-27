// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// #define ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION

#include <aliceVision/system/Logger.hpp>

#include <sycl/sycl.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <assert.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <cstring>

namespace aliceVision {
namespace depthMap {

#ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
using SyclColorBaseType = unsigned char;
#else
    #ifdef ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
using SyclColorBaseType = sycl::half;
    #else
using SyclColorBaseType = float;
    #endif  // ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#endif      // ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
using SyclRGBA = sycl::vec<SyclColorBaseType, 4>;

/*********************************************************************************
 * forward declarations
 *********************************************************************************/

template<class Type, unsigned Dim>
class SyclDeviceMemoryPitched;

/*********************************************************************************
 * SyclSizeBase
 *********************************************************************************/

template<unsigned Dim>
class SyclSizeBase
{
  public:
    SyclSizeBase()
    {
#pragma unroll
        for (int i = Dim; i--;)
            size[i] = 0;
    }
    inline size_t operator[](size_t i) const { return size[i]; }
    inline size_t& operator[](size_t i) { return size[i]; }
    inline SyclSizeBase operator+(const SyclSizeBase<Dim>& s) const
    {
        SyclSizeBase<Dim> r;

#pragma unroll
        for (size_t i = Dim; i--;)
            r[i] = (*this)[i] + s[i];

        return r;
    }
    inline SyclSizeBase operator-(const SyclSizeBase<Dim>& s) const
    {
        SyclSizeBase<Dim> r;

#pragma unroll
        for (size_t i = Dim; i--;)
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

template<unsigned Dim>
bool operator==(const SyclSizeBase<Dim>& s1, const SyclSizeBase<Dim>& s2)
{
    for (int i = Dim; i--;)
        if (s1[i] != s2[i])
            return false;

    return true;
}

template<unsigned Dim>
bool operator!=(const SyclSizeBase<Dim>& s1, const SyclSizeBase<Dim>& s2)
{
    for (size_t i = Dim; i--;)
        if (s1[i] != s2[i])
            return true;

    return false;
}

/*********************************************************************************
 * SyclSize
 *********************************************************************************/

template<unsigned Dim>
class SyclSize : public SyclSizeBase<Dim>
{
    SyclSize() {}
};

template<>
class SyclSize<1> : public SyclSizeBase<1>
{
  public:
    SyclSize() {}
    explicit SyclSize(size_t s0) { size[0] = s0; }
};

template<>
class SyclSize<2> : public SyclSizeBase<2>
{
  public:
    SyclSize() {}
    SyclSize(size_t s0, size_t s1)
    {
        size[0] = s0;
        size[1] = s1;
    }

    inline size_t x() const { return size[0]; }
    inline size_t y() const { return size[1]; }
};

template<>
class SyclSize<3> : public SyclSizeBase<3>
{
  public:
    SyclSize() {}
    SyclSize(size_t s0, size_t s1, size_t s2)
    {
        size[0] = s0;
        size[1] = s1;
        size[2] = s2;
    }

    inline size_t x() const { return size[0]; }
    inline size_t y() const { return size[1]; }
    inline size_t z() const { return size[2]; }
};

template<unsigned Dim>
SyclSize<Dim> operator/(const SyclSize<Dim>& lhs, const float& rhs)
{
    if (rhs == 0)
        fprintf(stderr, "Division by zero!!\n");
    SyclSize<Dim> out = lhs;
    for (size_t i = 0; i < Dim; ++i)
        out[i] /= rhs;

    return out;
}

template<unsigned Dim>
SyclSize<Dim> operator-(const SyclSize<Dim>& lhs, const SyclSize<Dim>& rhs)
{
    SyclSize<Dim> out = lhs;
    for (size_t i = Dim; i--;)
        out[i] -= rhs[i];
    return out;
}

/*********************************************************************************
 * SyclMemorySizeBase
 *********************************************************************************/

template<class Type, unsigned Dim>
class SyclMemorySizeBase
{
    SyclSize<Dim> _size;
    size_t _pitch;

  public:
    SyclMemorySizeBase() {}

    explicit SyclMemorySizeBase(const SyclSize<Dim>& size)
      : _size(size),
        _pitch(size[0] * sizeof(Type))
    {}

    /* Initialize or change the contained _size value. As a
     * convenience for the many subclasses whose pitch is always
     * size[0] * sizeof(Type), true can be passed as second
     * parameter.
     */
    void setSize(const SyclSize<Dim>& size, bool computePitch)
    {
        _size = size;
        if (computePitch)
        {
            _pitch = size[0] * sizeof(Type);
        }
    }

    /* Return the Size struct.
     * It is best to use this as an opaque type.
     */
    inline const SyclSize<Dim>& getSize() const { return _size; }

    /* Return the byte size of dimension 0 with padding.
     * This function may return useless info until the
     * actual pitch has been initiated by the subclass.
     */
    inline size_t getPitch() const { return _pitch; }

    /* Return the number of bytes that are required by the data
     * contained in the subclass. The pitch may be different.
     * For many subclasses, getBytesUnpadded() == getBytesPadded()
     */
    inline size_t getBytesUnpadded() const
    {
        size_t prod = _size[0] * sizeof(Type);
        for (int i = 1; i < Dim; i++)
            prod *= _size[i];
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
        for (int i = 1; i < Dim; i++)
            prod *= _size[i];
        return prod;
    }

    /* Returns the number of items that have been allocated,
     * ignoring padding.
     */
    inline size_t getUnitsTotal() const
    {
        size_t prod = _size[0];
        for (int i = 1; i < Dim; i++)
            prod *= _size[i];
        return prod;
    }

    /* Returns the number of items of class Type that is contained
     * in the given dimension. For dimensions >= Dim, return 1.
     */
    inline size_t getUnitsInDim(int dim) const { return (dim < Dim ? _size[dim] : 1); }

    /* For dim 0, return the pitch.
     * For all other dimensions, return the number of units in that dimension.
     */
    inline size_t getPaddedBytesInRow() const { return getPitch(); }

    /* For dim 0, return the number of meaning bytes.
     * For all other dimensions, return the number of units in that dimension.
     */
    inline size_t getUnpaddedBytesInRow() const { return _size[0] * sizeof(Type); }

    /* Return the number of bytes that are required for an n-dimensional
     * slice of the subclass, always starting at dimension 0.
     *
     * Note that "dim" itself is included in the computation.
     */
    inline size_t getBytesPaddedUpToDim(int dim) const
    {
        size_t prod = _pitch;
        for (int i = 1; i <= dim; i++)
            prod *= getUnitsInDim(i);
        return prod;
    }

  protected:
    /* Use to set the pitch when it has been returned by an allocation
     * function.
     */
    inline void setPitch(size_t pitch) { _pitch = pitch; }

    /* Allows the child class to pass the pitch to functions such as
     * SyclMallocPitched for initialization to the true value.
     */
    inline size_t& getPitchRef() { return _pitch; }
};

/*********************************************************************************
 * SyclHostMemoryHeap
 *********************************************************************************/

template<class Type, unsigned Dim>
class SyclHostMemoryHeap : public SyclMemorySizeBase<Type, Dim>
{
    Type* buffer = nullptr;
    sycl::queue queue;

  public:
    SyclHostMemoryHeap()
      : buffer(nullptr)
    {}

    explicit SyclHostMemoryHeap(const SyclSize<Dim>& size, sycl::queue queue)
        : buffer(nullptr),
          queue(queue)
    {
        allocate(size);
    }

    SyclHostMemoryHeap<Type, Dim>& operator=(const SyclHostMemoryHeap<Type, Dim>& rhs)
    {
        if (buffer != nullptr)
        {
            allocate(rhs.getSize());
        }
        else if (this->getSize() != rhs.getSize())
        {
            deallocate();
            allocate(rhs.getSize());
        }

        memcpy(buffer, rhs.buffer, rhs.getBytesPadded());
        return *this;
    }

    ~SyclHostMemoryHeap() { deallocate(); }

    void initBuffer() { memset(buffer, 0, this->getBytesPadded()); }

    // see below with copy() functions
    sycl::event copyFrom(const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::queue queue, sycl::event prerequisite);

    inline Type* getBuffer() { return buffer; }
    inline const Type* getBuffer() const { return buffer; }
    inline Type& operator()(size_t x) { return buffer[x]; }
    inline const Type& operator()(size_t x) const { return buffer[x]; }
    inline Type& operator()(size_t x, size_t y) { return getRow(y)[x]; }
    inline const Type& operator()(size_t x, size_t y) const { return getRow(y)[x]; }

    inline unsigned char* getBytePtr() { return (unsigned char*)buffer; }
    inline const unsigned char* getBytePtr() const { return (unsigned char*)buffer; }

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
    bool allocate(const SyclSize<Dim>& size)
    {
        this->setSize(size, true);

        buffer = sycl::malloc_host<Type>(this->getUnitsTotal(), queue);

        // this is our only way of checking if we have enough memory to allocate
        return !(buffer==nullptr);
    }

    void deallocate()
    {
        if (buffer == nullptr)
            return;
        sycl::free(buffer, queue);
        buffer = nullptr;
    }
};

/*********************************************************************************
 * SyclDeviceMemoryPitched
 *********************************************************************************/

template<class Type, unsigned Dim>
class SyclDeviceMemoryPitched : public SyclMemorySizeBase<Type, Dim>
{
    Type* buffer = nullptr;
    sycl::queue queue;

  public:
    SyclDeviceMemoryPitched(sycl::queue queue)
        : buffer(nullptr),
          queue(queue)
    {}

    explicit SyclDeviceMemoryPitched(const SyclSize<Dim>& size, sycl::queue queue) : queue(queue) { allocate(size); }

    explicit SyclDeviceMemoryPitched(const SyclHostMemoryHeap<Type, Dim>& rhs, sycl::queue) : queue(queue)
    {
        allocate(rhs.getSize());
        copyFrom(rhs);
    }

    ~SyclDeviceMemoryPitched() { deallocate(); }

    SyclDeviceMemoryPitched<Type, Dim>& operator=(const SyclDeviceMemoryPitched<Type, Dim>& rhs)
    {
        if (buffer == nullptr)
        {
            allocate(rhs.getSize());
        }
        else if (this->getSize() != rhs.getSize())
        {
            deallocate();
            allocate(rhs.getSize());
        }
        copyFrom(rhs);
        return *this;
    }

    // see below with copy() functions
    sycl::event copyFrom(const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::event prerequisite);
    sycl::event copyFrom(const SyclHostMemoryHeap<Type, Dim>& src, sycl::event prerequisite);

    Type* getBuffer() { return buffer; }
    sycl::queue getQueue() { return queue; }

    const Type* getBuffer() const { return buffer; }
    const sycl::queue getQueue() const { return queue; }

    Type& operator()(size_t x) { return buffer[x]; }

    Type& operator()(size_t x, size_t y)
    {
        Type* row = getRow(y);
        return row[x];
    }

    inline unsigned char* getBytePtr() { return (unsigned char*)buffer; }
    inline const unsigned char* getBytePtr() const { return (unsigned char*)buffer; }

    inline Type* getRow(size_t row)
    {
        unsigned char* ptr = getBytePtr();
        ptr += row * this->getPitch();
        return (Type*)ptr;
    }

    bool allocate(const SyclSize<Dim>& size)
    {
        this->setSize(size, false);

        buffer = sycl::malloc_device<Type>(this->getUnitsTotal(), queue);

        return !(buffer==nullptr);
    }

    void deallocate()
    {
        if (buffer == nullptr)
            return;

        sycl::free(buffer, queue);

        buffer = nullptr;
    }
};

/*********************************************************************************
 * SyclDeviceMemory
 *********************************************************************************/

template<class Type>
class SyclDeviceMemory : public SyclMemorySizeBase<Type, 1>
{
    Type* buffer = nullptr;
    sycl::queue queue;

  public:
    SyclDeviceMemory(sycl::queue queue)
        : buffer(nullptr),
          queue(queue)
    {}

    explicit SyclDeviceMemory(const size_t size) { allocate(size); }

    explicit inline SyclDeviceMemory(const SyclHostMemoryHeap<Type, 1>& rhs)
    {
        allocate(rhs.getSize());
        copy(*this, rhs);
    }

    // constructor with synchronous copy
    SyclDeviceMemory(const Type* inbuf, const size_t size)
    {
        allocate(size);
        copyFrom(inbuf, size, queue).wait();
    }

    ~SyclDeviceMemory() { deallocate(); }

    SyclDeviceMemory<Type>& operator=(const SyclDeviceMemory<Type>& rhs)
    {
        if (buffer == nullptr)
        {
            allocate(rhs.getSize());
        }
        else if (this->getSize() != rhs.getSize())
        {
            deallocate();
            allocate(rhs.getSize());
        }
        copy(*this, rhs);
        return *this;
    }

    Type* getBuffer() { return buffer; }
    const Type* getBuffer() const { return buffer; }

    unsigned char* getBytePtr() { return (unsigned char*)buffer; }
    const unsigned char* getBytePtr() const { return (unsigned char*)buffer; }

    bool allocate(const SyclSize<1>& size)
    {
        this->setSize(size, true);

        buffer = sycl::malloc_device(this->getSize(), queue);

        return !(buffer==nullptr);
    }
    bool allocate(const size_t size) { return allocate(SyclSize<1>(size)); }

    void deallocate()
    {
        if (buffer == nullptr)
            return;

        sycl::free(buffer, queue);

        buffer = nullptr;
    }

    void copyFrom(const Type* inbuf, const size_t num)
    {
        queue.copy(inbuf, buffer, num).wait();
    }

    sycl::event copyFrom(const Type* inbuf, const size_t num, sycl::queue queue, sycl::event prerequisite)
    {
        return queue.copy(inbuf, buffer, num, prerequisite);
    }
};

/*********************************************************************************
 * copyFrom member functions
 *********************************************************************************/

template<class Type, unsigned Dim>
sycl::event SyclDeviceMemoryPitched<Type, Dim>::copyFrom(const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::event prerequisite)
{
    return this->queue.copy(src.getBuffer(), this->getBuffer(), src.getUnitsTotal(), prerequisite);
}

template<class Type, unsigned Dim>
sycl::event SyclDeviceMemoryPitched<Type, Dim>::copyFrom(const SyclHostMemoryHeap<Type, Dim>& src, sycl::event prerequisite)
{
    return this->queue.copy(src.getBuffer(), this->getBuffer(), src.getUnitsTotal(), prerequisite);
}

template<class Type, unsigned Dim>
sycl::event SyclHostMemoryHeap<Type, Dim>::copyFrom(const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::queue queue, sycl::event prerequisite)
{
    return queue.copy(src.getBuffer(), this->getBuffer(), src.getUnitsTotal(), prerequisite);
}

/*********************************************************************************
 * copy functions
 *********************************************************************************/

template<class Type, unsigned Dim>
sycl::event copy(SyclHostMemoryHeap<Type, Dim>& dst, const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::queue queue, sycl::event prerequisite)
{
    return dst.copyFrom(src, queue, prerequisite);
}

template<class Type>
sycl::event copy(SyclHostMemoryHeap<Type, 1>& _dst, const SyclDeviceMemory<Type>& _src, sycl::queue queue, sycl::event prerequisite)
{
    return queue.copy(_src.getBuffer(), _dst.getBuffer(), _src.getUnitsTotal(), prerequisite);
}

template<class Type, unsigned Dim>
sycl::event copy(SyclDeviceMemoryPitched<Type, Dim>& _dst, const SyclHostMemoryHeap<Type, Dim>& _src, sycl::queue queue, sycl::event prerequisite)
{
    return _dst.copyFrom(_src, queue, prerequisite);
}

template<class Type, unsigned Dim>
sycl::event copy(SyclDeviceMemoryPitched<Type, Dim>& _dst, const SyclDeviceMemoryPitched<Type, Dim>& _src, sycl::queue queue, sycl::event prerequisite)
{
    return _dst.copyFrom(_src, queue, prerequisite);
}

template<class Type>
sycl::event copy(SyclDeviceMemory<Type>& _dst, const SyclHostMemoryHeap<Type, 1>& _src, sycl::queue queue, sycl::event prerequisite)
{
    return queue.copy(_src.getBuffer(), _dst.getBuffer(), _src.getUnitsTotal(), prerequisite);
}

template<class Type>
void copy(SyclDeviceMemory<Type>& _dst, const Type* buffer, const size_t numelems)
{
    _dst.copyFrom(buffer, numelems);
}

}  // namespace depthMap
}  // namespace aliceVision
