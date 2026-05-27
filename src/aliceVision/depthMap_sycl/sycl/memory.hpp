// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

// #define ALICEVISION_DEPTHMAP_TEXTURE_USE_UCHAR
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_HALF
#define ALICEVISION_DEPTHMAP_TEXTURE_USE_INTERPOLATION

#include <aliceVision/depthMap_sycl/sycl/sycl.hpp>

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/mvsUtils/fileIO.hpp>
#include <aliceVision/depthMap_sycl/sycl/buffer.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <stdexcept>
#include <assert.h>
#include <sstream>
#include <iostream>
#include <vector>
#include <cstring>

namespace aliceVision {
namespace depthMap_sycl {

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
using SyclRGB = sycl::vec<SyclColorBaseType, 3>;

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

template<unsigned Dim>
constexpr std::strong_ordering operator<=>(const SyclSizeBase<Dim>& s1, const SyclSizeBase<Dim>& s2)
{
    bool overflow = false;
    for (size_t i = Dim; i--;)
    {
        if (s1[i] < s2[i])
            return std::strong_ordering::less;
        else if (s1[i] > s2[i])
           overflow = true;
    }

    return overflow ? std::strong_ordering::greater : std::strong_ordering::equal;
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
    SyclHostMemoryHeap(sycl::queue& queue)
        : buffer(nullptr),
          queue(queue)
    {}

    explicit SyclHostMemoryHeap(const SyclSize<Dim>& size, sycl::queue& queue)
        : buffer(nullptr),
          queue(queue)
    {
        allocate(size);
    }

    // Copy operators call the destructor and so deallocate the memory we are trying to transfer
    // Move or manually create a new object and call memcpy
    SyclHostMemoryHeap(SyclHostMemoryHeap& in) = delete; // copy
    SyclHostMemoryHeap(SyclHostMemoryHeap&& in) = default; // move
    SyclHostMemoryHeap& operator=(SyclHostMemoryHeap& in) = delete;

    ~SyclHostMemoryHeap() { deallocate(); }

    void initBuffer() { memset(buffer, 0, this->getBytesPadded()); }

    // see below with copy() functions
    inline sycl::event copyFrom(const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::queue& queue, sycl::event prerequisite);
    inline void copyFrom(const image::Image<image::RGBfColor>& img);
    inline void copyTo(image::Image<image::RGBfColor>& img) const;

    inline Type* getBuffer() { return buffer; }
    inline const Type* getBuffer() const { return buffer; }

    inline sycl::queue& getQueue() { return queue; }
    inline const sycl::queue& getQueue() const { return queue; }

    inline Type& operator()(size_t x) { return buffer[x]; }
    inline const Type& operator()(size_t x) const { return buffer[x]; }

    inline Type& operator()(sycl::uint2 coords) {
        static_assert(Dim == 2, "This operator is only available for 2d images");
        return buffer[getAddress<2>(sycl::uint2(this->getUnitsInDim(0),
                                                this->getUnitsInDim(1)),
                                   coords)];
    }
    inline const Type& operator()(sycl::uint2 coords) const {
        static_assert(Dim == 2, "This operator is only available for 2d images");
        return buffer[getAddress<2>(sycl::uint2(this->getUnitsInDim(0),
                                                this->getUnitsInDim(1)),
                                    coords)];
    }
    inline Type& operator()(size_t x, size_t y) {
        return this->operator()(sycl::uint2(x, y));
    }
    inline const Type& operator()(size_t x, size_t y) const {
        return this->operator()(sycl::uint2(x, y));
    }

    inline Type& operator()(sycl::uint3 coords) {
        static_assert(Dim == 3, "This operator is only available for 3d volumes");
        return buffer[getAddress<3>(sycl::uint3(this->getUnitsInDim(0),
                                                this->getUnitsInDim(1),
                                                this->getUnitsInDim(2)),
                                   coords)];
    }
    inline const Type& operator()(sycl::uint3 coords) const {
        static_assert(Dim == 3, "This operator is only available for 3d volumes");
        return buffer[getAddress<3>(sycl::uint3(this->getUnitsInDim(0),
                                                this->getUnitsInDim(1),
                                                this->getUnitsInDim(2)),
                                   coords)];
    }
    inline Type& operator()(size_t x, size_t y, size_t z) {
        return this->operator()(sycl::uint3(x, y, z));
    }
    inline const Type& operator()(size_t x, size_t y, size_t z) const {
        return this->operator()(sycl::uint3(x, y, z));
    }

    inline unsigned char* getBytePtr() { return (unsigned char*)buffer; }
    inline const unsigned char* getBytePtr() const { return (unsigned char*)buffer; }

  public:
    void allocate(const SyclSize<Dim>& size)
    {
        this->setSize(size, true);

        buffer = sycl::malloc_host<Type>(this->getUnitsTotal(), queue);

        assert(buffer != nullptr);
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
    SyclDeviceMemoryPitched(sycl::queue& queue)
        : buffer(nullptr),
          queue(queue)
    {}

    explicit SyclDeviceMemoryPitched(const SyclSize<Dim>& size, bool& allocSuccess, sycl::queue& queue) : buffer(nullptr), queue(queue)
    { allocSuccess &= allocate(size); }

    explicit SyclDeviceMemoryPitched(const SyclHostMemoryHeap<Type, Dim>& rhs, bool& allocSuccess, sycl::queue) : queue(queue)
    {
        allocSuccess &= allocate(rhs.getSize());
        if (!allocSuccess) copyFrom(rhs);
    }

    // Copy operators call the destructor and so deallocate the memory we are trying to transfer
    // Move or manually create a new object and call copyFrom
    SyclDeviceMemoryPitched(SyclDeviceMemoryPitched& in) = delete; // copy
    SyclDeviceMemoryPitched(SyclDeviceMemoryPitched&& in) = default; // move
    SyclDeviceMemoryPitched& operator=(SyclDeviceMemoryPitched& in) = delete;

    ~SyclDeviceMemoryPitched() { deallocate(); }

    // see below with copy() functions
    inline sycl::event copyFrom(const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::queue& queue, sycl::event prerequisite);
    inline sycl::event copyFrom(const SyclHostMemoryHeap<Type, Dim>& src, sycl::queue& queue, sycl::event prerequisite);
    inline sycl::event copyFrom(const image::Image<image::RGBAfColor>& img, sycl::queue& queue, sycl::event prerequisite);
    inline sycl::event copyTo(image::Image<image::RGBAfColor>& img, sycl::queue& queue, sycl::event prerequisite);

    Type* getBuffer() { return buffer; }
    sycl::queue& getQueue() { return queue; }

    Type* const getBuffer() const { return buffer; }
    const sycl::queue& getQueue() const { return queue; }

    inline unsigned char* getBytePtr() { return (unsigned char*)buffer; }
    inline const unsigned char* getBytePtr() const { return (unsigned char*)buffer; }

    // We use a XZY layout so these functions can work. See sycl/buffer.hpp
    inline const Type* sliceY(unsigned int y) const {
        return &(this->buffer[y*this->getUnitsInDim(0)*this->getUnitsInDim(2)]);
    }
    inline Type* sliceY(unsigned int y) {
        return &(this->buffer[y*this->getUnitsInDim(0)*this->getUnitsInDim(2)]);
    }

    inline bool allocate(SyclSize<Dim> size)
    {
        this->setSize(size, false);

        assert(buffer==nullptr && "Memory leak, deallocate before calling allocate again");

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
 * SyclDevicePitchedAccess
 *********************************************************************************/

// Super minimal class for device access
// Circumvents deleted copy operator, but doesn't allow reallocation or anything of the like

template<class Type, unsigned Dim>
class SyclDevicePitchedAccessBase
{
public:
    Type* const buffer;
protected:
    const sycl::vec<uint, Dim> _dims;

    explicit SyclDevicePitchedAccessBase(const SyclDeviceMemoryPitched<Type, Dim>& owner) :
        buffer(owner.getBuffer()),
        _dims([&]{
            sycl::vec<unsigned int, Dim> dims{};
#pragma unroll
            for(int i = Dim; i--;)
                dims[i] = owner.getUnitsInDim(i);
            return dims;
        } ())
    {};
public:
    const sycl::vec<uint, Dim>& getDims() const { return _dims; };

    size_t getUnitsTotal() const {
        size_t product = 1;
#pragma unroll
        for(int i = 0; i < Dim; i++)
            product *= _dims[i];
        return product;
    };
};

template<class Type, unsigned Dim>
class SyclDevicePitchedAccess : public SyclDevicePitchedAccessBase<Type, Dim>
{
public:
    explicit SyclDevicePitchedAccess(const SyclDeviceMemoryPitched<Type, Dim>& owner) :
        SyclDevicePitchedAccessBase<Type, Dim>(owner) {};
};

template<class Type>
class SyclDevicePitchedAccess<Type, 1> : public SyclDevicePitchedAccessBase<Type, 1>
{
public:
    explicit SyclDevicePitchedAccess(const SyclDeviceMemoryPitched<Type, 1>& owner) :
        SyclDevicePitchedAccessBase<Type, 1>(owner) {};

    /**
     * @brief Nearest lookup at clamped unnormalized coords
     * @note Results in undefined behaviour if called from host
     * @param[in] coords coordinates to sample at
     * @return sampled texel
     */
    inline Type sample_near(const sycl::vec<float, 1>& coords) const
    {
        sycl::vec<unsigned int, 1> rounded = coords.template convert<unsigned int>();
        rounded = sycl::clamp(rounded, 0, this->_dims - 1);
        return this->operator()(rounded);
    };

    inline Type& operator()(unsigned int coords) const
    {
        return *(this->buffer + coords);
    };
};

template<class Type>
class SyclDevicePitchedAccess<Type, 2> : public SyclDevicePitchedAccessBase<Type, 2>
{
public:
    explicit SyclDevicePitchedAccess(const SyclDeviceMemoryPitched<Type, 2>& owner) :
        SyclDevicePitchedAccessBase<Type, 2>(owner) {};

    /**
     * @brief Linear sample at unnormalized coords
     * @note Results in undefined behaviour if called from host
     * @note Only works in two dimensions
     * @param[in] coords coordinates to sample at
     * @return sampled texel
     *\/
    inline Type linear(const sycl::vec<float, 2>& coords) const
    {
        const sycl::vec<float, 2> bl_c = sycl::floor(coords); // Bottom left
        const sycl::vec<float, 2> tr_c = sycl::ceil(coords);  // Top right
        const sycl::vec<float, 2> br_c = sycl::vec<float, 2>(tr_c.x(), bl_c.y());
        const sycl::vec<float, 2> tl_c = sycl::vec<float, 2>(bl_c.x(), tr_c.y());

        const Type bl_v = sample_near(bl_c);
        const Type tr_v = sample_near(tr_c);
        const Type br_v = sample_near(br_c);
        const Type tl_v = sample_near(tl_c);

        const sycl::vec<float, 2> mix = coords - bl_c;

        const Type b_v = sycl::mix(bl_v, br_v, mix.x()); // Bottom
        const Type t_v = sycl::mix(tl_v, tr_v, mix.x()); // Top

        return sycl::mix(b_v, t_v, mix.y());
    };
    */

    /**
     * @brief Nearest lookup at clamped unnormalized coords, in 2 dimensions
     * @note Results in undefined behaviour if called from host
     * @param[in] coords coordinates to sample at
     * @return sampled texel
     */
    inline Type sample_near(const sycl::vec<float, 2>& coords) const
    {
        sycl::vec<unsigned int, 2> rounded = coords.template convert<unsigned int>();
        rounded = sycl::clamp(rounded, sycl::vec<unsigned int, 2>(0), this->_dims - 1);
        return this->operator()(rounded);
    };

    inline Type& operator()(const sycl::vec<unsigned int, 2>& coords) const
    {
        return *(this->buffer + getAddress<2>(this->_dims, coords));
    };
};

template<class Type>
class SyclDevicePitchedAccess<Type, 3> : public SyclDevicePitchedAccessBase<Type, 3>
{
public:
    explicit SyclDevicePitchedAccess(const SyclDeviceMemoryPitched<Type, 3>& owner) :
        SyclDevicePitchedAccessBase<Type, 3>(owner) {};

    /**
     * @brief Nearest lookup at clamped unnormalized coords, in 3 dimensions
     * @note Results in undefined behaviour if called from host
     * @param[in] coords coordinates to sample at
     * @return sampled texel
     */
    inline Type sample_near(const sycl::vec<float, 3>& coords) const
    {
        sycl::vec<unsigned int, 3> rounded = coords.template convert<unsigned int>();
        rounded = sycl::clamp(rounded, sycl::vec<unsigned int, 3>(0), this->_dims - 1);
        return this->operator()(rounded);
    };

    inline Type& operator()(const sycl::vec<unsigned int, 3>& coords) const
    {
        return *(this->buffer + getAddress<3>(this->_dims, coords));
    };
};

/*********************************************************************************
 * helper function for using texture cache on devices where it is available
 *********************************************************************************/

template<class Type>
inline Type __readonly_load(const Type* const __restrict__ ptr)
{
__acpp_if_target_hiplike(
	return __ldg(ptr);
)
    // generic fallthrough
    return *ptr;
}

/*********************************************************************************
 * copyFrom member functions
 *********************************************************************************/

template<class Type, unsigned Dim>
sycl::event SyclDeviceMemoryPitched<Type, Dim>::copyFrom(const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::queue& queue, sycl::event prerequisite)
{
    assert(this->getSize() == src.getSize());
    return queue.copy(src.getBuffer(), this->getBuffer(), src.getUnitsTotal(), prerequisite);
}

template<class Type, unsigned Dim>
sycl::event SyclDeviceMemoryPitched<Type, Dim>::copyFrom(const SyclHostMemoryHeap<Type, Dim>& src, sycl::queue& queue, sycl::event prerequisite)
{
    assert(this->getSize() == src.getSize());
    return queue.copy(src.getBuffer(), this->getBuffer(), src.getUnitsTotal(), prerequisite);
}

template<class Type, unsigned Dim>
inline sycl::event SyclHostMemoryHeap<Type, Dim>::copyFrom(const SyclDeviceMemoryPitched<Type, Dim>& src, sycl::queue& queue, sycl::event prerequisite)
{
    bool canMemcopy = true;

    const SyclSize inSize = src.getSize();
    const SyclSize outSize = this->getSize();

#pragma unroll
    for(int i = Dim - 1; i--;) // Last dimension doesn't affect memory layout
    {
        if(inSize[i] != outSize[i])
        {
            canMemcopy = false;
            break;
        }
    }

    if(Dim == 1 || canMemcopy)
        return queue.copy(src.getBuffer(), this->getBuffer(), std::min(src.getUnitsTotal(), this->getUnitsTotal()), prerequisite);
    else
    {
        const SyclDevicePitchedAccess acc{src};
        sycl::vec<uint, Dim> dims{};
        sycl::range<Dim> range{};

#pragma unroll
        for(int i = Dim; i--;)
        {
            const unsigned int dim = std::min(inSize[i], outSize[i]);
            dims[i] = dim;
            range[i] = dim;
        }

        Type* buffer = this->getBuffer();

        return queue.submit([&] (sycl::handler& h) {
            h.depends_on(prerequisite);

            h.parallel_for(range, [=] (sycl::id<Dim> id) {
                const sycl::vec<uint, Dim> coords = [&] () {
                    sycl::vec<uint, Dim> coords{0};

#pragma unroll
                    for(int i = Dim; i--;)
                        coords[i] = id[i];

                    return coords;
                } ();

                *(buffer + getAddress<Dim>(dims, coords)) = acc(coords);
            });
        });
    }
}

template<>
inline sycl::event SyclDeviceMemoryPitched<sycl::float4, 2>::copyFrom(
                        const image::Image<image::RGBAfColor>& img,
                        sycl::queue& queue,
                        sycl::event prerequisite)
{
    static constexpr size_t size = sizeof(sycl::float4);
    static_assert(size == sizeof(image::RGBAfColor));

    assert(img.cols() <= this->getSize()[0]);
    assert(img.rows() <= this->getSize()[1]);

    if(img.width() == this->getUnitsInDim(0))
        return queue.memcpy(this->getBytePtr(),
                            (unsigned char*)img.data(),
                            img.size()*sizeof(sycl::float4),
                            prerequisite);
    else
        for(int r = 0; r < img.rows(); r++)
            prerequisite = queue.memcpy(this->getBytePtr() + r * this->getSize()[0] * size,
                                        (unsigned char*)img.data() + r * img.cols() * size,
                                        img.cols()*sizeof(sycl::float4),
                                        prerequisite);
    return prerequisite;
}

template<>
inline void SyclHostMemoryHeap<SyclRGB, 2>::copyFrom(const image::Image<image::RGBfColor>& img)
{
    const int width = img.width();
    const int height = img.width();
    assert(width == this->getUnitsInDim(0));
    assert(height == this->getUnitsInDim(1));
    // copy image from host memory to output images
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const image::RGBfColor value_in = img(y, x);
            SyclRGB& value_out = this->operator()(size_t(x), size_t(y));
            value_out.x() = value_in.r();
            value_out.y() = value_in.g();
            value_out.z() = value_in.b();
        }
    }
}

template<>
inline void SyclHostMemoryHeap<sycl::float3, 2>::copyFrom(const image::Image<image::RGBfColor>& img)
{
    const int width = img.width();
    const int height = img.width();
    assert(width == this->getUnitsInDim(0));
    assert(height == this->getUnitsInDim(1));
    // copy image from host memory to output images
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const image::RGBfColor value_in = img(y, x);
            sycl::float3& value_out = this->operator()(size_t(x), size_t(y));
            value_out.x() = value_in.r();
            value_out.y() = value_in.g();
            value_out.z() = value_in.b();
        }
    }
}

template<>
inline sycl::event SyclDeviceMemoryPitched<sycl::float4, 2>::copyTo(
                        image::Image<image::RGBAfColor>& img,
                        sycl::queue& queue,
                        sycl::event prerequisite)
{
    static constexpr size_t size = sizeof(sycl::float4);
    static_assert(size == sizeof(image::RGBAfColor));

    assert(img.cols() <= this->getSize()[0]);
    assert(img.rows() <= this->getSize()[1]);

    if(img.width() == this->getUnitsInDim(0))
        return queue.memcpy((unsigned char*)img.data(),
                            this->getBytePtr(),
                            img.size()*sizeof(sycl::float4),
                            prerequisite);
    else
        for(int r = 0; r < img.rows(); r++)
            prerequisite = queue.memcpy((unsigned char*)img.data() + r * img.cols() * size,
                                        this->getBytePtr() + r * this->getSize()[0] * size,
                                        img.cols()*sizeof(sycl::float4),
                                        prerequisite);
    return prerequisite;
}

template<>
inline void SyclHostMemoryHeap<SyclRGB, 2>::copyTo(image::Image<image::RGBfColor>& img) const
{
    const int width = img.width();
    const int height = img.width();
    assert(width == this->getUnitsInDim(0));
    assert(height == this->getUnitsInDim(1));
    // copy image from host memory to output images
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const sycl::float3 value_in = this->operator()(size_t(x), size_t(y)).convert<float>();
            image::RGBfColor& value_out = img(y, x);
            value_out.r() = value_in.x();
            value_out.g() = value_in.y();
            value_out.b() = value_in.z();
        }
    }
}

template<>
inline void SyclHostMemoryHeap<sycl::float3, 2>::copyTo(image::Image<image::RGBfColor>& img) const
{
    const int width = img.width();
    const int height = img.height();
    assert(width == this->getUnitsInDim(0));
    assert(height == this->getUnitsInDim(1));
    // copy image from host memory to output images
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const sycl::float3 value_in = this->operator()(size_t(x), size_t(y)).convert<float>();
            image::RGBfColor& value_out = img(y, x);
            value_out.r() = value_in.x();
            value_out.g() = value_in.y();
            value_out.b() = value_in.z();
        }
    }
}

}  // namespace depthMap_sycl
}  // namespace aliceVision
