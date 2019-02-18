#pragma once
#include "cudaUtil.hpp"

#include <assert.h>
#include <vector>
#include <memory>
#include <numeric>


namespace aliceVision {
namespace intrinsicDecomposition {


inline unsigned int totalElementsFromDims(const std::vector<unsigned int>& dims)
{
    return std::accumulate(dims.begin(), dims.end(), 1, std::multiplies<unsigned int>());
}


class OptImage
{
public:
    enum Type { FLOAT, DOUBLE, UCHAR, INT };
    enum Location { CPU, GPU };
    OptImage(std::vector<unsigned int> dims, void* data, Type type, unsigned int channelCount, Location location, bool usesOptFloat = false, bool ownsData = false) :
        m_dims(dims), m_data(data), m_type(type), m_channelCount(channelCount), m_location(location), m_usesOptFloat(usesOptFloat), m_ownsData(ownsData)
    {
        m_reserveSize = 0;
        for(unsigned int v: dims)
            m_reserveSize += v;
    }

    ~OptImage()
    {
        if (m_ownsData && m_data) {
            if (m_location == Location::GPU) {
                cudaSafeCall(cudaFree(m_data));
            } else {
                free(m_data);
            }
        }
    }

    void* data() const { return m_data; }
    Location location() const { return m_location; }
    Type type() const { return m_type; }
    void resetDims(const std::vector<unsigned int>& d)
    {
        unsigned int maxElements = 0;
        for(unsigned int v: d)
            maxElements += v;
        if(maxElements > m_reserveSize)
            throw std::runtime_error("Failed to reset dimensions of Opt Image. The new dimensions are larger that the allocated buffer.");
        m_dims = d;
    }
    std::vector<unsigned int> dims() const { return m_dims; }
    bool usesOptFloat() const { return m_usesOptFloat; }

    size_t dataSize() const { return totalElementsFromDims(m_dims) * OptImage::typeSize(m_type) * m_channelCount; }

    static cudaMemcpyKind cudaMemcpyType(OptImage::Location dstLoc, OptImage::Location srcLoc) {
        if (srcLoc == OptImage::Location::CPU) {
            return (dstLoc == OptImage::Location::CPU) ? cudaMemcpyHostToHost : cudaMemcpyHostToDevice;
        } else {
            return (dstLoc == OptImage::Location::CPU) ? cudaMemcpyDeviceToHost : cudaMemcpyDeviceToDevice;
        }
    }

    void update(void* newData, size_t byteCount, Location loc) {
        cudaSafeCall(cudaMemcpy(data(), newData, byteCount, cudaMemcpyType(location(), loc)));
    }

    template<typename T>
    void update(const std::vector<T>& data) {
        size_t byteCount = std::min(data.size()*sizeof(T), dataSize());
        update((void*)data.data(), byteCount, Location::CPU);
    }

    void copyTo(void* buffer, Location loc = Location::CPU, size_t maxBufferSize = std::numeric_limits<size_t>::max()) const
    {
        size_t byteCount = std::min(maxBufferSize, dataSize());
        cudaSafeCall(cudaMemcpy(buffer, data(), byteCount, cudaMemcpyType(loc, location())));
    }

    template<typename T>
    void copyTo(const std::vector<T>& data) const
    {
        copyTo((void*)data.data(), Location::CPU, data.size()*sizeof(T));
    }

    unsigned int channelCount() const { return m_channelCount; }

    static size_t typeSize(Type t) {
        switch (t){
        case Type::INT:
            return 4;
        case Type::FLOAT:
            return 4;
        case Type::DOUBLE:
            return 8;
        case Type::UCHAR:
            return 1;
        }
        return 0;
    }
protected:
    bool m_ownsData = false;
    std::vector<unsigned int> m_dims;
    std::size_t m_reserveSize = 0;
    unsigned int m_channelCount = 0;
    void* m_data = nullptr;
    bool m_usesOptFloat = false;
    Type m_type = Type::FLOAT;
    Location m_location = Location::CPU;
};


inline std::shared_ptr<OptImage> createEmptyOptImage(std::vector<unsigned int> dims, OptImage::Type type, unsigned int channelCount, OptImage::Location location, bool usesOptFloat) {
    void* data;
    size_t size = totalElementsFromDims(dims) * OptImage::typeSize(type) * channelCount;
    if (location == OptImage::Location::CPU) {
        data = malloc(size); 
    } else {
        cudaSafeCall(cudaMalloc(&data, size));
    }
    return std::shared_ptr<OptImage>(new OptImage(dims, data, type, channelCount, location, usesOptFloat, true));
}

inline void copyImage(const std::shared_ptr<OptImage>& dst, const std::shared_ptr<OptImage>& src) {
    assert(src->type() == dst->type());
    // TODO dimension asserts
    size_t size = dst->dataSize();
    dst->update(src->data(), size, src->location());
}

inline std::shared_ptr<OptImage> copyImageTo(const std::shared_ptr<OptImage>& original, OptImage::Location location) {
    std::shared_ptr<OptImage> newIm = createEmptyOptImage(original->dims(), original->type(), original->channelCount(), location, original->usesOptFloat());
    copyImage(newIm, original);
    return newIm;
}

// Only CPU->CPU
inline std::shared_ptr<OptImage> getDoubleImageFromFloatImage(const std::shared_ptr<OptImage> floatImage) {
    assert(floatImage->location() == OptImage::Location::CPU && floatImage->type() == OptImage::Type::FLOAT);
    std::shared_ptr<OptImage> newIm = createEmptyOptImage(floatImage->dims(), OptImage::Type::DOUBLE, floatImage->channelCount(), floatImage->location(), floatImage->usesOptFloat());
    float* fPtr = (float*)floatImage->data();
    double* dPtr = (double*)newIm->data();
    for (size_t i = 0; i < totalElementsFromDims(floatImage->dims()) * floatImage->channelCount(); ++i) {
        dPtr[i] = (double)fPtr[i];
    }
    return newIm;
}

// Only CPU->CPU
inline std::shared_ptr<OptImage> getFloatImageFromDoubleImage(const std::shared_ptr<OptImage> doubleImage) {
    assert(doubleImage->location() == OptImage::Location::CPU && doubleImage->type() == OptImage::Type::DOUBLE);
    std::shared_ptr<OptImage> newIm = createEmptyOptImage(doubleImage->dims(), OptImage::Type::FLOAT, doubleImage->channelCount(), doubleImage->location(), doubleImage->usesOptFloat());
    double* dPtr = (double*)doubleImage->data();
    float* fPtr = (float*)newIm->data();
    for (size_t i = 0; i < totalElementsFromDims(doubleImage->dims()) * doubleImage->channelCount(); ++i) {
        fPtr[i] = (float)dPtr[i];
    }
    return newIm;
}

}
}
