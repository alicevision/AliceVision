#pragma once

#include "cudaUtil.hpp"
#include "OptImage.hpp"

#include <assert.h>
#include <vector>
#include <memory>
#include <numeric>


namespace aliceVision {
namespace intrinsicDecomposition {

/** 
    Small wrapper class for connectivity.
    Does not allow for full expressivity Opt allows for;
    only currently supports graphs connecting 1D images.

    {m_indices[0][i], m_indices[1][i], ..., m_indices[m_indices.size()-1][i]}

    defines a single hyper-edge of the graph.
*/
class OptGraph
{
public:

    OptGraph(std::vector<std::vector<int>> indices) : m_indices(indices), m_edgeCount((int)indices[0].size()){
        copyToGPU();
    }

    OptGraph(size_t edgeCount, size_t edgeSize) : m_edgeCount((int)edgeCount) {
        m_indices.resize(edgeSize);
        m_gpuIndices.resize(edgeSize);
        for (size_t i = 0; i < edgeSize; ++i) {
            m_indices[i].resize(edgeCount);
        }
        copyToGPU();
    }

    int* edgeCountPtr() {
        return &m_edgeCount;
    }

    size_t edgeSize() const {
        return m_indices.size();
    }

    int* gpuVertexPtr(int index) {
        return (int*)m_gpuIndices[index]->data();
    }

private:
    void copyToGPU() {
        m_gpuIndices.resize(m_indices.size());
        for (size_t i = 0; i < m_indices.size(); ++i) {
            std::vector<unsigned int> dims = { (unsigned int)m_indices[i].size() };
            auto cpuImage = std::make_shared<OptImage>(dims, (void*)m_indices[i].data(), OptImage::Type::INT, 1, OptImage::Location::CPU);
            m_gpuIndices[i] = copyImageTo(cpuImage, OptImage::Location::GPU);
        }
    }

    // CPU storage
    std::vector<std::vector<int>> m_indices;
    std::vector<std::shared_ptr<OptImage>> m_gpuIndices;
    // Copy of m_gpuIndices.size() in int form for use by Opt
    int m_edgeCount = 0;
};

inline std::shared_ptr<OptGraph> createGraphFromNeighborLists(const std::vector<int>& neighborIdx, const std::vector<int>& neighborOffset)
{
    // Convert to our edge format
    std::vector<int> h_head;
    std::vector<int> h_tail;
    for (int head = 0; head < (int)neighborOffset.size() - 1; ++head) {
        for (int j = neighborOffset[head]; j < neighborOffset[head + 1]; ++j) {
            h_head.push_back(head);
            h_tail.push_back(neighborIdx[j]);
        }
    }
    return std::make_shared<OptGraph>(std::vector<std::vector<int> >({ h_head, h_tail }));

}

}
}
