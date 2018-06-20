// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MaxFlow_AdjList.hpp"

namespace aliceVision {
namespace fuseCut {

void MaxFlow_AdjList::printStats() const
{
    ALICEVISION_LOG_INFO("# vertices: " << _graph.m_vertices.size() << ", capacity: " << _graph.m_vertices.capacity());
    // std::cout << "nb edges: " << _graph.m_edges.size() << ", capacity: " << _graph.m_edges.capacity() << std::endl;

    VertexIterator vi, vi_end;

    std::map<std::size_t, int> histSize;
    std::map<std::size_t, int> histCapacity;

    for(boost::tie(vi, vi_end) = vertices(_graph); vi != vi_end; ++vi)
    {
        std::size_t s = _graph.m_vertices[*vi].m_out_edges.size();
        std::size_t c = _graph.m_vertices[*vi].m_out_edges.capacity();

        if(histSize.find(s) == histSize.end())
            histSize[s] = 1;
        else
            ++histSize[s];

        if(histCapacity.find(c) == histCapacity.end())
            histCapacity[c] = 1;
        else
            ++histCapacity[c];
    }
    // std::cout << "edges: size=" << _graph.m_edges.size() << ", capacity=" << _graph.m_edges.capacity() << std::endl;

    for(const auto& it: histSize)
    {
      ALICEVISION_LOG_INFO("\t- size[" << it.first << "]: " << it.second);
    }
    for(const auto& it: histCapacity)
    {
      ALICEVISION_LOG_INFO("\t- capacity[" << it.first << "]: " << it.second);
    }
}

void MaxFlow_AdjList::printColorStats() const
{
  std::map<int, int> histColor;

  for(const auto& color: _color)
  {
    const int c = (int)color;
    if(histColor.find(c) == histColor.end())
        histColor[c] = 1;
    else
        ++histColor[c];
  }
  ALICEVISION_LOG_INFO("Full (white):" << int(boost::white_color));
  ALICEVISION_LOG_INFO("Empty (black):" << int(boost::black_color));
  ALICEVISION_LOG_INFO("Undefined (gray):" << int(boost::gray_color));

  for(const auto& it: histColor)
  {
    ALICEVISION_LOG_INFO("\t- color[" << it.first << "]: " << it.second);
  }
}

} // namespace fuseCut
} // namespace aliceVision
