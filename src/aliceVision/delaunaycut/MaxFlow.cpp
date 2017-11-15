// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "MaxFlow.hpp"


void MaxFlow::printStats() const
{
    std::cout << "nb vertices: " << _graph.m_vertices.size() << ", capacity: " << _graph.m_vertices.capacity() << std::endl;
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
      std::cout << "size[" << it.first << "]: " << it.second << std::endl;
    }
    for(const auto& it: histCapacity)
    {
      std::cout << "capacity[" << it.first << "]: " << it.second << std::endl;
    }
}

void MaxFlow::printColorStats() const
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
  std::cout << "Full (white):" << int(boost::white_color) << std::endl;
  std::cout << "Empty (black):" << int(boost::black_color) << std::endl;
  std::cout << "Undefined (gray):" << int(boost::gray_color) << std::endl;

  for(const auto& it: histColor)
  {
    std::cout << "color[" << it.first << "]: " << it.second << std::endl;
  }
}
