// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once


#include <aliceVision/types.hpp>

#include <memory>
#include <Eigen/Dense>

namespace aliceVision {
namespace fuseCut {

struct RayInfo
{
    IndexT start;
    IndexT end;
};

class SimpleNode
{
  public:
    using uptr = std::unique_ptr<SimpleNode>;
    using ptr = SimpleNode*;

  public:
    SimpleNode(const Eigen::Vector3d& bbmin, const Eigen::Vector3d& bbmax)
    {
        _bbMin = bbmin;
        _bbMax = bbmax;
    }

    double getSize()
    {
        Eigen::Vector3d rel = _bbMax - _bbMin;
        return rel.maxCoeff();
    }

    void store(const Eigen::Vector3d& pt)
    {
        if (!isInside(pt))
        {
            return;
        }

        double size = getSize();
        if (size > _minSize)
        {
            subdivide();

            for (auto& item : _nodes)
            {
                if (item == nullptr)
                {
                    continue;
                }

                item->store(pt);
            }
        }
        else
        {
            _count++;
        }
    }

    bool isInside(const Eigen::Vector3d& pt) const
    {
        return (pt.x() >= _bbMin.x() && pt.y() >= _bbMin.y() && pt.z() >= _bbMin.z() && pt.x() <= _bbMax.x() && pt.y() <= _bbMax.y() &&
                pt.z() <= _bbMax.z());
    }

    void subdivide()
    {
        if (_nodes[0] != nullptr)
        {
            return;
        }

        Eigen::Vector3d center = (_bbMin + _bbMax) * 0.5;

        double xs[] = {_bbMin.x(), center.x(), _bbMax.x()};
        double ys[] = {_bbMin.y(), center.y(), _bbMax.y()};
        double zs[] = {_bbMin.z(), center.z(), _bbMax.z()};

        int pos = 0;
        for (int ix = 0; ix < 2; ix++)
        {
            for (int iy = 0; iy < 2; iy++)
            {
                for (int iz = 0; iz < 2; iz++)
                {
                    Eigen::Vector3d lmin;
                    lmin.x() = xs[ix];
                    lmin.y() = ys[iy];
                    lmin.z() = zs[iz];

                    Eigen::Vector3d lmax;
                    lmax.x() = xs[ix + 1];
                    lmax.y() = ys[iy + 1];
                    lmax.z() = zs[iz + 1];

                    _nodes[pos] = std::make_unique<SimpleNode>(lmin, lmax);
                    pos++;
                }
            }
        }
    }

    void visit(std::vector<SimpleNode::ptr>& list)
    {
        if (_count > 0)
        {
            list.push_back(this);
        }

        for (auto& item : _nodes)
        {
            if (item)
            {
                item->visit(list);
            }
        }
    }

    void regroup(size_t maxPointsPerNode)
    {
        size_t count = getCount();

        if (count == 0)
        {
            return;
        }

        if (count < maxPointsPerNode)
        {
            _count = count;

            for (auto& item : _nodes)
            {
                item.reset();
                item = nullptr;
            }

            return;
        }

        for (auto& item : _nodes)
        {
            if (item)
            {
                item->regroup(maxPointsPerNode);
            }
        }
    }

    const size_t getCount() const
    {
        size_t count = _count;

        for (const auto& item : _nodes)
        {
            if (item)
            {
                count += item->getCount();
            }
        }

        return count;
    }

    const Eigen::Vector3d& getBBMin() const { return _bbMin; }

    const Eigen::Vector3d& getBBMax() const { return _bbMax; }

  private:
    std::array<SimpleNode::uptr, 8> _nodes;
    Eigen::Vector3d _bbMin;
    Eigen::Vector3d _bbMax;
    size_t _count = 0;
    double _minSize = 2.0;
};

}  // namespace fuseCut
}  // namespace aliceVision