// This file is part of the AliceVision project.
// Copyright (c) 2018 AliceVision contributors.
// Copyright (C) 2013  LGG, EPFL
//   "Sparse Iterative Closest Point" by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.
#pragma once

#include <nanoflann.hpp>

namespace nanoflann {

/// @brief NANOFLANN KD-tree adaptor for EIGEN
/// KD-tree adaptor for working with data directly stored in an Eigen Matrix, without duplicating the data storage.
/// This code is adapted from the KDTreeEigenMatrixAdaptor class of nanoflann.hpp
template <class MatrixType, int DIM = -1, class Distance = nanoflann::metric_L2, typename IndexType = int>
struct KDTreeAdaptor {
    typedef KDTreeAdaptor<MatrixType,DIM,Distance> self_t;
    typedef typename MatrixType::Scalar              num_t;
    typedef typename Distance::template traits<num_t,self_t>::distance_t metric_t;
    typedef KDTreeSingleIndexAdaptor< metric_t,self_t,DIM,IndexType>  index_t;
    index_t* index;
    KDTreeAdaptor(const MatrixType &mat, const int leaf_max_size = 10) : m_data_matrix(mat) {
        const size_t dims = mat.rows();
        index = new index_t( dims, *this, nanoflann::KDTreeSingleIndexAdaptorParams(leaf_max_size) );
        index->buildIndex();
    }
    ~KDTreeAdaptor() {delete index;}
    const MatrixType &m_data_matrix;
    /// Query for the num_closest closest points to a given point (entered as query_point[0:dim-1]).
    inline void query(const num_t *query_point, const size_t num_closest, IndexType *out_indices, num_t *out_distances_sq) const {
        nanoflann::KNNResultSet<typename MatrixType::Scalar,IndexType> resultSet(num_closest);
        resultSet.init(out_indices, out_distances_sq);
        index->findNeighbors(resultSet, query_point, nanoflann::SearchParams());
    }
    /// Query for the closest points to a given point (entered as query_point[0:dim-1]).
    inline IndexType closest(const num_t *query_point) const {
        IndexType out_indices;
        num_t out_distances_sq;
        query(query_point, 1, &out_indices, &out_distances_sq);
        return out_indices;
    }
    const self_t & derived() const {return *this;}
    self_t & derived() {return *this;}
    inline size_t kdtree_get_point_count() const {return m_data_matrix.cols();}
    /// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
    inline num_t kdtree_distance(const num_t *p1, const size_t idx_p2,size_t size) const {
        num_t s=0;
        for (size_t i=0; i<size; i++) {
            const num_t d= p1[i]-m_data_matrix.coeff(i,idx_p2);
            s+=d*d;
        }
        return s;
    }
    /// Returns the dim'th component of the idx'th point in the class:
    inline num_t kdtree_get_pt(const size_t idx, int dim) const {
        return m_data_matrix.coeff(dim,idx);
    }
    /// Optional bounding-box computation: return false to default to a standard bbox computation loop.
    template <class BBOX> bool kdtree_get_bbox(BBOX&) const {return false;}
};

}
