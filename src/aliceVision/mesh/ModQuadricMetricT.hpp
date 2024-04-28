/* ========================================================================= *
 *                                                                           *
 *                               OpenMesh                                    *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openmesh.org                               *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenMesh.                                            *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
 *                                                                           *
 * ========================================================================= */

/*===========================================================================*\
 *                                                                           *
 *   $Revision$                                                         *
 *   $Date$                   *
 *                                                                           *
\*===========================================================================*/

//=============================================================================
//
//  CLASS ModQuadricT
//
//=============================================================================

#ifndef OSG_MODQUADRICMETRIC_HH
    #define OSG_MODQUADRICMETRIC_HH

//== INCLUDES =================================================================

    #include <float.h>
    #include <OpenMesh/Tools/Decimater/ModBaseT.hh>
    #include <OpenMesh/Core/Utils/Property.hh>
    #include <OpenMesh/Core/Utils/vector_cast.hh>
    #include "QuadricMetricT.hpp"

//== NAMESPACE ================================================================

namespace OpenMesh {
namespace Decimater {

//== CLASS DEFINITION =========================================================

/** \brief Mesh decimation module computing collapse priority based on error quadrics.
 *
 *  This module can be used as a binary and non-binary module.
 */
template<class MeshT>
class ModQuadricMetricT : public ModBaseT<MeshT>
{
  public:
    // Defines the types Self, Handle, Base, Mesh, and CollapseInfo
    // and the memberfunction name()
    DECIMATING_MODULE(ModQuadricMetricT, MeshT, Quadric);

  public:
    /** Constructor
     *  \internal
     */
    ModQuadricMetricT(MeshT& _mesh)
      : Base(_mesh, false)
    {
        unset_max_err();
        Base::mesh().add_property(quadrics_);
    }

    /// Destructor
    virtual ~ModQuadricMetricT() { Base::mesh().remove_property(quadrics_); }

  public:  // inherited
    /// Initalize the module and prepare the mesh for decimation.
    virtual void initialize(void);

    /** Compute collapse priority based on error quadrics.
     *
     *  \see ModBaseT::collapse_priority() for return values
     *  \see set_max_err()
     */
    virtual float collapse_priority(const CollapseInfo& _ci)
    {
        using namespace OpenMesh;

        typedef Geometry::QuadricMetricT<double> Q;

        Q q = Base::mesh().property(quadrics_, _ci.v0);
        q += Base::mesh().property(quadrics_, _ci.v1);

        double err = q(_ci.p1) / double(q.getCount());

        return float((err < max_err_) ? err : float(Base::ILLEGAL_COLLAPSE));
    }

    /// Post-process halfedge collapse (accumulate quadrics)
    virtual void postprocess_collapse(const CollapseInfo& _ci)
    {
        Base::mesh().property(quadrics_, _ci.v1) += Base::mesh().property(quadrics_, _ci.v0);
    }

    /// set the percentage of maximum quadric error
    void set_error_tolerance_factor(double _factor);

  public:  // specific methods
    /** Set maximum quadric error constraint and enable binary mode.
     *  \param _err    Maximum error allowed
     *  \param _binary Let the module work in non-binary mode in spite of the
     *                 enabled constraint.
     *  \see unset_max_err()
     */
    void set_max_err(double _err, bool _binary = true)
    {
        max_err_ = _err;
        Base::set_binary(_binary);
    }

    /// Unset maximum quadric error constraint and restore non-binary mode.
    /// \see set_max_err()
    void unset_max_err(void)
    {
        max_err_ = DBL_MAX;
        Base::set_binary(false);
    }

    /// Return value of max. allowed error.
    double max_err() const { return max_err_; }

  private:
    // maximum quadric error
    double max_err_;

    // this vertex property stores a quadric for each vertex
    VPropHandleT<Geometry::QuadricMetricT<double>> quadrics_;
};

//=============================================================================
}  // namespace Decimater
}  // namespace OpenMesh
//=============================================================================

    #if !defined(OPENMESH_DECIMATER_MODQUADRICMETRIC_CC)
        #define OSG_MODQUADRIC_TEMPLATES
        #include "ModQuadricMetricT.cpp"
    #endif

//=============================================================================
#endif  // OSG_MODQUADRIC_HH defined
//=============================================================================
