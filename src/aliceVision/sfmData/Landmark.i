// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <aliceVision/numeric/eigen.i>
%eigen_typemaps(Vec3)

%include <aliceVision/sfmData/Observation.i>

// We don't want to have a direct access to rgb
// It's easier to wrap it around a Vec3
// As it's already binded to numpy
%ignore aliceVision::sfmData::Landmark::_rgb;

// Private members should not be directly accessible
%ignore aliceVision::sfmData::Landmark::_state;
%ignore aliceVision::sfmData::Landmark::_descType;
%ignore aliceVision::sfmData::Landmark::_observations;
%ignore aliceVision::sfmData::Landmark::_parallaxRobust;
%ignore aliceVision::sfmData::Landmark::_isLocked;
%ignore aliceVision::sfmData::Landmark::_X;
%ignore aliceVision::sfmData::Landmark::_pointFetcher;

//Add new swig only C++ code
%extend aliceVision::sfmData::Landmark {

    // From RgbColor to Vec3
    Vec3 getRgbVec3() const {
        Vec3 ret;

        ret.x() = $self->getRgb().r();
        ret.y() = $self->getRgb().g();
        ret.z() = $self->getRgb().b();

        return ret;
    }
    
    // From Vec3 to RgbColor
    void setRgbVec3(Vec3 value) {
        image::RGBColor color;
        color.r() = value.x();
        color.g() = value.y();
        color.b() = value.z();
        $self->setRgb(color);
    }
    
    // If the user asks for the rgb property
    // It is not a direct access to the rgb
    // variable but a getter/setter
    %pythoncode %{
        rgb = property(getRgbVec3, setRgbVec3)
        state = property(getState, setState)
        descType = property(getDescType, setDescType)
        X = property(getX, setX)
    %}
}

%include <aliceVision/sfmData/Landmark.hpp>

%{
#include <aliceVision/sfmData/Landmark.hpp>
%}