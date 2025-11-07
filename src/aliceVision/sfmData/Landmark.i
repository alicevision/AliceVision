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
%ignore aliceVision::sfmData::Landmark::rgb;

//Add new swig only C++ code
%extend aliceVision::sfmData::Landmark {

    // From RgbColor to Vec3
    Vec3 getRgb() const {
        Vec3 ret;

        ret.x() = $self->rgb.r();
        ret.y() = $self->rgb.g();
        ret.z() = $self->rgb.b();

        return ret;
    }
    
    // From Vec3 to RgbColor
    void setRgb(Vec3 value) {
        $self->rgb.r() = value.x();
        $self->rgb.g() = value.y();
        $self->rgb.b() = value.z();
    }
    
    // If the user asks for the rgb property
    // It is not a direct access to the rgb
    // variable but a getter/setter
    %pythoncode %{
        rgb = property(getRgb, setRgb)
    %}
}

%include <aliceVision/sfmData/Landmark.hpp>

%{
#include <aliceVision/sfmData/Landmark.hpp>
%}