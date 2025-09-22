// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <std_shared_ptr.i>
%shared_ptr(aliceVision::camera::IntrinsicBase)

%{
#include <aliceVision/camera/IntrinsicBase.hpp>
#include <aliceVision/camera/IntrinsicScaleOffset.hpp>
#include <aliceVision/camera/IntrinsicScaleOffsetDisto.hpp>
%}

%include <aliceVision/camera/IntrinsicBase.hpp>

%extend aliceVision::camera::IntrinsicBase {
    bool isIntrinsicScaleOffset() const {
        return dynamic_cast<const aliceVision::camera::IntrinsicScaleOffset*>($self) != nullptr;
    }

    aliceVision::camera::IntrinsicScaleOffset* asIntrinsicScaleOffset() {
        return dynamic_cast<aliceVision::camera::IntrinsicScaleOffset*>($self);
    }

    bool isIntrinsicScaleOffsetDisto() const {
        return dynamic_cast<const aliceVision::camera::IntrinsicScaleOffsetDisto*>($self) != nullptr;
    }

    aliceVision::camera::IntrinsicScaleOffsetDisto* asIntrinsicScaleOffsetDisto() {
        return dynamic_cast<aliceVision::camera::IntrinsicScaleOffsetDisto*>($self);
    }
}
