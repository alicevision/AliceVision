// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <aliceVision/sfmData/ExposureSetting.hpp>

%{
#include <aliceVision/sfmData/ExposureSetting.hpp>
%}

// Python version of the overloading of the "<<" operator
%extend aliceVision::sfmData::ExposureSetting {
    %pythoncode %{
        def __str__(self):
            return "shutter: {}, fnumber: {}, iso: {}".format(self._shutter, self._fnumber, self._iso)
    %}
};

%template(ExposureSettingVector) std::vector<aliceVision::sfmData::ExposureSetting>;