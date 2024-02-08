// This file is part of the AliceVision project.
// Copyright (c) 2024 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%include <stdint.i>
%include <std_container.i>
%include <std_string.i>
%include <std_pair.i>
%include <std_shared_ptr.i>
%include <std_vector.i>
%include <std_set.i>
%include <std_map.i>
%include <stl.i>


%include <aliceVision/types.hpp>

%{
#include <aliceVision/types.hpp>
#include <memory>
%}

%inline %{
    typedef long unsigned int size_t;
    typedef uint32_t IndexT;
%}


%template(IntVector) std::vector<int>;
%template(DoubleVector) std::vector<double>;
%template(StringVector) std::vector<std::string>;
%template(IndexTVector) std::vector<IndexT>;

%template(StringStringMap) std::map<std::string, std::string>;

%template(IndexTSet) std::set<IndexT>;

%template(SizeTPair) std::pair<size_t, size_t>;

// As defined in aliceVision/types.hpp
%template(Pair) std::pair<IndexT, IndexT>;
%template(PairSet) std::set<aliceVision::Pair>;
%template(PairVec) std::vector<aliceVision::Pair>;
