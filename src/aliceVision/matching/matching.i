// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

%module (module="pyalicevision") matching

%include <aliceVision/config.hpp>
%include <aliceVision/global.i>
%include <std_map.i>
%include <std_vector.i>

%{    
#include <aliceVision/matching/io.hpp>

using namespace aliceVision;
using namespace aliceVision::matching;
%} 

%ignore aliceVision::matching::operator<<;
%ignore aliceVision::matching::operator>>;



%include <aliceVision/matching/IndMatch.hpp>




%template(IndMatches) std::vector<aliceVision::matching::IndMatch>;
%template(MatchesPerDesc) std::map<enum aliceVision::feature::EImageDescriberType, aliceVision::matching::IndMatches>;

%include <aliceVision/feature/imageDescriberCommon.hpp>
%include <aliceVision/matching/MatchesCollections.hpp>
%include <aliceVision/matching/io.hpp>

%template(EImageDescriberTypeVector) std::vector<enum aliceVision::feature::EImageDescriberType>;

%template(PairwiseMatches) std::map<aliceVision::Pair, aliceVision::matching::MatchesPerDescType>;
