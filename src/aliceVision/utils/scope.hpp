// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <utility>
#include <functional>

namespace aliceVision {
namespace utils {

template <typename Fn>
class ScopeGuard {
public:
    explicit ScopeGuard(Fn&& fn) : fn_(std::forward<Fn>(fn)) {}
    ~ScopeGuard() { fn_(); }

    ScopeGuard(const ScopeGuard&) = delete;
    ScopeGuard& operator=(const ScopeGuard&) = delete;

private:
    Fn fn_;
};

}
}
