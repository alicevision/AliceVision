// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "fwd.hpp"
#include "path.hpp"
#include "directory_entry.hpp"

namespace aliceVision {
namespace vfs {

class directory_iterator
{
public:
    using value_type = directory_entry;
    using difference_type = std::ptrdiff_t;
    using pointer = const directory_entry*;
    using reference = const directory_entry&;
    using iterator_category = std::input_iterator_tag;

    directory_iterator() noexcept;
    directory_iterator(const directory_iterator& other);
    explicit directory_iterator(const path& p, directory_options opts = directory_options::none);
    ~directory_iterator();

    directory_iterator& operator=(const directory_iterator& other);
    directory_iterator& operator++();

    const directory_entry& operator*() const;
    const directory_entry* operator->() const;

    bool operator==(const directory_iterator& other) const
    {
        return _it == other._it;
    }

    bool operator!=(const directory_iterator& other) const
    {
        return _it != other._it;
    }

private:
    boost::filesystem::directory_iterator _it;
    mutable bool _isCached = false;
    mutable directory_entry _entry;
};

directory_iterator begin(const directory_iterator& iter);
directory_iterator end(const directory_iterator& iter);

} // namespace vfs
} // namespace aliceVision
