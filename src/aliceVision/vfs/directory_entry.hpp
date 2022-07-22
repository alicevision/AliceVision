// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "path.hpp"

namespace aliceVision {
namespace vfs {

class directory_entry
{
public:
    directory_entry();
    directory_entry(const directory_entry& other);
    directory_entry(const boost::filesystem::directory_entry& other);
    explicit directory_entry(const vfs::path& p, file_status status = file_status(),
                             file_status symlink_status = file_status());
    ~directory_entry();

    directory_entry& operator=(const directory_entry& other);
    void assign(const path& p, file_status status = file_status(),
                file_status symlink_status = file_status());

    void replace_filename(const path& p, file_status status = file_status(),
                          file_status symlink_status = file_status());

    vfs::path path() const;
    file_status status() const;
    file_status symlink_status() const;

    bool operator==(const directory_entry& rhs) const;
    bool operator!=(const directory_entry& rhs) const;
    bool operator< (const directory_entry& rhs) const;
    bool operator<=(const directory_entry& rhs) const;
    bool operator> (const directory_entry& rhs) const;
    bool operator>=(const directory_entry& rhs) const;

private:
    boost::filesystem::directory_entry _entry;
};

} // namespace vfs
} // namespace aliceVision
