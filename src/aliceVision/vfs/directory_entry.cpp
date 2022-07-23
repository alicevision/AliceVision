// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "directory_entry.hpp"

namespace aliceVision {
namespace vfs {

directory_entry::directory_entry() = default;

directory_entry::directory_entry(const directory_entry& other) : _entry{other._entry}
{}

directory_entry::directory_entry(const vfs::path& p, file_status status,
                                 file_status symlink_status) :
    _entry{p.boost_path(), status, symlink_status}
{}

directory_entry::directory_entry(const boost::filesystem::directory_entry& other) : _entry{other}
{}

directory_entry::~directory_entry() = default;

directory_entry& directory_entry::operator=(const directory_entry& other)
{
    _entry = other._entry;
    return *this;
}

void directory_entry::assign(const vfs::path& p, file_status status, file_status symlink_status)
{
    _entry.assign(p.boost_path(), status, symlink_status);
}

void directory_entry::replace_filename(const vfs::path& p, file_status status,
                                       file_status symlink_status)
{
    _entry.replace_filename(p.boost_path(), status, symlink_status);
}

vfs::path directory_entry::path() const
{
    return _entry.path();
}

file_status directory_entry::status() const
{
    return _entry.status();
}

file_status directory_entry::symlink_status() const
{
    return _entry.symlink_status();
}

bool directory_entry::operator==(const directory_entry& rhs) const { return _entry == rhs._entry; }
bool directory_entry::operator!=(const directory_entry& rhs) const { return _entry != rhs._entry; }
bool directory_entry::operator< (const directory_entry& rhs) const { return _entry < rhs._entry; }
bool directory_entry::operator<=(const directory_entry& rhs) const { return _entry <= rhs._entry; }
bool directory_entry::operator> (const directory_entry& rhs) const { return _entry > rhs._entry; }
bool directory_entry::operator>=(const directory_entry& rhs) const { return _entry >= rhs._entry; }

} // namespace vfs
} // namespace aliceVision
