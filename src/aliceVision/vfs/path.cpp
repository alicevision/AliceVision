// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "path.hpp"

namespace aliceVision {
namespace vfs {

path::path() = default;

path::path(const path& p) : _path{p._path}
{}

path::path(const std::string& s) : _path{s}
{}

path::path(const char* s) : _path{s}
{}

path::path(path&& p) : _path{std::move(p._path)}
{}

path::path(const boost::filesystem::path& p) : _path{p}
{}

path::path(boost::filesystem::path&& p) : _path{std::move(p)}
{}

path& path::operator=(path&& p)
{
    _path = std::move(p._path);
    return *this;
}

path& path::operator=(const path& p)
{
    _path = p._path;
    return *this;
}

path& path::operator=(const std::string& s)
{
    _path = s;
    return *this;
}

path& path::operator=(const char* s)
{
    _path = s;
    return *this;
}

path& path::operator+=(const path& p)
{
    _path += p._path;
    return *this;
}

path& path::operator+=(const std::string& s)
{
    _path += s;
    return *this;
}

path& path::operator+=(const char* s)
{
    _path += s;
    return *this;
}

path& path::operator/=(const path& p)
{
    _path /= p._path;
    return *this;
}

path& path::operator/=(const std::string& s)
{
    _path /= s;
    return *this;
}

path& path::operator/=(const char* s)
{
    _path /= s;
    return *this;
}

path& path::append(const std::string& source)
{
    _path.append(source);
    return *this;
}

path& path::append(const char* source)
{
    _path.append(source);
    return *this;
}

path& path::make_preferred()
{
    _path.make_preferred();
    return *this;
}

path& path::remove_filename()
{
    _path.remove_filename();
    return *this;
}

path& path::remove_trailing_separator()
{
    _path.remove_trailing_separator();
    return *this;
}

path& path::replace_extension(const path& new_extension)
{
    _path.replace_extension(new_extension._path);
    return *this;
}

void path::swap(path& rhs)
{
    _path.swap(rhs._path);
}

std::string path::string() const
{
    return _path.string();
}

std::string path::generic_string() const
{
    return _path.generic_string();
}

boost::filesystem::path path::boost_path() const
{
    return _path;
}

int path::compare(const path& p) const
{
    return _path.compare(p._path);
}

int path::compare(const std::string& s) const
{
    return _path.compare(s);
}

path path::root_path() const
{
    return _path.root_path();
}

path path::root_name() const
{
    return _path.root_name();
}

path path::root_directory() const
{
    return _path.root_directory();
}

path path::relative_path() const
{
    return _path.relative_path();
}

path path::parent_path() const
{
    return _path.parent_path();
}

path path::filename() const
{
    return _path.filename();
}

path path::stem() const
{
    return _path.stem();
}

path path::extension() const
{
    return _path.extension();
}

bool path::empty() const
{
    return _path.empty();
}

bool path::filename_is_dot() const
{
    return _path.filename_is_dot();
}

bool path::filename_is_dot_dot() const
{
    return _path.filename_is_dot_dot();
}

bool path::has_root_path() const
{
    return _path.has_root_path();
}

bool path::has_root_name() const
{
    return _path.has_root_name();
}

bool path::has_root_directory() const
{
    return _path.has_root_directory();
}

bool path::has_relative_path() const
{
    return _path.has_relative_path();
}

bool path::has_parent_path() const
{
    return _path.has_parent_path();
}

bool path::has_filename() const
{
    return _path.has_filename();
}

bool path::has_stem() const
{
    return _path.has_stem();
}

bool path::has_extension() const
{
    return _path.has_extension();
}

bool path::is_relative() const
{
    return _path.is_relative();
}

bool path::is_absolute() const
{
    return _path.is_absolute();
}

bool path::operator==(const path& rhs) const { return _path == rhs._path; }
bool path::operator!=(const path& rhs) const { return _path != rhs._path; }
bool path::operator< (const path& rhs) const { return _path < rhs._path; }
bool path::operator<=(const path& rhs) const { return _path <= rhs._path; }
bool path::operator> (const path& rhs) const { return _path > rhs._path; }
bool path::operator>=(const path& rhs) const { return _path >= rhs._path; }

std::istream& operator>>(std::istream& str, path& path)
{
    str >> path._path;
    return str;
}

std::ostream& operator<<(std::ostream& str, const path& path)
{
    str << path._path;
    return str;
}

} //namespace vfs
} //namespace aliceVision
