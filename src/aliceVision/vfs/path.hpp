// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "boost_common.hpp"

namespace aliceVision {
namespace vfs {

/**
 * This is a wrapper for boost::filesystem::path that also understands virtual in-memory file
 * system at paths starting with memory://.
 *
 * Note: only a subset of functionality supported by boost::filesystem::path is exposed. In
 * particular, wide strings are not supported as they are not used within AliceVision.
 */
class path
{
public:
    path();
    path(const path& p);
    path(const std::string& s);
    path(const char* s);
    path(path&& p);

    path(const boost::filesystem::path& p);
    path(boost::filesystem::path&& p);

    path& operator=(path&& p);

    path& operator=(const path& p);
    path& operator=(const std::string& s);
    path& operator=(const char* s);

    path& operator+=(const path& p);
    path& operator+=(const std::string& s);
    path& operator+=(const char* s);

    path& operator/=(const path& p);

    path& operator/=(const std::string& s);
    path& operator/=(const char* s);

    path& append(const std::string& source);
    path& append(const char* source);

    path& make_preferred();

    path& remove_filename();
    path& remove_trailing_separator();
    path& replace_extension(const path& new_extension = path());
    void swap(path& rhs);

    std::string string() const;
    std::string generic_string() const;
    boost::filesystem::path boost_path() const;

    int compare(const path& p) const;
    int compare(const std::string& s) const;

    path root_path() const;
    path root_name() const;

    path root_directory() const;
    path relative_path() const;
    path parent_path() const;
    path filename() const;
    path stem() const;
    path extension() const;

    bool empty() const;
    bool filename_is_dot() const;
    bool filename_is_dot_dot() const;
    bool has_root_path() const;
    bool has_root_name() const;
    bool has_root_directory() const;
    bool has_relative_path() const;
    bool has_parent_path() const;
    bool has_filename() const;
    bool has_stem() const;
    bool has_extension() const;
    bool is_relative() const;
    bool is_absolute() const;

    bool operator==(const path& rhs) const;
    bool operator!=(const path& rhs) const;
    bool operator< (const path& rhs) const;
    bool operator<=(const path& rhs) const;
    bool operator> (const path& rhs) const;
    bool operator>=(const path& rhs) const;

    /*class iterator;
    typedef iterator const_iterator;
    class reverse_iterator;
    typedef reverse_iterator const_reverse_iterator;

    BOOST_FILESYSTEM_DECL iterator begin() const;
    BOOST_FILESYSTEM_DECL iterator end() const;
    reverse_iterator rbegin() const;
    reverse_iterator rend() const;*/

    friend std::istream& operator>>(std::istream& str, path& path);
    friend std::ostream& operator<<(std::ostream& str, const path& path);

private:
    boost::filesystem::path _path;
};

inline void swap(path& lhs, path& rhs)
{
    lhs.swap(rhs);
}

inline path operator/(const path& lhs, const path& rhs)
{
    path p = lhs;
    p /= rhs;
    return p;
}

inline path operator/(path&& lhs, const path& rhs)
{
    lhs /= rhs;
    return std::move(lhs);
}

std::istream& operator>>(std::istream& str, path& path);
std::ostream& operator<<(std::ostream& str, const path& path);

} //namespace vfs
} //namespace aliceVision

