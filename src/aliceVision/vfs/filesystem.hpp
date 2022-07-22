// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "path.hpp"
#include "directory_iterator.hpp"

namespace aliceVision {
namespace vfs {

class istream;
class ostream;

/**
 * This is a wrapper for boost::filesystem that also optionally implements a virtual in-memory file
 * system at paths starting with memory://. This allows to avoid unnecessary I/O in cases where
 * whole AliceVision pipeline is in a single process, such as on Apple iOS which doesn't support
 * running more than a single process.
 */
class filesystem
{
public:
    istream open_read_binary(const path& p);
    istream open_read_text(const path& p);
    istream open_read(const path& p, std::ios::openmode mode);
    ostream open_write_binary(const path& p);
    ostream open_write_text(const path& p);
    ostream open_write(const path& p, std::ios::openmode mode);

    // Returns true if the path refers to within virtual filesystem.
    bool is_virtual_path(const path& p);

    path absolute(const path& p);
    path absolute(const path& p, const path& base);

    path canonical(const path& p);
    path canonical(const path& p, error_code& ec);
    path canonical(const path& p, const path& base);
    path canonical(const path& p, const path& base, error_code& ec);

    void copy(const path& from, const path& to);
    void copy(const path& from, const path& to, error_code& ec);
    void copy_directory(const path& from, const path& to);
    void copy_directory(const path& from, const path& to, error_code& ec);

    void copy_file(const path& from, const path& to);
    void copy_file(const path& from, const path& to, error_code& ec);
    void copy_file(const path& from, const path& to, copy_option option);
    void copy_file(const path& from, const path& to, copy_option option, error_code& ec);

    void copy_symlink(const path& existing_symlink, const path& new_symlink);
    void copy_symlink(const path& existing_symlink, const path& new_symlink, error_code& ec);

    bool create_directories(const path& p);
    bool create_directories(const path& p, error_code& ec);
    bool create_directory(const path& p);
    bool create_directory(const path& p, error_code& ec);

    void create_directory_symlink(const path& to, const path& new_symlink);
    void create_directory_symlink(const path& to, const path& new_symlink, error_code& ec);
    void create_hard_link(const path& to, const path& new_hard_link);
    void create_hard_link(const path& to, const path& new_hard_link, error_code& ec);

    void create_symlink(const path& to, const path& new_symlink);
    void create_symlink(const path& to, const path& new_symlink, error_code& ec);

    path current_path();
    path current_path(error_code& ec);
    void current_path(const path& p);
    void current_path(const path& p, error_code& ec);

    bool exists(file_status s) noexcept;
    bool exists(const path& p);
    bool exists(const path& p, error_code& ec);

    bool equivalent(const path& p1, const path& p2);
    bool equivalent(const path& p1, const path& p2, error_code& ec);

    std::uintmax_t file_size(const path& p);
    std::uintmax_t file_size(const path& p, error_code& ec);
    std::uintmax_t hard_link_count(const path& p);
    std::uintmax_t hard_link_count(const path& p, error_code& ec);

    bool is_directory(file_status s) noexcept;
    bool is_directory(const path& p);
    bool is_directory(const path& p, error_code& ec);

    bool is_empty(const path& p);
    bool is_empty(const path& p, error_code& ec);

    bool is_other(file_status s) noexcept;
    bool is_other(const path& p);
    bool is_other(const path& p, error_code& ec);

    bool is_regular_file(file_status s) noexcept;
    bool is_regular_file(const path& p);
    bool is_regular_file(const path& p, error_code& ec);

    bool is_symlink(file_status s);
    bool is_symlink(const path& p);
    bool is_symlink(const path& p, error_code& ec);

    std::time_t last_write_time(const path& p);
    std::time_t last_write_time(const path& p, error_code& ec);
    void last_write_time(const path& p, std::time_t new_time);
    void last_write_time(const path& p, std::time_t new_time, error_code& ec);

    path read_symlink(const path& p);
    path read_symlink(const path& p, error_code& ec);

    path relative(const path& p);
    path relative(const path& p, error_code& ec);
    path relative(const path& p, const path& base);
    path relative(const path& p, const path& base, error_code& ec);

    bool remove(const path& p);
    bool remove(const path& p, error_code& ec);

    std::uintmax_t remove_all(const path& p);
    std::uintmax_t remove_all(const path& p, error_code& ec);

    void rename(const path& from, const path& to);
    void rename(const path& from, const path& to, error_code& ec);

    void resize_file(const path& p, std::uintmax_t size);
    void resize_file(const path& p, std::uintmax_t size, error_code& ec);

    space_info space(const path& p);
    space_info space(const path& p, error_code& ec);

    file_status status(const path& p);
    file_status status(const path& p, error_code& ec);

    bool status_known(file_status s) noexcept;

    file_status symlink_status(const path& p);
    file_status symlink_status(const path& p, error_code& ec);

    path system_complete(const path& p);
    path system_complete(const path& p, error_code& ec);

    path temp_directory_path();
    path temp_directory_path(error_code& ec);

    path unique_path(const path& model="%%%%-%%%%-%%%%-%%%%");
    path unique_path(const path& model, error_code& ec);

    path weakly_canonical(const path& p);
    path weakly_canonical(const path& p, error_code& ec);
};

} //namespace vfs
} //namespace aliceVision

