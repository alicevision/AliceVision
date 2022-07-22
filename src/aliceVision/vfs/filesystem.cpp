// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include "filesystem.hpp"
#include "istream.hpp"
#include "ostream.hpp"

namespace aliceVision {
namespace vfs {

istream filesystem::open_read_binary(const path& p)
{
    return open_read(p, std::ios_base::binary);
}

istream filesystem::open_read_text(const path& p)
{
    return open_read(p, {});
}

istream filesystem::open_read(const path& p, std::ios::openmode mode)
{
    auto buffer = std::make_unique<std::filebuf>();
    buffer->open(p.string(), std::ios_base::in | mode);
    return istream(std::move(buffer));
}

ostream filesystem::open_write_binary(const path& p)
{
    return open_write(p, std::ios_base::binary);
}

ostream filesystem::open_write_text(const path& p)
{
    return open_write(p, {});
}

ostream filesystem::open_write(const path& p, std::ios::openmode mode)
{
    auto buffer = std::make_unique<std::filebuf>();
    buffer->open(p.string(), std::ios_base::out | mode);
    return ostream(std::move(buffer));
}

bool filesystem::is_virtual_path(const path& p)
{
    return false;
}

path filesystem::absolute(const path& p)
{
    return boost::filesystem::absolute(p.boost_path());
}

path filesystem::absolute(const path& p, const path& base)
{
    return boost::filesystem::absolute(p.boost_path(), base.boost_path());
}

path filesystem::canonical(const path& p)
{
    return boost::filesystem::canonical(p.boost_path());
}

path filesystem::canonical(const path& p, error_code& ec)
{
    return boost::filesystem::canonical(p.boost_path(), ec);
}

path filesystem::canonical(const path& p, const path& base)
{
    return boost::filesystem::canonical(p.boost_path(), base.boost_path());
}

path filesystem::canonical(const path& p, const path& base, error_code& ec)
{
    return boost::filesystem::canonical(p.boost_path(), base.boost_path(), ec);
}

void filesystem::copy(const path& from, const path& to)
{
    boost::filesystem::copy(from.boost_path(), to.boost_path());
}

void filesystem::copy(const path& from, const path& to, error_code& ec)
{
    boost::filesystem::copy(from.boost_path(), to.boost_path(), ec);
}

void filesystem::copy_directory(const path& from, const path& to)
{
    boost::filesystem::copy_directory(from.boost_path(), to.boost_path());
}

void filesystem::copy_directory(const path& from, const path& to, error_code& ec)
{
    boost::filesystem::copy_directory(from.boost_path(), to.boost_path(), ec);
}

void filesystem::copy_file(const path& from, const path& to)
{
    boost::filesystem::copy_file(from.boost_path(), to.boost_path());
}

void filesystem::copy_file(const path& from, const path& to, error_code& ec)
{
    boost::filesystem::copy_file(from.boost_path(), to.boost_path(), ec);
}

void filesystem::copy_file(const path& from, const path& to, copy_option option)
{
    boost::filesystem::copy_file(from.boost_path(), to.boost_path(), option);
}

void filesystem::copy_file(const path& from, const path& to, copy_option option, error_code& ec)
{
    boost::filesystem::copy_file(from.boost_path(), to.boost_path(), option, ec);
}

void filesystem::copy_symlink(const path& existing_symlink, const path& new_symlink)
{
    boost::filesystem::copy_symlink(existing_symlink.boost_path(), new_symlink.boost_path());
}

void filesystem::copy_symlink(const path& existing_symlink, const path& new_symlink, error_code& ec)
{
    boost::filesystem::copy_symlink(existing_symlink.boost_path(), new_symlink.boost_path(), ec);
}

bool filesystem::create_directories(const path& p)
{
    return boost::filesystem::create_directories(p.boost_path());
}

bool filesystem::create_directories(const path& p, error_code& ec)
{
    return boost::filesystem::create_directories(p.boost_path(), ec);
}

bool filesystem::create_directory(const path& p)
{
    return boost::filesystem::create_directory(p.boost_path());
}

bool filesystem::create_directory(const path& p, error_code& ec)
{
    return boost::filesystem::create_directory(p.boost_path(), ec);
}

void filesystem::create_directory_symlink(const path& to, const path& new_symlink)
{
    boost::filesystem::create_directory_symlink(to.boost_path(), new_symlink.boost_path());
}

void filesystem::create_directory_symlink(const path& to, const path& new_symlink, error_code& ec)
{
    boost::filesystem::create_directory_symlink(to.boost_path(), new_symlink.boost_path(), ec);
}

void filesystem::create_hard_link(const path& to, const path& new_hard_link)
{
    boost::filesystem::create_hard_link(to.boost_path(), new_hard_link.boost_path());
}

void filesystem::create_hard_link(const path& to, const path& new_hard_link, error_code& ec)
{
    boost::filesystem::create_hard_link(to.boost_path(), new_hard_link.boost_path(), ec);
}

void filesystem::create_symlink(const path& to, const path& new_symlink)
{
    boost::filesystem::create_symlink(to.boost_path(), new_symlink.boost_path());
}

void filesystem::create_symlink(const path& to, const path& new_symlink, error_code& ec)
{
    boost::filesystem::create_symlink(to.boost_path(), new_symlink.boost_path(), ec);
}

path filesystem::current_path()
{
    return boost::filesystem::current_path();
}

path filesystem::current_path(error_code& ec)
{
    return boost::filesystem::current_path(ec);
}

void filesystem::current_path(const path& p)
{
    boost::filesystem::current_path(p.boost_path());
}

void filesystem::current_path(const path& p, error_code& ec)
{
    boost::filesystem::current_path(p.boost_path(), ec);
}

bool filesystem::exists(file_status s) noexcept
{
    return boost::filesystem::exists(s);
}

bool filesystem::exists(const path& p)
{
    return boost::filesystem::exists(p.boost_path());
}

bool filesystem::exists(const path& p, error_code& ec)
{
    return boost::filesystem::exists(p.boost_path(), ec);
}

bool filesystem::equivalent(const path& p1, const path& p2)
{
    return boost::filesystem::equivalent(p1.boost_path(), p2.boost_path());
}

bool filesystem::equivalent(const path& p1, const path& p2, error_code& ec)
{
    return boost::filesystem::equivalent(p1.boost_path(), p2.boost_path(), ec);
}

std::uintmax_t filesystem::file_size(const path& p)
{
    return boost::filesystem::file_size(p.boost_path());
}

std::uintmax_t filesystem::file_size(const path& p, error_code& ec)
{
    return boost::filesystem::file_size(p.boost_path(), ec);
}

std::uintmax_t filesystem::hard_link_count(const path& p)
{
    return boost::filesystem::hard_link_count(p.boost_path());
}

std::uintmax_t filesystem::hard_link_count(const path& p, error_code& ec)
{
    return boost::filesystem::hard_link_count(p.boost_path(), ec);
}

bool filesystem::is_directory(file_status s) noexcept
{
    return boost::filesystem::is_directory(s);
}

bool filesystem::is_directory(const path& p)
{
    return boost::filesystem::is_directory(p.boost_path());
}

bool filesystem::is_directory(const path& p, error_code& ec)
{
    return boost::filesystem::is_directory(p.boost_path(), ec);
}

bool filesystem::is_empty(const path& p)
{
    return boost::filesystem::is_empty(p.boost_path());
}

bool filesystem::is_empty(const path& p, error_code& ec)
{
    return boost::filesystem::is_empty(p.boost_path(), ec);
}

bool filesystem::is_other(file_status s) noexcept
{
    return boost::filesystem::is_other(s);
}

bool filesystem::is_other(const path& p)
{
    return boost::filesystem::is_other(p.boost_path());
}

bool filesystem::is_other(const path& p, error_code& ec)
{
    return boost::filesystem::is_other(p.boost_path(), ec);
}

bool filesystem::is_regular_file(file_status s) noexcept
{
    return boost::filesystem::is_regular_file(s);
}

bool filesystem::is_regular_file(const path& p)
{
    return boost::filesystem::is_regular_file(p.boost_path());
}

bool filesystem::is_regular_file(const path& p, error_code& ec)
{
    return boost::filesystem::is_regular_file(p.boost_path(), ec);
}

bool filesystem::is_symlink(file_status s)
{
    return boost::filesystem::is_symlink(s);
}

bool filesystem::is_symlink(const path& p)
{
    return boost::filesystem::is_symlink(p.boost_path());
}

bool filesystem::is_symlink(const path& p, error_code& ec)
{
    return boost::filesystem::is_symlink(p.boost_path(), ec);
}

std::time_t filesystem::last_write_time(const path& p)
{
    return boost::filesystem::last_write_time(p.boost_path());
}

std::time_t filesystem::last_write_time(const path& p, error_code& ec)
{
    return boost::filesystem::last_write_time(p.boost_path(), ec);
}

void filesystem::last_write_time(const path& p, const std::time_t new_time)
{
    boost::filesystem::last_write_time(p.boost_path(), new_time);
}

void filesystem::last_write_time(const path& p, const std::time_t new_time, error_code& ec)
{
    boost::filesystem::last_write_time(p.boost_path(), new_time, ec);
}

path filesystem::read_symlink(const path& p)
{
    return boost::filesystem::read_symlink(p.boost_path());
}

path filesystem::read_symlink(const path& p, error_code& ec)
{
    return boost::filesystem::read_symlink(p.boost_path(), ec);
}

path filesystem::relative(const path& p)
{
    return boost::filesystem::relative(p.boost_path());
}

path filesystem::relative(const path& p, error_code& ec)
{
    return boost::filesystem::relative(p.boost_path(), ec);
}

path filesystem::relative(const path& p, const path& base)
{
    return boost::filesystem::relative(p.boost_path(), base.boost_path());
}

path filesystem::relative(const path& p, const path& base, error_code& ec)
{
    return boost::filesystem::relative(p.boost_path(), base.boost_path(), ec);
}

bool filesystem::remove(const path& p)
{
    return boost::filesystem::remove(p.boost_path());
}

bool filesystem::remove(const path& p, error_code& ec)
{
    return boost::filesystem::remove(p.boost_path(), ec);
}

std::uintmax_t filesystem::remove_all(const path& p)
{
    return boost::filesystem::remove_all(p.boost_path());
}

std::uintmax_t filesystem::remove_all(const path& p, error_code& ec)
{
    return boost::filesystem::remove_all(p.boost_path(), ec);
}

void filesystem::rename(const path& from, const path& to)
{
    boost::filesystem::rename(from.boost_path(), to.boost_path());
}

void filesystem::rename(const path& from, const path& to, error_code& ec)
{
    boost::filesystem::rename(from.boost_path(), to.boost_path(), ec);
}

void filesystem::resize_file(const path& p, std::uintmax_t size)
{
    boost::filesystem::resize_file(p.boost_path(), size);
}

void filesystem::resize_file(const path& p, std::uintmax_t size, error_code& ec)
{
    boost::filesystem::resize_file(p.boost_path(), size, ec);
}

space_info filesystem::space(const path& p)
{
    return boost::filesystem::space(p.boost_path());
}

space_info filesystem::space(const path& p, error_code& ec)
{
    return boost::filesystem::space(p.boost_path(), ec);
}

file_status filesystem::status(const path& p)
{
    return boost::filesystem::status(p.boost_path());
}

file_status filesystem::status(const path& p, error_code& ec)
{
    return boost::filesystem::status(p.boost_path(), ec);
}

bool filesystem::status_known(file_status s) noexcept
{
    return boost::filesystem::status_known(s);
}

file_status filesystem::symlink_status(const path& p)
{
    return boost::filesystem::symlink_status(p.boost_path());
}

file_status filesystem::symlink_status(const path& p, error_code& ec)
{
    return boost::filesystem::symlink_status(p.boost_path(), ec);
}

path filesystem::system_complete(const path& p)
{
    return boost::filesystem::system_complete(p.boost_path());
}

path filesystem::system_complete(const path& p, error_code& ec)
{
    return boost::filesystem::system_complete(p.boost_path(), ec);
}

path filesystem::temp_directory_path()
{
    return boost::filesystem::temp_directory_path();
}

path filesystem::temp_directory_path(error_code& ec)
{
    return boost::filesystem::temp_directory_path(ec);
}

path filesystem::unique_path(const path& model)
{
    return boost::filesystem::unique_path(model.boost_path());
}

path filesystem::unique_path(const path& model, error_code& ec)
{
    return boost::filesystem::unique_path(model.boost_path(), ec);
}

path filesystem::weakly_canonical(const path& p)
{
    return boost::filesystem::weakly_canonical(p.boost_path());
}

path filesystem::weakly_canonical(const path& p, error_code& ec)
{
    return boost::filesystem::weakly_canonical(p.boost_path(), ec);
}

} //namespace vfs
} //namespace aliceVision
