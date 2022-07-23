// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <istream>
#include <fstream>
#include <memory>

namespace aliceVision {
namespace vfs {

class istream : public std::istream {
public:
    istream(const istream& other) = delete;

    istream(istream&& other) :
        std::istream{std::move(other)},
        _buffer{std::move(other._buffer)}
    {
        set_rdbuf(_buffer.get());
    }

    istream& operator=(istream&& other)
    {
        std::istream::operator=(std::move(other));
        _buffer = std::move(other._buffer);
        return *this;
    }

    std::streambuf* rdbuf() const
    {
        return _buffer.get();
    }

    bool is_open() const
    {
        return _buffer->is_open();
    }

    void close()
    {
        if (!_buffer->close())
        {
            setstate(std::ios_base::failbit);
        }
    }

    void swap(istream& other)
    {
        std::istream::swap(other);
        _buffer.swap(other._buffer);
    }

    ~istream() override {}

    // Helpers for compatibility with code using C IO APIs
    std::size_t fread(void* buffer, std::size_t size, std::size_t count)
    {
        auto total_size = size * count;
        read(reinterpret_cast<char*>(buffer), total_size);
        return gcount() / size; // truncation towards zero is expected and appropriate in this case
    }

private:
    friend class filesystem;
    istream(std::unique_ptr<std::filebuf>&& buffer) :
        std::istream{buffer.get()},
        _buffer{std::move(buffer)}
    {
    }

    std::unique_ptr<std::filebuf> _buffer;
};

} // namespace vfs
} // namespace aliceVision
