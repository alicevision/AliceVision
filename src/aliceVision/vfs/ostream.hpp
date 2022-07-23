// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include "path.hpp"
#include "null_streambuf.hpp"
#include <cstdarg>
#include <ostream>
#include <fstream>
#include <memory>
#include <vector>

namespace aliceVision {
namespace vfs {

class ostream : public std::ostream {
public:
    ostream() : std::ostream{&_null_buffer} { }

    ostream(const char* filename, std::ios_base::openmode mode = std::ios_base::out) : ostream()
    {
        open(filename, mode);
    }

    ostream(const std::string& filename, std::ios_base::openmode mode = std::ios_base::out) : ostream()
    {
        open(filename, mode);
    }

    ostream(const path& filename, std::ios_base::openmode mode = std::ios_base::out) : ostream()
    {
        open(filename, mode);
    }

    ostream(const ostream& other) = delete;

    ostream(ostream&& other) :
        std::ostream{std::move(other)},
        _buffer{std::move(other._buffer)}
    {
        set_rdbuf(_buffer.get());
    }

    ostream& operator=(ostream&& other)
    {
        std::ostream::operator=(std::move(other));
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

    void open(const char* filename, std::ios_base::openmode mode = std::ios_base::out);

    void open(const std::string& filename, std::ios_base::openmode mode = std::ios_base::out)
    {
        open(filename.c_str(), mode);
    }

    void open(const path& filename, std::ios_base::openmode mode = std::ios_base::out)
    {
        open(filename.string().c_str(), mode);
    }

    void close()
    {
        if (!_buffer->close())
        {
            setstate(std::ios_base::failbit);
        }
    }

    void swap(ostream& other)
    {
        std::ostream::swap(other);
        _buffer.swap(other._buffer);
    }

    ~ostream() override {}

    // Helpers for compatibility with code using C IO APIs
    int printf(const char* format, ...)
    {
        if (_c_temp_buffer.empty())
            _c_temp_buffer.reserve(128);

        std::va_list args;
        va_start(args, format);

        auto written_count = std::vsnprintf(_c_temp_buffer.data(), _c_temp_buffer.size(),
                                            format, args);

        // written_count does not include terminating null character.
        while (written_count == _c_temp_buffer.size() - 1)
        {
            _c_temp_buffer.resize(_c_temp_buffer.size() * 2);
            written_count = std::vsnprintf(_c_temp_buffer.data(), _c_temp_buffer.size(),
                                           format, args);
        }

        if (written_count > 0)
        {
            write(_c_temp_buffer.data(), written_count);
        }

        va_end(args);
        return written_count;
    }

    std::size_t fwrite(const void* buffer, std::size_t size, std::size_t count)
    {
        auto total_size = size * count;
        auto initial_pos = tellp();
        write(reinterpret_cast<const char*>(buffer), total_size);
        return (tellp() - initial_pos) / size; // truncation towards zero is appropriate in this case
    }

private:
    friend class filesystem;
    ostream(std::unique_ptr<std::filebuf>&& buffer) :
        std::ostream{buffer.get()},
        _buffer{std::move(buffer)}
    {
    }

    std::unique_ptr<std::filebuf> _buffer;

    // We need to set the buffer to something in constructor so that the state of the stream
    // is good until open() is called, or a input operation is performed (which will fail)
    null_streambuf _null_buffer;
    std::vector<char> _c_temp_buffer;
};

} // namespace vfs
} // namespace aliceVision
