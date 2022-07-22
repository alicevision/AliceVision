// This file is part of the AliceVision project.
// Copyright (c) 2016 AliceVision contributors.
// Copyright (c) 2012 openMVG contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/vfs/filesystem.hpp>
#include <OpenImageIO/filesystem.h>
#include <aliceVision/vfs/istream.hpp>
#include <aliceVision/vfs/ostream.hpp>

namespace aliceVision {
namespace image {

class VfsIOReadProxy : public OIIO::Filesystem::IOProxy {
public:
    VfsIOReadProxy(vfs::filesystem& fs, const vfs::path& path) :
        IOProxy{path.string(), Read},
        _stream{fs.open_read_binary(path)}
    {
        _stream.seekg(0, std::ios_base::end);
        _size = _stream.tellg();
        _stream.seekg(0, std::ios_base::beg);
    }

    ~VfsIOReadProxy() override { }

    const char* proxytype() const override
    {
        return "VfsIOReadProxy";
    }

    void close() override
    {
        _stream.close();
        m_mode = Closed;
    }

    std::int64_t tell() override
    {
        return _stream.tellg();
    }

    bool seek(std::int64_t offset) override
    {
        _stream.seekg(offset);
        return true;
    }

    std::size_t read(void *buf, std::size_t size) override
    {
        _stream.read(reinterpret_cast<char*>(buf), size);
        return _stream.gcount();
    }

    std::size_t write(const void *buf, std::size_t size) override
    {
        throw std::runtime_error("Proxy is configured only for reading");
    }

    std::size_t pread(void *buf, std::size_t size, std::int64_t offset) override
    {
        auto initial_pos = _stream.tellg();
        _stream.seekg(offset);
        auto read_size = read(buf, size);
        _stream.seekg(initial_pos);
        return read_size;
    }

    std::size_t pwrite(const void *buf, std::size_t size, std::int64_t offset) override
    {
        throw std::runtime_error("Proxy is configured only for reading");
    }

    std::size_t size() const override
    {
        return _size;
    }

    void flush() const override { }

protected:
    vfs::istream _stream;
    std::size_t _size = 0;
};

class VfsIOWriteProxy : public OIIO::Filesystem::IOProxy {
public:
    VfsIOWriteProxy(vfs::filesystem& fs, const vfs::path& path) :
        IOProxy{path.string(), Write},
        _stream{fs.open_write_binary(path)}
    {
    }

    ~VfsIOWriteProxy() override { }

    const char* proxytype () const override
    {
        return "VfsIOWriteProxy";
    }

    void close() override
    {
        _stream.close();
        m_mode = Closed;
    }

    std::int64_t tell() override
    {
        return _stream.tellp();
    }

    bool seek(std::int64_t offset) override
    {
        _stream.seekp(offset);
        return true;
    }

    std::size_t read(void *buf, std::size_t size) override
    {
        throw std::runtime_error("Proxy is configured only for writing");
    }

    std::size_t write(const void *buf, std::size_t size) override
    {
        _stream.write(reinterpret_cast<const char*>(buf), size);
        _size = std::max<std::size_t>(_size, _stream.tellp());
        return size;
    }

    std::size_t pread(void *buf, std::size_t size, std::int64_t offset) override
    {
        throw std::runtime_error("Proxy is configured only for writing");
    }

    std::size_t pwrite(const void *buf, std::size_t size, std::int64_t offset) override
    {
        auto initial_pos = _stream.tellp();
        _stream.seekp(offset);
        write(buf, size);
        _stream.seekp(initial_pos);

        _size = std::max(_size, offset + size);
        return size;
    }

    virtual size_t size() const override
    {
        return _size;
    }

    void flush() const override
    {
        _stream.flush();
    }

protected:
    mutable vfs::ostream _stream;
    std::size_t _size = 0;
};

} // namespace image
} // namespace aliceVision
