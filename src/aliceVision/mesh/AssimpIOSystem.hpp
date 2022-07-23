// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/vfs/filesystem.hpp>
#include <aliceVision/vfs/istream.hpp>
#include <aliceVision/vfs/ostream.hpp>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>

namespace aliceVision {
namespace mesh {

namespace {
    std::ios_base::seekdir convertAiOriginToIosBaseSeekDir(aiOrigin origin)
    {
        switch (origin)
        {
            case aiOrigin_SET: return std::ios_base::beg;
            case aiOrigin_CUR: return std::ios_base::cur;
            case aiOrigin_END: return std::ios_base::end;
            default: throw std::invalid_argument("Unsupported aiOrigin");
        }
    }
} // namespace

class VfsIOReadStream : public Assimp::IOStream
{
public:
    VfsIOReadStream(vfs::istream&& stream) : _stream{std::move(stream)}
    {
        _stream.seekg(0, std::ios_base::end);
        _size = _stream.tellg();
        _stream.seekg(0, std::ios_base::beg);
    }

    ~VfsIOReadStream() override { }

    std::size_t Read(void* buffer, std::size_t size, std::size_t count) override
    {
        auto total_size = size * count;
        if (total_size == 0)
            return 0;

        _stream.read(reinterpret_cast<char*>(buffer), total_size);
        return _stream.gcount();
    }

    std::size_t Write(const void* buffer, std::size_t size, std::size_t count) override
    {
        return 0; // Stream is configured for reading
    }

    virtual aiReturn Seek(std::size_t offset, aiOrigin origin) override
    {
        _stream.seekg(offset, convertAiOriginToIosBaseSeekDir(origin));
        return aiReturn_SUCCESS;
    }

    std::size_t Tell() const override
    {
        return _stream.tellg();
    }

    std::size_t FileSize() const override
    {
        return _size;
    }

    void Flush() override { }

private:
    mutable vfs::istream _stream;
    std::size_t _size = 0;
};

class VfsIOWriteStream : public Assimp::IOStream
{
public:
    VfsIOWriteStream(vfs::ostream&& stream) : _stream{std::move(stream)} { }

    ~VfsIOWriteStream() override { }


    std::size_t Read(void* buffer, std::size_t size, std::size_t count) override
    {
        return 0; // Stream is configured for writing
    }

    std::size_t Write(const void* buffer, std::size_t size, std::size_t count) override
    {
        auto total_size = size * count;
        if (total_size == 0)
            return 0;

        auto initial_pos = _stream.tellp();
        _stream.write(reinterpret_cast<const char*>(buffer), total_size);
        _size = std::max<std::size_t>(_size, _stream.tellp());
        return _stream.tellp() - initial_pos;
    }

    virtual aiReturn Seek(std::size_t offset, aiOrigin origin) override
    {
        _stream.seekp(offset, convertAiOriginToIosBaseSeekDir(origin));
        return aiReturn_SUCCESS;
    }

    std::size_t Tell() const override
    {
        return _stream.tellp();
    }

    std::size_t FileSize() const override
    {
        return _size;
    }

    void Flush() override { }

private:
    mutable vfs::ostream _stream;
    std::size_t _size = 0;
};


class VfsIOSystem : public Assimp::IOSystem
{
public:
    bool Exists(const char* path) const override
    {
        return vfs::exists(path);
    }

    char getOsSeparator() const override
    {
        // boost::filesystem::path::preferred_separator is not fit here because its type
        // changes depending on platform
#ifdef BOOST_WINDOWS_API
        return '\\';
#else
        return '/';
#endif
    }

    Assimp::IOStream* Open(const char* path, const char* mode) override
    {
        if (path == nullptr || mode == nullptr)
            return nullptr;

        if (mode[0] == 'r')
        {
            vfs::istream stream{path, std::ios_base::binary};
            if (!stream.is_open())
                return nullptr;

            return new VfsIOReadStream(std::move(stream));
        }

        if (mode[0] == 'w')
        {
            vfs::ostream stream{path, std::ios_base::binary};
            if (!stream.is_open())
                return nullptr;

            return new VfsIOWriteStream(std::move(stream));
        }
        return nullptr;
    }

    void Close(Assimp::IOStream* stream) override
    {
        delete stream;
    }

    bool ComparePaths(const char* one, const char* second) const override
    {
        return vfs::equivalent(one, second);
    }

    bool CreateDirectory(const std::string &path) override
    {
        vfs::error_code ec;
        vfs::create_directory(path, ec);
        return !ec.failed();
    }

    bool ChangeDirectory(const std::string &path) override
    {
        vfs::error_code ec;
        vfs::current_path(path, ec);
        return !ec.failed();
    }

    bool DeleteFile(const std::string &file) override
    {
        vfs::error_code ec;
        bool existed = vfs::remove(file, ec);
        return existed && !ec.failed();
    }
};

} // namespace mesh
} // namespace aliceVision
