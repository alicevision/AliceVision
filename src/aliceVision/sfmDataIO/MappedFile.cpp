// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#include <aliceVision/sfmDataIO/MappedFile.hpp>

#ifdef _WIN32
    #include <windows.h>
#else
    #include <sys/mman.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <unistd.h>
#endif

namespace aliceVision {
namespace sfmDataIO {

MemoryMappedFile::MemoryMappedFile() 
    : _data(nullptr)
    , _size(0)
#ifdef _WIN32
    , _fileHandle(INVALID_HANDLE_VALUE)
    , _mapHandle(INVALID_HANDLE_VALUE)
#else
    , _fileDescriptor(-1)
#endif
{
}

MemoryMappedFile::~MemoryMappedFile() 
{
    close();
}

bool MemoryMappedFile::open(const std::string& filename) {
    if (isOpen()) 
    {
        close();
    }
    
#ifdef _WIN32
    // Windows implementation - read-only
    _fileHandle = CreateFileA(filename.c_str(), 
                            GENERIC_READ, 
                            FILE_SHARE_READ | FILE_SHARE_WRITE, 
                            nullptr, 
                            OPEN_EXISTING, 
                            FILE_ATTRIBUTE_NORMAL, 
                            nullptr);
    
    if (_fileHandle == INVALID_HANDLE_VALUE) {
        return false;
    }
    
    // Get file size
    LARGE_INTEGER fileSize;
    if (!GetFileSizeEx(_fileHandle, &fileSize)) 
    {
        cleanup();
        return false;
    }
    
    _size = static_cast<size_t>(fileSize.QuadPart);
    
    if (_size == 0) 
    {
        cleanup();
        return false;
    }
    
    // Create file mapping - read-only
    _mapHandle = CreateFileMappingA(_fileHandle, nullptr, PAGE_READONLY, 0, 0, nullptr);
    
    if (_mapHandle == INVALID_HANDLE_VALUE) 
    {
        cleanup();
        return false;
    }
    
    // Map view of file - read-only
    _data = MapViewOfFile(_mapHandle, FILE_MAP_READ, 0, 0, 0);
    
    if (_data == nullptr) 
    {
        cleanup();
        return false;
    }
    
#else
    // Unix/Linux implementation - read-only
    _fileDescriptor = ::open(filename.c_str(), O_RDONLY);
    
    if (_fileDescriptor == -1) 
    {
        return false;
    }
    
    // Get file size
    struct stat fileStats;
    if (fstat(_fileDescriptor, &fileStats) == -1) 
    {
        cleanup();
        return false;
    }
    
    _size = static_cast<size_t>(fileStats.st_size);
    
    if (_size == 0) 
    {
        cleanup();
        return false;
    }
    
    // Map the file - read-only
    _data = mmap(nullptr, _size, PROT_READ, MAP_SHARED, _fileDescriptor, 0);
    
    if (_data == MAP_FAILED) 
    {
        _data = nullptr;
        cleanup();
        return false;
    }
#endif
    
    return true;
}

void MemoryMappedFile::close() 
{
    cleanup();
    _size = 0;
}

bool MemoryMappedFile::isOpen() const 
{
    return _data != nullptr;
}

const void* MemoryMappedFile::data() const 
{
    return _data;
}

size_t MemoryMappedFile::size() const 
{
    return _size;
}

void MemoryMappedFile::cleanup() 
{
#ifdef _WIN32
    if (_data != nullptr) {
        UnmapViewOfFile(_data);
        _data = nullptr;
    }
    
    if (_mapHandle != INVALID_HANDLE_VALUE) 
    {
        CloseHandle(_mapHandle);
        _mapHandle = INVALID_HANDLE_VALUE;
    }
    
    if (_fileHandle != INVALID_HANDLE_VALUE) 
    {
        CloseHandle(_fileHandle);
        _fileHandle = INVALID_HANDLE_VALUE;
    }
#else
    if (_data != nullptr) {
        munmap(_data, _size);
        _data = nullptr;
    }
    
    if (_fileDescriptor != -1) {
        ::close(_fileDescriptor);
        _fileDescriptor = -1;
    }
#endif
}

}
}