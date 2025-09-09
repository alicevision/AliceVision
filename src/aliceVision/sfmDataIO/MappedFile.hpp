// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <string>

#ifdef _WIN32
    #include <windows.h>
#endif

namespace aliceVision {
namespace sfmDataIO {

class MemoryMappedFile {
public:
    MemoryMappedFile();
    ~MemoryMappedFile();
    MemoryMappedFile(const MemoryMappedFile&) = delete;
    MemoryMappedFile& operator=(const MemoryMappedFile&) = delete;
    
    /** 
     * @brief Open a file for read-only memory mapping
     * @param filename path to the file to open
     * @return true on success
     */
    bool open(const std::string& filename);
    
    /** 
     * @brief Close the file
     */
    void close();
    
    /** 
     * @brief Is a file opened
     * @return true if opened
     */
    bool isOpen() const;
    
    /** 
     * @brief Get a void pointer to the mapped data
     * @return a pointer to the file's data
     */
    const void* data() const;
    
    /** 
     * @brief Get a casted pointer to the mapped data
     * @return a pointer to the file's data
     */
    template<typename T>
    const T* data() const {
        return static_cast<const T*>(data());
    }
    
    /** 
     * @brief Get the byte size of the mapped data
     * @return a size in bytes
     */
    size_t size() const;

private:
    void cleanup();
    
    void * _data;
    size_t _size;
    
#ifdef _WIN32
    HANDLE _fileHandle;
    HANDLE _mapHandle;
#else
    int _fileDescriptor;
#endif
};

}
}