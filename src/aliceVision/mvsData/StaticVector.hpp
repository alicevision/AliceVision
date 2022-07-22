// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>
#include <aliceVision/vfs/filesystem.hpp>
#include <aliceVision/vfs/istream.hpp>
#include <aliceVision/vfs/ostream.hpp>

#include <algorithm>
#include <assert.h>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <zlib.h>
#include <stdexcept>

namespace aliceVision {

template <class T>
class StaticVector
{
    std::vector<T> _data;

    typedef typename std::vector<T>::iterator Iterator;
    typedef typename std::vector<T>::const_iterator ConstIterator;
    typedef typename std::vector<T>::reference Reference;
    typedef typename std::vector<T>::const_reference ConstReference;

public:
    StaticVector()
    {}

    StaticVector( int n )
        : _data( n )
    {}

    StaticVector( int n, const T& value )
        : _data( n, value )
    {}

    const T& operator[](int index) const
    {
        return _data[index];
    }

    T& operator[](int index)
    {
        return _data[index];
    }

    void clear() { _data.clear(); }

    Iterator begin() { return _data.begin(); }
    Iterator end() { return _data.end(); }

    ConstIterator begin() const { return _data.begin(); }
    ConstIterator end() const { return _data.end(); }

    Reference front() { return _data.front(); }
    ConstReference front() const { return _data.front(); }

    Reference back() { return _data.back(); }
    ConstReference back() const { return _data.back(); }

    const std::vector<T>& getData() const { return _data; }
    std::vector<T>& getDataWritable() { return _data; }
    int size() const { return _data.size(); }
    bool empty() const { return _data.empty(); }
    size_t capacity() const { return _data.capacity(); }
    void reserve(int n) { _data.reserve(n); }
    void resize(int n) { _data.resize(n); }
    void resize(int n, T value) { _data.resize(n, value); }
    void resize_with(int n, const T& val) { _data.resize(n, val); }
    void swap( StaticVector& other ) { _data.swap(other._data); }
    void assign(int n, T value) { _data.assign(n, value); }

    void shrink_to_fit()
    {
        _data.shrink_to_fit();
    }

    void reserveAddIfNeeded(int nplanned, int ntoallocated)
    {
        if(size() + nplanned > capacity())
        {
            reserveAdd(nplanned + ntoallocated);
        }
    }

    void reserveAdd(int ntoallocated)
    {
        _data.reserve(capacity() + ntoallocated);
    }

    void push_back(const T& val)
    {
        _data.push_back(val);
    }

    void push_front(const T& val)
    {
        _data.insert(_data.begin(), val);
    }

    void push_back_arr(StaticVector<T>* arr)
    {
        _data.insert(_data.end(), arr->getData().begin(), arr->getData().end());
    }

    void push_back_arr(StaticVector<T>& arr)
    {
        _data.insert(_data.end(), arr.getData().begin(), arr.getData().end());
    }

    void remove(int i)
    {
        _data.erase(_data.begin() + i);
    }

    int push_back_distinct(const T& val)
    {
        int id = indexOf(val);
        if(id == -1)
            _data.push_back(val);
        return id;
    }

    T pop()
    {
        T val = _data.back();
        _data.pop_back();
        return val;
    }

    int indexOf(const T& value) const
    {
        const auto it = std::find(_data.begin(), _data.end(), value);
        return it != _data.end() ? std::distance(_data.begin(), it) : -1;
    }

    int indexOfSorted(const T& value) const
    {
        return indexOf(value);
    }

    int indexOfNearestSorted(const T& value) const
    {
        // Retrieve the first element >= value in _data
        auto it = std::lower_bound(_data.begin(), _data.end(), value);
        if(it == _data.end())
            return -1;
        // If we're between two values...
        if(it != _data.begin())
        {
            // ...select the index of the closest value between it (>= value) and prevIt (< value)
            auto prevIt = std::prev(it);
            it = (value - *prevIt) < (*it - value) ? prevIt : it;
        }
        return std::distance(_data.begin(), it);
    }

    int minValId() const
    {
        if(_data.empty())
            return -1;
        return std::distance(_data.begin(), std::min_element(_data.begin(), _data.end()));
    }

    int maxValId() const
    {
        if (_data.empty())
            return -1;
        return std::distance(_data.begin(), std::max_element(_data.begin(), _data.end()));
    }
};

// TODO: to remove
// Avoid the problematic case of std::vector<bool>::operator[]
using StaticVectorBool = StaticVector<char>;

template <class T>
int sizeOfStaticVector(const StaticVector<T>* a)
{
    if(a == nullptr)
        return 0;
    return a->size();
}

template <class T>
int sizeOfStaticVector(const StaticVector<T>& a)
{
    if(a.empty())
        return 0;
    return a.size();
}

template <class T>
int indexOf(T* arr, int n, const T& what)
{
    int isthereindex = -1;

    int i = 0;
    while((i < n) && (isthereindex == -1))
    {
        if(arr[i] == what)
        {
            isthereindex = i;
        };
        i++;
    };

    return isthereindex;
}

template <class T>
void saveArrayOfArraysToFile(vfs::filesystem& fs, const std::string& fileName,
                             StaticVector<StaticVector<T>*>* aa)
{
    ALICEVISION_LOG_DEBUG("[IO] saveArrayOfArraysToFile: " << fileName);
    auto f = fs.open_write_binary(fileName);
    int n = aa->size();
    f.fwrite(&n, sizeof(int), 1);
    for(int i = 0; i < n; i++)
    {
        int m = 0;
        StaticVector<T>* a = (*aa)[i];
        if(a == NULL)
        {
            f.fwrite(&m, sizeof(int), 1);
        }
        else
        {
            m = a->size();
            f.fwrite(&m, sizeof(int), 1);
            if(m > 0)
            {
                f.fwrite(&(*a)[0], sizeof(T), m);
            };
        };
    };
}

template <class T>
void saveArrayOfArraysToFile(vfs::filesystem& fs, const std::string& fileName,
                             StaticVector<StaticVector<T>>& aa)
{
    ALICEVISION_LOG_DEBUG("[IO] saveArrayOfArraysToFile: " << fileName);
    auto f = fs.open_write_binary(fileName);
    int n = aa.size();
    f.fwrite(&n, sizeof(int), 1);
    for(int i = 0; i < n; i++)
    {
        int m = 0;
        StaticVector<T>& a = aa[i];
        if(a.empty())
        {
            f.fwrite(&m, sizeof(int), 1);
        }
        else
        {
            m = a.size();
            f.fwrite(&m, sizeof(int), 1);
            if(m > 0)
            {
                f.fwrite(&a[0], sizeof(T), m);
            };
        };
    };
}

template <class T>
StaticVector<StaticVector<T>*>* loadArrayOfArraysFromFile(vfs::filesystem& fs, const std::string& fileName)
{
    ALICEVISION_LOG_DEBUG("[IO] loadArrayOfArraysFromFile: " << fileName);
    auto f = fs.open_read_binary(fileName);
    if (!f)
    {
        ALICEVISION_THROW_ERROR("[IO] loadArrayOfArraysFromFile: can't open file " << fileName);
    }

    int n = 0;
    size_t retval = f.fread(&n, sizeof(int), 1);
    if( retval != 1 )
    {
        ALICEVISION_THROW_ERROR("[IO] loadArrayOfArraysFromFile: can't read outer array size");
    }
    StaticVector<StaticVector<T>*>* aa = new StaticVector<StaticVector<T>*>();
    aa->reserve(n);
    aa->resize_with(n, NULL);
    for(int i = 0; i < n; i++)
    {
        int m = 0;
        retval = f.fread(&m, sizeof(int), 1);
        if( retval != 1 )
        {
            ALICEVISION_THROW_ERROR("[IO] loadArrayOfArraysFromFile: can't read inner array size");
        }
        if(m > 0)
        {
            StaticVector<T>* a = new StaticVector<T>();
            a->resize(m);
            retval = f.fread(&(*a)[0], sizeof(T), m);
            if( retval != m )
            {
                ALICEVISION_THROW_ERROR("[IO] loadArrayOfArraysFromFile: can't read vector element");
            }
            (*aa)[i] = a;
        };
    };

    return aa;
}

template <class T>
void loadArrayOfArraysFromFile(vfs::filesystem& fs, StaticVector<StaticVector<T>>& out_aa,
                               const std::string& fileName)
{
    ALICEVISION_LOG_DEBUG("[IO] loadArrayOfArraysFromFile: " << fileName);
    auto f = fs.open_read_binary(fileName);
    if (!f)
    {
        ALICEVISION_THROW_ERROR("[IO] loadArrayOfArraysFromFile: can't open file " << fileName);
    }

    int n = 0;
    size_t retval = f.fread(&n, sizeof(int), 1);
    if( retval != 1 )
    {
        ALICEVISION_THROW_ERROR("[IO] loadArrayOfArraysFromFile: can't read outer array size");
    }

    out_aa.reserve(n);
    out_aa.resize(n);
    for(int i = 0; i < n; i++)
    {
        int m = 0;
        retval = f.fread(&m, sizeof(int), 1);
        if( retval != 1 )
        {
            ALICEVISION_THROW_ERROR("[IO] loadArrayOfArraysFromFile: can't read inner array size");
        }
        if(m > 0)
        {
            StaticVector<T>& a = out_aa[i];
            a.resize(m);
            retval = f.fread(&a[0], sizeof(T), m);
            if( retval != m )
            {
                ALICEVISION_THROW_ERROR("[IO] loadArrayOfArraysFromFile: can't read vector element");
            }
        };
    };
}


template <class T>
void saveArrayToFile(vfs::filesystem& fs, const std::string& fileName, const StaticVector<T>& a,
                     bool docompress = true)
{
    saveArrayToFile(fs, fileName, &a, docompress );
}

template <class T>
void saveArrayToFile(vfs::filesystem& fs, const std::string& fileName, const StaticVector<T>* a,
                     bool docompress = true)
{
    ALICEVISION_LOG_DEBUG("[IO] saveArrayToFile: " << fileName);

    vfs::path filepath = fileName;
    fs.create_directories(filepath.parent_path());

    if( !a )
    {
        ALICEVISION_LOG_DEBUG("[IO] saveArrayToFile called with NULL static vector");
        return;
    }

    if( a->empty() )
    {
        ALICEVISION_LOG_WARNING("[IO] saveArrayToFile called with 0-sized static vector");
        return;
    }

    if((docompress == false) || (a->size() < 1000))
    {
        auto f = fs.open_write_binary(fileName);
        if (!f)
        {
            ALICEVISION_THROW_ERROR( "[IO] file " << fileName << " could not be opened, msg: ");
        }
        int n = a->size();
        if( n == 0 )
        {
            return;
        }
        int items = f.fwrite(&n, sizeof(int), 1);
        if (items < 1 && !f)
        {
            ALICEVISION_THROW_ERROR( "[IO] failed to write 1 int to " << fileName << ", msg: ");
        }
        items = f.fwrite(&(*a)[0], sizeof(T), n);
        if (items < n && !f)
        {
            ALICEVISION_THROW_ERROR( "[IO] failed to write n items to " << fileName << ", msg: ");
        }
    }
    else
    {

        /* ===========================================================================
        //from zlib - compress.c
             Compresses the source buffer into the destination buffer. The level
           parameter has the same meaning as in deflateInit.  sourceLen is the byte
           length of the source buffer. Upon entry, destLen is the total size of the
           destination buffer, which must be at least 0.1% larger than sourceLen plus
           12 bytes. Upon exit, destLen is the actual size of the compressed buffer.

             compress2 returns Z_OK if success, Z_MEM_ERROR if there was not enough
           memory, Z_BUF_ERROR if there was not enough room in the output buffer,
           Z_STREAM_ERROR if the level parameter is invalid.
        */

        // uLong comprLen = sizeof(T)*a->size();
        uLong comprLen = uLong((double)(sizeof(T) * a->size()) * 1.02) + 12;
        Byte* compr = (Byte*)calloc((uInt)comprLen, 1);
        int err = compress(compr, &comprLen, (const Bytef*)(&(*a)[0]), sizeof(T) * a->size());

        if(err != Z_OK)
        {
            ALICEVISION_LOG_ERROR("compress error " << err << " : " << (sizeof(T) * a->size()) << " -> " << comprLen << ", n " << a->size());

            auto f = fs.open_write_binary(fileName);
            if (!f)
            {
                free(compr);
                ALICEVISION_THROW_ERROR( "[IO] file " << fileName << " could not be opened, msg: ");
            }
            int n = a->size();
            if( n > 0 )
            {
                int items = f.fwrite(&n, sizeof(int), 1);
                if (items < 1 && !f)
                {
                    free(compr);
                    ALICEVISION_THROW_ERROR( "[IO] failed to write 1 int to " << fileName << ", msg: ");
                }
                items = f.fwrite(&(*a)[0], sizeof(T), n);
                if (items < 1 && !f)
                {
                    free(compr);
                    ALICEVISION_THROW_ERROR( "[IO] failed to write " << n << " items to " << fileName << ", msg: ");
                }
            }
        }
        else
        {
            auto f = fs.open_write_binary(fileName);
            if (!f)
            {
                free(compr);
                ALICEVISION_THROW_ERROR( "[IO] file " << fileName << " could not be opened, msg: ");
            }
            int n = -1;
            int items = f.fwrite(&n, sizeof(int), 1);
            if (items < 1 && !f)
            {
                free(compr);
                ALICEVISION_THROW_ERROR( "[IO] failed to write 1 int to " << fileName << ", msg: ");
            }
            n = a->size();
            items = f.fwrite(&n, sizeof(int), 1);
            if (items < 1 && !f)
            {
                free(compr);
                ALICEVISION_THROW_ERROR( "[IO] failed to write 1 int to " << fileName << ", msg: ");
            }
            items = f.fwrite(&comprLen, sizeof(uLong), 1);
            if (items < 1 && !f)
            {
                free(compr);
                ALICEVISION_THROW_ERROR( "[IO] failed to write 1 uLong to " << fileName << ", msg: ");
            }
            items = f.fwrite(compr, sizeof(Byte), comprLen);
            if (items < 1 && !f)
            {
                free(compr);
                ALICEVISION_THROW_ERROR( "[IO] failed to write " << comprLen << " items to " << fileName << ", msg: ");
            }
        };

        free(compr);
    };
}

template <class T>
StaticVector<T>* loadArrayFromFile(vfs::filesystem& fs, const std::string& fileName,
                                   bool printfWarning = false)
{
    ALICEVISION_LOG_DEBUG("[IO] loadArrayFromFile: " << fileName);

    auto f = fs.open_read_binary(fileName);
    if (!f)
    {
        ALICEVISION_THROW_ERROR("loadArrayFromFile : can't open file " << fileName);
    }
    else
    {
        int n = 0;
        size_t retval = f.fread(&n, sizeof(int), 1);
        if( retval != 1 )
        {
            ALICEVISION_THROW_ERROR("[IO] loadArrayFromFile: can't read array size (1) from " << fileName);
        }
        StaticVector<T>* a = NULL;

        if(n == -1)
        {
            retval = f.fread(&n, sizeof(int), 1);
            if( retval != 1 )
            {
                ALICEVISION_THROW_ERROR("[IO] loadArrayFromFile: can't read array size (2)");
            }
            a = new StaticVector<T>();
            a->resize(n);

            uLong comprLen;
            retval = f.fread(&comprLen, sizeof(uLong), 1);
            if( retval != 1 )
            {
                delete a;
                ALICEVISION_THROW_ERROR("[IO] loadArrayFromFile: can't read ulong elem size");
            }
            Byte* compr = (Byte*)calloc((uInt)comprLen, 1);
            retval = f.fread(compr, sizeof(Byte), comprLen);
            if( retval != comprLen )
            {
                delete a;
                ALICEVISION_THROW_ERROR("[IO] loadArrayFromFile: can't read blob");
            }

            uLong uncomprLen = sizeof(T) * n;
            int err = uncompress((Bytef*)(&(*a)[0]), &uncomprLen, compr, comprLen);

            if(err != Z_OK)
            {
                delete a;
                ALICEVISION_THROW_ERROR("uncompress error " << err << " : " << (sizeof(T) * n) << " -> " << uncomprLen << ", n " << n);
            }

            if(uncomprLen != sizeof(T) * n)
            {
                delete a;
                ALICEVISION_THROW_ERROR("loadArrayFromFile: uncompression failed uncomprLen!=sizeof(T)*n");
            }

            free(compr);
        }
        else
        {
            a = new StaticVector<T>();
            a->resize(n);
            size_t retval = f.fread(&(*a)[0], sizeof(T), n);
            if( retval != n )
            {
                delete a;
                ALICEVISION_THROW_ERROR("[IO] loadArrayFromFile: can't read n elements");
            }
        }
        return a;
    }
}

template <class T>
bool loadArrayFromFile(vfs::filesystem& fs, StaticVector<T>& out, const std::string& fileName,
                       bool printfWarning = false)
{
    ALICEVISION_LOG_DEBUG("[IO] loadArrayFromFile: " << fileName);

    auto f = fs.open_read_binary(fileName);
    if (!f)
    {
        throw std::runtime_error("loadArrayFromFile : can't open file " + fileName);
    }
    else
    {
        int n = 0;
        size_t retval = f.fread(&n, sizeof(int), 1);
        if( retval != 1 )
            ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read array size (1) from " << fileName);
        out.clear();

        if(n == -1)
        {
            retval = f.fread(&n, sizeof(int), 1);
            if( retval != 1 )
                ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read array size (2)");
            out.resize(n);

            uLong comprLen;
            retval = f.fread(&comprLen, sizeof(uLong), 1);
            if( retval != 1 )
                ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read ulong elem size");
            Byte* compr = (Byte*)calloc((uInt)comprLen, 1);
            retval = f.fread(compr, sizeof(Byte), comprLen);
            if( retval != comprLen )
                ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read blob");

            uLong uncomprLen = sizeof(T) * n;
            int err = uncompress((Bytef*)out.getDataWritable().data(), &uncomprLen, compr, comprLen);

            if(err != Z_OK)
            {
                ALICEVISION_LOG_ERROR("uncompress error " << err << " : " << (sizeof(T) * n) << " -> " << uncomprLen << ", n " << n);
            }

            if(uncomprLen != sizeof(T) * n)
            {
                throw std::runtime_error("loadArrayFromFile: uncompression failed uncomprLen!=sizeof(T)*n");
            }

            free(compr);
        }
        else
        {
            out.resize(n);
            size_t retval = f.fread(out.getDataWritable().data(), sizeof(T), n);
            if( retval != n )
                ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read n elements");
        }

        return true;
    }
}

template <class T>
void loadArrayFromFileIntoArray(vfs::filesystem& fs, StaticVector<T>* a, const std::string& fileName,
                                bool printfWarning = false)
{
    ALICEVISION_LOG_DEBUG("[IO] loadArrayFromFileIntoArray: " << fileName);

    auto f = fs.open_read_binary(fileName);
    if (!f)
    {
        ALICEVISION_THROW_ERROR("loadArrayFromFileIntoArray: can not open file: " << fileName);
    }
    int n = 0;
    f.fread(&n, sizeof(int), 1);
 
    if(n == -1)
    {
        f.fread(&n, sizeof(int), 1);
        if(a->size() != n)
        {
            ALICEVISION_THROW_ERROR("loadArrayFromFileIntoArray: expected length " << a->size() << " loaded length " << n);
        }
 
        uLong comprLen;
        f.fread(&comprLen, sizeof(uLong), 1);
        Byte* compr = (Byte*)calloc((uInt)comprLen, 1);
        f.fread(compr, sizeof(Byte), comprLen);
 
        uLong uncomprLen = sizeof(T) * n;
        int err = uncompress((Bytef*)(&(*a)[0]), &uncomprLen, compr, comprLen);
 
        if(err != Z_OK)
        {
            ALICEVISION_THROW_ERROR("uncompress error " << err << " : " << (sizeof(T) * n) << " -> " << uncomprLen << ", n " << n);
        }
 
        if(uncomprLen != sizeof(T) * n)
        {
            ALICEVISION_THROW_ERROR("loadArrayFromFileIntoArray: uncompression failed uncomprLen!=sizeof(T)*n");
        }
 
        free(compr);
    }
    else
    {
        if(a->size() != n)
        {
            ALICEVISION_THROW_ERROR("loadArrayFromFileIntoArray: expected length " << a->size() << " loaded length " << n);
        }
        f.fread(&(*a)[0], sizeof(T), n);
    }
}

int getArrayLengthFromFile(vfs::filesystem& fs, const std::string& fileName);

template <class T>
void deleteAllPointers(StaticVector<T*>& vec)
{
  for (int i = 0; i < vec.size(); ++i)
  {
    if (vec[i] != NULL)
    {
      delete(vec[i]);
      vec[i] = NULL;
    }
  }
}

template <class T>
void deleteArrayOfArrays(StaticVector<StaticVector<T>*>** aa)
{
    for(int i = 0; i < (*aa)->size(); i++)
    {
        if((*(*aa))[i] != NULL)
        {
            delete((*(*aa))[i]);
            (*(*aa))[i] = NULL;
        }
    }
    delete(*aa);
}

template <class T>
void deleteArrayOfArrays(StaticVector<StaticVector<T>*>& aa)
{
    for(int i = 0; i < aa.size(); i++)
    {
        if(aa[i] != NULL)
        {
            delete aa[i];
            aa[i] = NULL;
        };
    };
    aa.clear();
}

template <class T>
StaticVector<StaticVector<T>*>* cloneArrayOfArrays(StaticVector<StaticVector<T>*>* inAOA)
{
    StaticVector<StaticVector<T>*>* outAOA = new StaticVector<StaticVector<T>*>();
    outAOA->reserve(inAOA->size());
    // copy
    for(int i = 0; i < inAOA->size(); i++)
    {
        if((*inAOA)[i] == NULL)
        {
            outAOA->push_back(NULL);
        }
        else
        {
            StaticVector<T>* outA = new StaticVector<T>();
            outA->reserve((*inAOA)[i]->size());
            outA->push_back_arr((*inAOA)[i]);
            outAOA->push_back(outA);
        };
    };

    return outAOA;
}

} // namespace aliceVision
