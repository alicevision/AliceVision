// This file is part of the AliceVision project.
// Copyright (c) 2017 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/system/Logger.hpp>

#include <algorithm>
#include <assert.h>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <zlib.h>
#include <iostream>
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
int sizeOfStaticVector(StaticVector<T>* a)
{
    if(a == nullptr)
        return 0;
    return a->size();
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
void saveArrayOfArraysToFile(std::string fileName, StaticVector<StaticVector<T>*>* aa)
{
    ALICEVISION_LOG_DEBUG("[IO] saveArrayOfArraysToFile: " << fileName);
    FILE* f = fopen(fileName.c_str(), "wb");
    int n = aa->size();
    fwrite(&n, sizeof(int), 1, f);
    for(int i = 0; i < n; i++)
    {
        int m = 0;
        StaticVector<T>* a = (*aa)[i];
        if(a == NULL)
        {
            fwrite(&m, sizeof(int), 1, f);
        }
        else
        {
            m = a->size();
            fwrite(&m, sizeof(int), 1, f);
            if(m > 0)
            {
                fwrite(&(*a)[0], sizeof(T), m, f);
            };
        };
    };
    fclose(f);
}

template <class T>
StaticVector<StaticVector<T>*>* loadArrayOfArraysFromFile(std::string fileName)
{
    ALICEVISION_LOG_DEBUG("[IO] loadArrayOfArraysFromFile: " << fileName);
    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == nullptr)
        throw std::runtime_error("loadArrayOfArraysFromFile : can't open file " + fileName);

    int n = 0;
    size_t retval = fread(&n, sizeof(int), 1, f);
    if( retval != 1 )
        ALICEVISION_LOG_WARNING("[IO] loadArrayOfArraysFromFile: can't read outer array size");
    StaticVector<StaticVector<T>*>* aa = new StaticVector<StaticVector<T>*>();
    aa->reserve(n);
    aa->resize_with(n, NULL);
    for(int i = 0; i < n; i++)
    {
        int m = 0;
        retval = fread(&m, sizeof(int), 1, f);
        if( retval != 1 )
            ALICEVISION_LOG_WARNING("[IO] loadArrayOfArraysFromFile: can't read inner array size");
        if(m > 0)
        {
            StaticVector<T>* a = new StaticVector<T>();
            a->resize(m);
            retval = fread(&(*a)[0], sizeof(T), m, f);
            if( retval != m )
                ALICEVISION_LOG_WARNING("[IO] loadArrayOfArraysFromFile: can't read vector element");
            (*aa)[i] = a;
        };
    };
    fclose(f);

    return aa;
}

template <class T>
void saveArrayToFile(std::string fileName, StaticVector<T>* a, bool docompress = true)
{
    ALICEVISION_LOG_DEBUG("[IO] saveArrayToFile: " << fileName);

    if((docompress == false) || (a->size() < 1000))
    {
        FILE* f = fopen(fileName.c_str(), "wb");
        int n = a->size();
        fwrite(&n, sizeof(int), 1, f);
        fwrite(&(*a)[0], sizeof(T), n, f);
        fclose(f);
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

            FILE* f = fopen(fileName.c_str(), "wb");
            int n = a->size();
            fwrite(&n, sizeof(int), 1, f);
            fwrite(&(*a)[0], sizeof(T), n, f);
            fclose(f);
        }
        else
        {
            FILE* f = fopen(fileName.c_str(), "wb");
            int n = -1;
            fwrite(&n, sizeof(int), 1, f);
            n = a->size();
            fwrite(&n, sizeof(int), 1, f);
            fwrite(&comprLen, sizeof(uLong), 1, f);
            fwrite(compr, sizeof(Byte), comprLen, f);
            fclose(f);
        };

        free(compr);
    };
}

template <class T>
StaticVector<T>* loadArrayFromFile(std::string fileName, bool printfWarning = false)
{
    ALICEVISION_LOG_DEBUG("[IO] loadArrayFromFile: " << fileName);

    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == NULL)
    {
        throw std::runtime_error("loadArrayFromFile : can't open file " + fileName);
    }
    else
    {
        int n = 0;
        size_t retval = fread(&n, sizeof(int), 1, f);
        if( retval != 1 )
            ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read array size (1) from " << fileName);
        StaticVector<T>* a = NULL;

        if(n == -1)
        {
            retval = fread(&n, sizeof(int), 1, f);
            if( retval != 1 )
                ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read array size (2)");
            a = new StaticVector<T>();
            a->resize(n);

            uLong comprLen;
            retval = fread(&comprLen, sizeof(uLong), 1, f);
            if( retval != 1 )
                ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read ulong elem size");
            Byte* compr = (Byte*)calloc((uInt)comprLen, 1);
            retval = fread(compr, sizeof(Byte), comprLen, f);
            if( retval != comprLen )
                ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read blob");

            uLong uncomprLen = sizeof(T) * n;
            int err = uncompress((Bytef*)(&(*a)[0]), &uncomprLen, compr, comprLen);

            if(err != Z_OK)
            {
                ALICEVISION_LOG_ERROR("uncompress error " << err << " : " << (sizeof(T) * n) << " -> " << uncomprLen << ", n " << n);
            }

            if(uncomprLen != sizeof(T) * n)
            {
                delete a;
                fclose(f);
                throw std::runtime_error("loadArrayFromFile: uncompression failed uncomprLen!=sizeof(T)*n");
            }

            free(compr);
        }
        else
        {
            a = new StaticVector<T>();
            a->resize(n);
            size_t retval = fread(&(*a)[0], sizeof(T), n, f);
            if( retval != n )
                ALICEVISION_LOG_WARNING("[IO] loadArrayFromFile: can't read n elements");
        }

        fclose(f);

        return a;
    }
}

template <class T>
void loadArrayFromFileIntoArray(StaticVector<T>* a, std::string fileName, bool printfWarning = false)
{
    ALICEVISION_LOG_DEBUG("[IO] loadArrayFromFileIntoArray: " << fileName);

    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == NULL)
    {
        throw std::runtime_error("loadArrayFromFileIntoArray: can not open file: " + fileName);
    }
    int n = 0;
    fread(&n, sizeof(int), 1, f);
 
    if(n == -1)
    {
        fread(&n, sizeof(int), 1, f);
        if(a->size() != n)
        {
            std::stringstream s;
            s << "loadArrayFromFileIntoArray: expected length " << a->size() << " loaded length " << n;
            fclose(f);
            throw std::runtime_error(s.str());
        }
 
        uLong comprLen;
        fread(&comprLen, sizeof(uLong), 1, f);
        Byte* compr = (Byte*)calloc((uInt)comprLen, 1);
        fread(compr, sizeof(Byte), comprLen, f);
 
        uLong uncomprLen = sizeof(T) * n;
        int err = uncompress((Bytef*)(&(*a)[0]), &uncomprLen, compr, comprLen);
 
        if(err != Z_OK)
        {
            ALICEVISION_LOG_ERROR("uncompress error " << err << " : " << (sizeof(T) * n) << " -> " << uncomprLen << ", n " << n);
        }
 
        if(uncomprLen != sizeof(T) * n)
        {
            throw std::runtime_error("loadArrayFromFileIntoArray: uncompression failed uncomprLen!=sizeof(T)*n");
        }
 
        free(compr);
    }
    else
    {
        if(a->size() != n)
        {
            std::stringstream s;
            s << "loadArrayFromFileIntoArray: expected length " << a->size() << " loaded length " << n;
            throw std::runtime_error(s.str());
        }
        fread(&(*a)[0], sizeof(T), n, f);
    }
 
    fclose(f);
}

int getArrayLengthFromFile(std::string fileName);

template <class T>
void deleteArrayOfArrays(StaticVector<StaticVector<T>*>** aa)
{
    for(int i = 0; i < (*aa)->size(); i++)
    {
        if((*(*aa))[i] != NULL)
        {
            delete(*(*aa))[i];
            (*(*aa))[i] = NULL;
        };
    };
    delete(*aa);
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
