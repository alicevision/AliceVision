// This file is part of the AliceVision project.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <aliceVision/eig33/eig33.hpp>
#include <algorithm>
#include <assert.h>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <zlib.h>
#include <iostream>
#include <stdexcept>

template <class T>
class staticVector
{
    std::vector<T> _data;

    typedef typename std::vector<T>::iterator Iterator;
    typedef typename std::vector<T>::const_iterator ConstIterator;

public:
    staticVector()
    {
    }

    staticVector(int _allocated)
    {
        _data.reserve(_allocated);
    }

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

    const std::vector<T>& getData() const { return _data; }
    std::vector<T>& getDataWritable() { return _data; }
    int size() const { return _data.size(); }
    bool empty() const { return _data.empty(); }
    size_t capacity() const { return _data.capacity(); }
    void reserve(int n) { _data.reserve(n); }
    void resize(int n) { _data.resize(n); }
    void resize_with(int n, const T& val) { _data.resize(n, val); }
    void swap( staticVector& other ) { _data.swap(other._data); }

    void shrink_to_fit()
    {
        _data.shrink_to_fit();
    }

    void resizeAddIfNeeded(int nplanned, int ntoallocated)
    {
        if(size() + nplanned > capacity())
        {
            resizeAdd(nplanned + ntoallocated);
        }
    }

    void resizeAdd(int ntoallocated)
    {
        _data.reserve(capacity() + ntoallocated);
    }

    void push_sorted_asc(const T& val)
    {
        _data.insert(std::lower_bound(_data.begin(), _data.end(), val), val);
    }

    void push_back(const T& val)
    {
        _data.push_back(val);
    }

    void push_front(const T& val)
    {
        _data.insert(_data.begin(), val);
    }

    void push_back_arr(staticVector<T>* arr)
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

    T minVal() const
    {
        if (_data.empty())
            return 0;
        return *std::min_element(_data.begin(), _data.end());
    }

    T maxVal() const
    {
        if(_data.empty())
            return 0;
        return *std::max_element(_data.begin(), _data.end());
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
using staticVectorBool = staticVector<char>;

template <class T>
int sizeOfStaticVector(staticVector<T>* a)
{
    if(a == nullptr)
        return 0;
    return a->size();
};

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
};

template <class T>
void saveArrayOfArraysToFile(std::string fileName, staticVector<staticVector<T>*>* aa)
{
    std::cout << "[IO] saveArrayOfArraysToFile: " << fileName << std::endl;
    FILE* f = fopen(fileName.c_str(), "wb");
    int n = aa->size();
    fwrite(&n, sizeof(int), 1, f);
    for(int i = 0; i < n; i++)
    {
        int m = 0;
        staticVector<T>* a = (*aa)[i];
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
staticVector<staticVector<T>*>* loadArrayOfArraysFromFile(std::string fileName)
{
    std::cout << "[IO] loadArrayOfArraysFromFile: " << fileName << std::endl;
    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == nullptr)
        throw std::runtime_error("loadArrayOfArraysFromFile : can't open file " + fileName);

    int n = 0;
    fread(&n, sizeof(int), 1, f);
    staticVector<staticVector<T>*>* aa = new staticVector<staticVector<T>*>(n);
    aa->resize_with(n, NULL);
    for(int i = 0; i < n; i++)
    {
        int m = 0;
        fread(&m, sizeof(int), 1, f);
        if(m > 0)
        {
            staticVector<T>* a = new staticVector<T>(m);
            a->resize(m);
            fread(&(*a)[0], sizeof(T), m, f);
            (*aa)[i] = a;
        };
    };
    fclose(f);

    return aa;
}

template <class T>
void saveArrayToFile(std::string fileName, staticVector<T>* a, bool docompress = true)
{
    std::cout << "[IO] saveArrayToFile: " << fileName << std::endl;
    /*
    gzFile f = gzopen(fileName.c_str(),"wb");
    if (f==NULL) {printf("gzopen error\n");exit(1);};
    int n = a->size();
    if (gzwrite(&n,sizeof(int),1,f)==0) {printf("gzwrite error\n");exit(1);};
    if (gzwrite(&(*a)[0],sizeof(T),n,f)==0) {printf("gzwrite error\n");exit(1);};
    gzclose(f);
    */

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
            printf("compress error %i : %lu -> %lu, n %i \n", err, sizeof(T) * a->size(), comprLen, a->size());

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
staticVector<T>* loadArrayFromFile(std::string fileName, bool printfWarning = false)
{
    std::cout << "[IO] loadArrayFromFile: " << fileName << std::endl;
    /*
    gzFile f = gzopen(fileName.c_str(),"rb");
    if (f==NULL) {
            if (printfWarning==true) {
                    printf("WARNING: file %s does not exist!\n", fileName.c_str());
            };
            return NULL;
    }else{
            staticVector<T>* a = NULL;
            int n=0;
            if (gzread(&n,sizeof(int),1,f)==-1) {printf("gzread error\n");exit(1);};
            a = new staticVector<T>(n);
            a->resize(n);
            if (gzread(&(*a)[0],sizeof(T),n,f)==-1) {printf("gzread error\n");exit(1);};
            gzclose(f);
            return a;
    };
    */

    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == NULL)
    {
        throw std::runtime_error("loadArrayOfArraysFromFile : can't open file " + fileName);
    }
    else
    {
        int n = 0;
        fread(&n, sizeof(int), 1, f);
        staticVector<T>* a = NULL;

        if(n == -1)
        {
            fread(&n, sizeof(int), 1, f);
            a = new staticVector<T>(n);
            a->resize(n);

            uLong comprLen;
            fread(&comprLen, sizeof(uLong), 1, f);
            Byte* compr = (Byte*)calloc((uInt)comprLen, 1);
            fread(compr, sizeof(Byte), comprLen, f);

            uLong uncomprLen = sizeof(T) * n;
            int err = uncompress((Bytef*)(&(*a)[0]), &uncomprLen, compr, comprLen);

            if(err != Z_OK)
            {
                printf("uncompress error %i : %lu -> %lu, n %i \n", err, sizeof(T) * n, uncomprLen, n);
            };

            if(uncomprLen != sizeof(T) * n)
            {
                printf("WARNING uncompression failed uncomprLen!=sizeof(T)*n \n");
                exit(1);
            };

            free(compr);
        }
        else
        {
            a = new staticVector<T>(n);
            a->resize(n);
            fread(&(*a)[0], sizeof(T), n, f);
        };

        fclose(f);

        return a;
    };
}

template <class T>
void loadArrayFromFileIntoArray(staticVector<T>* a, std::string fileName, bool printfWarning = false)
{
    std::cout << "[IO] loadArrayFromFileIntoArray: " << fileName << std::endl;
    /*
    gzFile f = gzopen(fileName.c_str(),"rb");
    if (f==NULL) {
            if (printfWarning==true) {
                    printf("WARNING: file %s does not exist!\n", fileName.c_str());
            };
            return NULL;
    }else{
            staticVector<T>* a = NULL;
            int n=0;
            if (gzread(&n,sizeof(int),1,f)==-1) {printf("gzread error\n");exit(1);};
            a = new staticVector<T>(n);
            a->resize(n);
            if (gzread(&(*a)[0],sizeof(T),n,f)==-1) {printf("gzread error\n");exit(1);};
            gzclose(f);
            return a;
    };
    */

    FILE* f = fopen(fileName.c_str(), "rb");
    if(f == NULL)
    {
        if(printfWarning == true)
        {
            printf("WARNING: file %s does not exist!\n", fileName.c_str());
        };
        // return NULL;
    }
    else
    {
        int n = 0;
        fread(&n, sizeof(int), 1, f);

        if(n == -1)
        {
            fread(&n, sizeof(int), 1, f);
            if(a->size() != n)
            {
                printf("ERROR loadArrayFromFileIntoArray expected length %i loaded length %i\n", a->size(), n);
                exit(1);
            };

            uLong comprLen;
            fread(&comprLen, sizeof(uLong), 1, f);
            Byte* compr = (Byte*)calloc((uInt)comprLen, 1);
            fread(compr, sizeof(Byte), comprLen, f);

            uLong uncomprLen = sizeof(T) * n;
            int err = uncompress((Bytef*)(&(*a)[0]), &uncomprLen, compr, comprLen);

            if(err != Z_OK)
            {
                printf("uncompress error %i : %lu -> %lu, n %i \n", err, sizeof(T) * n, uncomprLen, n);
            };

            if(uncomprLen != sizeof(T) * n)
            {
                printf("WARNING uncompression failed uncomprLen!=sizeof(T)*n \n");
                exit(1);
            };

            free(compr);
        }
        else
        {
            if(a->size() != n)
            {
                printf("ERROR loadArrayFromFileIntoArray expected length %i loaded length %i\n", a->size(), n);
                exit(1);
            };
            fread(&(*a)[0], sizeof(T), n, f);
        };

        fclose(f);

        // return a;
    };
}

int getArrayLengthFromFile(std::string fileName);

template <class T>
void deleteArrayOfArrays(staticVector<staticVector<T>*>** aa)
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
staticVector<staticVector<T>*>* cloneArrayOfArrays(staticVector<staticVector<T>*>* inAOA)
{
    staticVector<staticVector<T>*>* outAOA = new staticVector<staticVector<T>*>(inAOA->size());
    // copy
    for(int i = 0; i < inAOA->size(); i++)
    {
        if((*inAOA)[i] == NULL)
        {
            outAOA->push_back(NULL);
        }
        else
        {
            staticVector<T>* outA = new staticVector<T>((*inAOA)[i]->size());
            outA->push_back_arr((*inAOA)[i]);
            outAOA->push_back(outA);
        };
    };

    return outAOA;
}
