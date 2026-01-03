// This file is part of the AliceVision project.
// Copyright (c) 2025 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

#include <map>
#include <memory>
#include <aliceVision/types.hpp>
#include <iterator>
#include <tuple>

namespace aliceVision {
namespace sfmData {

template <class T, bool IsConstant>
struct MapTraits
{
    using mapType = std::map<IndexT, std::shared_ptr<T>>;
    using baseIterator = std::conditional_t<IsConstant, 
                                            typename mapType::const_iterator, 
                                            typename mapType::iterator>;

    using keyType = const IndexT;
    using valueType = std::conditional_t<IsConstant, const T, T>;
    using pairType = std::conditional_t<IsConstant, 
                                        const std::pair<const IndexT, T>, 
                                        std::pair<const IndexT, T>>;
};

/**
 * ProxyPair is a fake pair used 
 * for reference access
*/
template <class T, bool IsConstant>
class ProxyPair
{
public:
    using Traits = MapTraits<T, IsConstant>;
public:
    explicit ProxyPair(Traits::baseIterator it) 
    : _it(it)
    {
    }

    Traits::baseIterator getIterator() const 
    {
        return _it;
    }

    Traits::keyType & first() const 
    {
        return _it->first;
    }

    Traits::valueType & second() const 
    {
        return *(_it->second);
    }

    #ifndef SWIG
    template<std::size_t I>
    decltype(auto) get() const
    {
        if constexpr (I == 0) 
        {
            return first();
        }
        else if constexpr (I == 1)
        {
            return second();
        }
        else
        {
            static_assert(I < 2, "Index out of bounds");
            return first();
        }
    }
    #endif

private:
    Traits::baseIterator _it;
};

/**
 * map iterator
 * Skip item where shared_ptr is null
 * Returned item is the object instead of the pointer
*/
template <class T, bool IsConstant>
class ValueIteratorT 
{
public:
    using Traits = MapTraits<T, IsConstant>;
    using iterator_category = std::forward_iterator_tag;
    using value_type = Traits::pairType;
    using difference_type = ptrdiff_t;
    using pointer = void;
    using reference = ProxyPair<T, IsConstant>;

public:
    
    ValueIteratorT(Traits::baseIterator current, Traits::baseIterator end)
    : _current(current), _end(end), _pair(current)
    {
        nextWhileInvalid();
    }

    reference & operator*()
    {
        return _pair;
    }

    ValueIteratorT& operator++() {
        
        ++_current;
        nextWhileInvalid();
        return *this;
    }

    ValueIteratorT operator++(int) {
        ValueIteratorT tmp = *this;
        ++(*this);
        return tmp;
    }

    bool operator==(const ValueIteratorT & other) const 
    {
        return _current == other._current;
    }

    bool operator!=(const ValueIteratorT & other) const 
    {
        return !(*this == other);
    }

    const std::pair<const IndexT, std::shared_ptr<T>>& raw() const {
        return *_current;
    }

    Traits::baseIterator baseIterator() const
    {
        return _current;
    }

private:
    void nextWhileInvalid()
    {
        while (_current != _end)
        {
            if (_current->second != nullptr)
            {
                break;
            }

            ++_current;
        }

        if (_current != _end)
        {
            _pair = ProxyPair<T, IsConstant>(_current);
        }
    }

private:
    Traits::baseIterator _current;
    Traits::baseIterator _end;
    reference _pair;
};


/**
 * Utility for loops to iterate over values instead of pointers
*/
template <typename T, bool IsConstant>
class RangeValueT
{
public:
    using Traits = MapTraits<T, IsConstant>;

public:
    RangeValueT(Traits::baseIterator begin_, Traits::baseIterator end_) 
    : _begin(begin_), _end(end_)
    {

    }

    ValueIteratorT<T, IsConstant> begin()
    {
        return ValueIteratorT<T, IsConstant>(_begin, _end);
    }

    ValueIteratorT<T, IsConstant> end()
    {
        return ValueIteratorT<T, IsConstant>(_end, _end);
    }

private:
    Traits::baseIterator _begin;
    Traits::baseIterator _end;
};


/**
 * Fallback range
*/
template <typename T, bool IsConstant>
class RangeBaseT
{
public:
    using Traits = MapTraits<T, IsConstant>;

public:
    RangeBaseT(Traits::baseIterator begin_, Traits::baseIterator end_) 
    : _begin(begin_), _end(end_)
    {

    }

    Traits::baseIterator begin()
    {
        return _begin;
    }

    Traits::baseIterator end()
    {
        return _end;
    }

private:
    Traits::baseIterator _begin;
    Traits::baseIterator _end;
};

/**
 * we have two iterators, one iterating over the shared_ptr
 * the second iterating directly on the contained values
 * ForceValueIterator make sure the value iterator is used by default
*/
template <class T, bool ForceValueIterator = false>
class SharedPtrMap : public std::map<IndexT, std::shared_ptr<T>>
{
public:
    using mapType = std::map<IndexT, std::shared_ptr<T>>;
    using ValueIterator = ValueIteratorT<T, false>;
    using ConstValueIterator = ValueIteratorT<T, true>;
    using RangeValue = RangeValueT<T, false>;
    using ConstRangeValue = RangeValueT<T, true>;
    using RangeBase = RangeBaseT<T, false>;
    using ConstRangeBase = RangeBaseT<T, true>;

public:
    SharedPtrMap() : mapType() {};

    SharedPtrMap(const SharedPtrMap & other) : mapType()
    {
        //Clone the shared_ptr content instead of shallow copy
        for (const auto & [key, value] : other.baseRange())
        { 
            this->emplace(key, value->clone());
        }
    }

    SharedPtrMap<T> & operator=(const SharedPtrMap<T> &other)
    {
        if (this == &other)
        {
            return *this;
        }
        
        //Assign by copy
        SharedPtrMap<T> tmp(other);
        tmp.swap(*this);

        return *this;
    }

    bool operator!=(const SharedPtrMap<T> & other) const
    {
        return !(*this == other);
    }

    bool operator==(const SharedPtrMap<T> & other) const
    {
        //Check same size
        if (this->size() != other.size())
        {
            return false;
        }

        //Compare all items
        for (const auto & [key, value]: other)
        {   
            //Check that we have an item with same key
            const auto it = this->mapType::find(key);
            if (it == this->mapType::end())
            {
                return false;
            }

            //If both have nullptr, then it's ok, no need to compare values
            const auto sptr = it->second;
            if (sptr == nullptr && value == nullptr)
            {
                continue;
            }

            //Error if one has nullptr
            if (sptr == nullptr || value == nullptr)
            {
                return false;
            }

            //Compare values
            if (!((*sptr) == (*value)))
            {
                return false;
            }
        }

        return true;
    }

    bool isValid(const IndexT &key) const 
    {
        //Check that the key exists
        auto it = this->mapType::find(key);
        if (it == this->mapType::end())
        {
            return false;
        }

        //Check that the value is the same
        return (it->second != nullptr);
    }

    #ifndef SWIG
    void assign(const IndexT & key, const T & value)
    {
        std::shared_ptr<T> ptr;

        //Does the key exists ? If it exists, retrieve the pointer
        auto it = this->mapType::find(key);
        if (it != this->mapType::end())
        {
            ptr = it->second;
        }
        
        if (ptr == nullptr)
        {
            ptr = std::make_shared<T>(value);
            this->insert_or_assign(key, ptr);
        }
        else 
        {
            *ptr = value;
        }
    }
    #endif

    ConstRangeValue valueRange() const
    {
        return ConstRangeValue(mapType::begin(), mapType::end());
    }

    RangeValue valueRange()
    {
        return RangeValue(mapType::begin(), mapType::end());
    }

    ConstRangeBase baseRange() const
    {
        return ConstRangeBase(mapType::begin(), mapType::end());
    }

    RangeBase baseRange()
    {
        return RangeBase(mapType::begin(), mapType::end());
    }

    //Delete operators [] as we don't want to create on the fly
    T & operator[] (const IndexT& index)
    {
        std::shared_ptr<T> ptr;

        //Does the key exists ? If it exists, retrieve the pointer
        auto it = this->mapType::find(index);
        if (it != this->mapType::end())
        {
            ptr = it->second;
        }
        
        if (ptr == nullptr)
        {
            ptr = std::make_shared<T>();
            this->insert_or_assign(index, ptr);
        }
        
        return *ptr;
    }

    T & operator[] (IndexT&& index) 
    {
        std::shared_ptr<T> ptr;

        //Does the key exists ? If it exists, retrieve the pointer
        auto it = this->mapType::find(index);
        if (it != this->mapType::end())
        {
            ptr = it->second;
        }
        
        if (ptr == nullptr)
        {
            ptr = std::make_shared<T>();
            this->insert_or_assign(index, ptr);
        }
        
        return *ptr;
    }    

    auto begin() 
    {
        if constexpr (ForceValueIterator)
        {
            return ValueIteratorT<T, false>(mapType::begin(), mapType::end());
        }
        else 
        {
            return mapType::begin();
        }
    }

    auto end() 
    {
        if constexpr (ForceValueIterator)
        {
            return ValueIteratorT<T, false>(mapType::end(), mapType::end());
        }
        else 
        {
            return mapType::end();
        }
    }

    auto begin() const
    {
        if constexpr (ForceValueIterator)
        {
            return ValueIteratorT<T, true>(mapType::begin(), mapType::end());
        }
        else 
        {
            return mapType::begin();
        }
    }

    auto end() const
    {
        if constexpr (ForceValueIterator)
        {
            return ValueIteratorT<T, true>(mapType::end(), mapType::end());
        }
        else 
        {
            return mapType::end();
        }
    }

    auto find(IndexT index) 
    {
        if constexpr (ForceValueIterator)
        {
            return ValueIteratorT<T, false>(mapType::find(index), mapType::end());
        }
        else 
        {
            return mapType::find(index);
        }
    }

    auto find(IndexT index) const
    {
        if constexpr (ForceValueIterator)
        {
            return ValueIteratorT<T, true>(mapType::find(index), mapType::end());
        }
        else 
        {
            return mapType::find(index);
        }
    }

    std::vector<IndexT> getKeys() const 
    {
        std::vector<IndexT> keys;
        for (const auto& pair : *this) 
        {
            keys.push_back(pair.first);
        }

        return keys;
    }

    std::vector<std::shared_ptr<T>> getValues() const 
    {
        std::vector<std::shared_ptr<T>> values;
        for (const auto& pair : *this) 
        {
            values.push_back(pair.second);
        }

        return values;
    }
};

}  // namespace sfmData
}  // namespace aliceVision

namespace std
{

template <class T, bool IsConstant>
struct tuple_size<aliceVision::sfmData::ProxyPair<T, IsConstant>> :
std::integral_constant<std::size_t, 2> {};

template<class T, bool IsConstant>
struct tuple_element<0, aliceVision::sfmData::ProxyPair<T, IsConstant>> {
    using type = aliceVision::sfmData::MapTraits<T, IsConstant>::keyType&;
};

template<class T, bool IsConstant>
struct tuple_element<1, aliceVision::sfmData::ProxyPair<T, IsConstant>> {
    using type = aliceVision::sfmData::MapTraits<T, IsConstant>::valueType&;
};



}

