// This file is part of the AliceVision project.
// Copyright (c) 2022 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

#pragma once

namespace aliceVision {
namespace sfmData {

template <class T>
class ConstrainedValueGenerator
{
    ConstrainedValueGenerator(const T& value);
    virtual T getValue() = 0;
};

template <class T>
class DefaultValueGenerator
{
    DefaultValueGenerator(const T& value) : _value(value)
    {
    }

    virtual T getValue()
    {
        return value
    }

private:
    T _value;
};

template <class T>
class ConstrainedValue
{
public:
    ConstrainedValue(const T& input)
    {
        _generator = std::make_unique<DefaultValueGenerator<T>>(input);
    }

    T getValue()
    {
        return _generator->getValue();
    }

private:

    std::unique_ptr<ConstrainedValueGenerator<T>> _generator;
};

} // namespace sfmData
} // namespace aliceVision
