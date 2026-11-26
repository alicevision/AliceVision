// This file is part of the AliceVision project.
// Copyright (c) 2026 AliceVision contributors.
// This Source Code Form is subject to the terms of the Mozilla Public License,
// v. 2.0. If a copy of the MPL was not distributed with this file,
// You can obtain one at https://mozilla.org/MPL/2.0/.

// Convert the C++ exceptions thrown by the wrapped functions into Python exceptions.
// Without it, an uncaught C++ exception (e.g. when reading an invalid file) terminates the Python process.
// The standard exceptions keep their meaning (invalid_argument: ValueError, out_of_range: IndexError, ...),
// the other ones are raised as RuntimeError.

%include <exception.i>

%{
#include <new>
#include <stdexcept>

// Set the Python error matching the C++ exception being handled.
// Must be called from a catch block: it rethrows the current exception to find its type.
// Kept out of %exception, which is inlined into every wrapper function.
static inline void aliceVision_setPythonErrorFromCurrentException()
{
    try
    {
        throw;
    }
    catch (const std::invalid_argument& e)
    {
        SWIG_Error(SWIG_ValueError, e.what());
    }
    catch (const std::domain_error& e)
    {
        SWIG_Error(SWIG_ValueError, e.what());
    }
    catch (const std::out_of_range& e)
    {
        SWIG_Error(SWIG_IndexError, e.what());
    }
    catch (const std::overflow_error& e)
    {
        SWIG_Error(SWIG_OverflowError, e.what());
    }
    catch (const std::bad_alloc& e)
    {
        SWIG_Error(SWIG_MemoryError, e.what());
    }
    catch (const std::exception& e)
    {
        SWIG_Error(SWIG_RuntimeError, e.what());
    }
    catch (...)
    {
        SWIG_Error(SWIG_RuntimeError, "Unknown C++ exception");
    }
}
%}

%exception {
    try
    {
        $action
    }
    catch (...)
    {
        aliceVision_setPythonErrorFromCurrentException();
        SWIG_fail;
    }
}
