# Add a given compiler flag to flag variables.
#
# Usage:
# AddCXXCompilerFlag(<flag>
#                    [CODE <var>]
#                    [EXTRA_FLAGS <var>]
#                    [FLAGS <var>]
#                    [HEADERS <var>]
#                    [RESULT <var>]
#                    [TESTS <var>])
#
# Input argument:
# <flag>             flag to be added after succesful completion of all tests
#
# Optional input arguments:
# CODE <VAR>         variable holding the test code; this overrides the
#                    automatic generation of the test code
# EXTRA_FLAGS <var>  variable holding the list of extra compiler flags that
#                    are used without checks
# FLAGS <var>        variable holding the list of flags to which <flag> is
#                    added after succesful completion of all tests
# HEADERS <var>      variable holding the list of header files prepended to
#                    the C++ test code's main function
# TESTS <var>        variable holding the list of tests to be included in
#                    the C++ test code's main function body
#
# Output argument:
# RESULT <var>       variable holding the result of all tests

#=============================================================================
# This code is largely inspired by
#
# AddCompilerFlag.cmake
# Copyright 2010-2015 Matthias Kretz <kretz@kde.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  * Neither the names of contributing organizations nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
#
# and
#
# CheckCXXCompilerFlag.cmake
# Copyright 2006-2009 Kitware, Inc.
# Copyright 2006 Alexander Neundorf <neundorf@kde.org>
# Copyright 2011-2013 Matthias Kretz <kretz@kde.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#  * The names of Kitware, Inc., the Insight Consortium, or the names of
#    any consortium members, or of any contributors, may not be used to
#    endorse or promote products derived from this software without
#    specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS ``AS IS''
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHORS OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# =============================================================================

include(CheckIncludeFileCXX)
include(OFA/CheckCXXCompilerFlag)

macro(AddCXXCompilerFlag _flag)
    set(state 0)
    unset(_code)
    unset(_extra_flags)
    unset(_flags)
    unset(_headers)
    unset(_result)
    unset(_tests)

    foreach(_arg ${ARGN})
        if("x${_arg}" STREQUAL "xCODE")
            set(state 1)
        elseif("x${_arg}" STREQUAL "xEXTRA_FLAGS")
            set(state 2)
        elseif("x${_arg}" STREQUAL "xFLAGS")
            set(state 3)
        elseif("x${_arg}" STREQUAL "xHEADERS")
            set(state 4)
        elseif("x${_arg}" STREQUAL "xRESULT")
            set(state 5)
        elseif("x${_arg}" STREQUAL "xTESTS")
            set(state 6)

        elseif(state EQUAL 1)
            set(_code ${_arg})
        elseif(state EQUAL 2)
            set(_extra_flags ${_arg})
        elseif(state EQUAL 3)
            set(_flags ${_arg})
        elseif(state EQUAL 4)
            set(_headers ${_arg})
        elseif(state EQUAL 5)
            set(_result ${_arg})
        elseif(state EQUAL 6)
            set(_tests ${_arg})
        else()
            message(FATAL_ERROR "[OptimizeForArchitecture] The argument ${_arg} is not supported by AddCXXCompilerFlag")
        endif()
    endforeach()

    set(_check_include_file_cxx TRUE)
    set(_check_cxx_source_compiles TRUE)

    # Check availability of header file(s)
    foreach(_header ${_headers})
        set(_resultVar "HAVE_${_header}")
        string(REGEX REPLACE "[-.+/:= ]" "_" _resultVar "${_resultVar}")
        check_include_file_cxx(${_header} ${_resultVar} "${_flag}${_extra_flags}")

        if(NOT ${_resultVar})
            set(_check_include_file_cxx FALSE)
        endif()
    endforeach()

    # Check if compiler supports flag and can compile code
    set(_cxx_code)
    foreach(_header ${_headers})
        set(_cxx_code "${_cxx_code}\n#include<${_header}>")
    endforeach()

    if(_code)
        set(_cxx_code "${_cxx_code}\n${_code}")
    elseif(_tests)
        set(_cxx_code "${_cxx_code}\nint main() {")
        foreach(_test ${_tests})
            set(_cxx_code "${_cxx_code}\n${_test}")
        endforeach()
        set(_cxx_code "${_cxx_code}\nreturn 0; }")
    else()
        set(_cxx_code "${_cxx_code}\nint main() { return 0; }")
    endif()

    set(_CMAKE_REQUIRED_FLAGS "${CMAKE_REQUIRED_FLAGS}")
    set(CMAKE_REQUIRED_FLAGS "${_flag}${_extra_flags}")
    set(_resultVar "HAVE_${_flag}")
    string(REGEX REPLACE "[-.+/:= ]" "_" _resultVar "${_resultVar}")
    check_cxx_source_compiles("${_cxx_code}" ${_resultVar}
    # Some compilers do not fail with a bad flag
    FAIL_REGEX "error: bad value (.*) for .* switch"       # GNU
    FAIL_REGEX "argument unused during compilation"        # clang
    FAIL_REGEX "warning: the flag .* has been deprecated"  # clang
    FAIL_REGEX "is valid for .* but not for C\\\\+\\\\+"   # GNU
    FAIL_REGEX "unrecognized .*option"                     # GNU
    FAIL_REGEX "ignored for target"                        # GNU
    FAIL_REGEX "ignoring unknown option"                   # MSVC
    FAIL_REGEX "warning D9002"                             # MSVC
    FAIL_REGEX "[Uu]nknown option"                         # HP
    FAIL_REGEX "[Ww]arning: [Oo]ption"                     # SunPro
    FAIL_REGEX "[Ww]arning: illegal use of -xarch option"  # SunPro
    FAIL_REGEX "command option .* is not recognized"       # XL
    FAIL_REGEX "WARNING: unknown flag:"                    # Open64
    FAIL_REGEX "command line error"                        # ICC
    FAIL_REGEX "command line warning"                      # ICC
    FAIL_REGEX "#10236:"                                   # ICC: File not found
    FAIL_REGEX " #10159: "                                 # ICC
    FAIL_REGEX " #10353: "                                 # ICC: option '-mfma' ignored, suggest using '-march=core-avx2'
    FAIL_REGEX " #10006: "                                 # ICC: ignoring unknown option '-mavx512fp16'
    )
    set(CMAKE_REQUIRED_FLAGS "${_CMAKE_REQUIRED_FLAGS}")

    if(NOT ${_resultVar})
        set(_check_cxx_source_compiles FALSE)
    endif()

    if (DEFINED _result)
        if (${_check_include_file_cxx} AND ${_check_cxx_source_compiles})
            set(${_result} TRUE)
        else()
            set(${_result} FALSE)
        endif()
    endif()

    if(DEFINED _flags AND ${_check_include_file_cxx} AND ${_check_cxx_source_compiles})
        list(APPEND ${_flags} "${_flag}")
    endif()
endmacro(AddCXXCompilerFlag)
