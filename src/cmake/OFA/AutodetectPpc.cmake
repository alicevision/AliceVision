#=============================================================================
# Autodetection of PPC / PPC64 CPUs
#
# This is a two-step process:
#
# 1. Get the CPUID from the system by reading /proc/cpuconfig (on
# Linux), the system registry (on Windows), or executing an
# OS-specific command (macOS, BSD, SunOS, ...)
#
# 2. Determine the specific CPU from the CPUID
#=============================================================================

macro(OFA_AutodetectPpc)
    set(_cpu)

    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")

        # Linux
        file(READ "/proc/cpuinfo" _cpuinfo)
        string(REGEX REPLACE ".*cpu[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu "${_cpuinfo}")
        if(_cpu STREQUAL "POWER3")
            set(TARGET_ARCHITECTURE "power3")
        elseif(_cpu STREQUAL "POWER4")
            set(TARGET_ARCHITECTURE "power4")
        elseif(_cpu STREQUAL "POWER5")
            set(TARGET_ARCHITECTURE "power5")
        elseif(_cpu STREQUAL "POWER5+")
            set(TARGET_ARCHITECTURE "power5+")
        elseif(_cpu STREQUAL "POWER6")
            set(TARGET_ARCHITECTURE "power6")
        elseif(_cpu STREQUAL "POWER6X")
            set(TARGET_ARCHITECTURE "power6x")
        elseif(_cpu STREQUAL "POWER7")
            set(TARGET_ARCHITECTURE "power7")
        elseif(_cpu STREQUAL "POWER8" OR _cpu STREQUAL "POWER8NVL")
            set(TARGET_ARCHITECTURE "power8")
        elseif(_cpu STREQUAL "POWER9" OR _cpu STREQUAL "POWER9NVL")
            set(TARGET_ARCHITECTURE "power9")
        elseif(_cpu STREQUAL "POWER10" OR _cpu STREQUAL "POWER10NVL")
            set(TARGET_ARCHITECTURE "power10")
        else()
            message(WARNING "[OptimizeForArchitecture] Auto-detection of optimization flags failed and will use the generic CPU settings.")
        endif()

        # TODO: AIX, FreeBSD, ...

    else()

        message(FATAL_ERROR "[OptimizeForArchitecture] OptimizeForArchitecture.cmake does not implement support for CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
        return()

    endif()

    if(OFA_VERBOSE)
        message(STATUS "[OptimizeForArchitecture] CPU: ${_cpu}")
    endif()
endmacro(OFA_AutodetectPpc)
