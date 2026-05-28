#=============================================================================
# Autodetection of X86 / X86_64 CPUs
#
# This is a two-step process:
#
# 1. Get the CPUID from the system by reading /proc/cpuconfig (on
# Linux), the system registry (on Windows), or executing an
# OS-specific command (macOS, BSD, SunOS, ...)
#
# 2. Determine the specific CPU from the CPUID
#=============================================================================

macro(OFA_AutodetectX86)
    set(_vendor_id)
    set(_cpu_family)
    set(_cpu_model)
    set(_cpu_stepping)

    # Get CPUID from system
    if(CMAKE_SYSTEM_NAME STREQUAL "Linux")

        # Linux
        file(READ "/proc/cpuinfo" _cpuinfo)
        string(REGEX REPLACE ".*vendor_id[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _vendor_id "${_cpuinfo}")
        string(REGEX REPLACE ".*cpu family[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_family "${_cpuinfo}")
        string(REGEX REPLACE ".*model[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_model "${_cpuinfo}")
        string(REGEX REPLACE ".*stepping[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_stepping "${_cpuinfo}")
        string(REGEX REPLACE ".*flags[ \t]*:[ \t]+([^\n]+).*" "\\1" _cpu_flags "${_cpuinfo}")

    elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")

        # macOS
        execute_process(COMMAND /usr/sbin/sysctl -n machdep.cpu.vendor machdep.cpu.family machdep.cpu.model machdep.cpu.stepping machdep.cpu.features
      OUTPUT_VARIABLE _sysctl_output_string RESULT_VARIABLE _error)
        if(NOT _error)
            string(REPLACE "\n" ";" _sysctl_output ${_sysctl_output_string})
            list(GET _sysctl_output 0 _vendor_id)
            list(GET _sysctl_output 1 _cpu_family)
            list(GET _sysctl_output 2 _cpu_model)
            list(GET _sysctl_output 3 _cpu_stepping)
            list(GET _sysctl_output 4 _cpu_flags)
            string(TOLOWER "${_cpu_flags}" _cpu_flags)
            string(REPLACE "." "_" _cpu_flags "${_cpu_flags}")
        else()
            # Apple Silicon (ARM64) running in Rosetta 2 mode
            #
            # The regular detection mechanism for macOS-x64_86 does not work
            # because the emulated CPU does not provide the required
            # information via the sysctl command. We therefore generate fake
            # vendor, model, and stepping information based on the
            # macOS-specific CPU codes.
            execute_process(COMMAND /usr/sbin/sysctl -n hw.cputype machdep.cpu.family hw.cpufamily machdep.cpu.features
        OUTPUT_VARIABLE _sysctl_output_string RESULT_VARIABLE _error)
            if(NOT _error)
                string(REPLACE "\n" ";" _sysctl_output ${_sysctl_output_string})
                list(GET _sysctl_output 0 _cpu_implementer)
                list(GET _sysctl_output 1 _cpu_family)
                list(GET _sysctl_output 2 _cpu_model)
                list(GET _sysctl_output 3 _cpu_flags)
                string(TOLOWER "${_cpu_flags}" _cpu_flags)
                string(REPLACE "." "_" _cpu_flags "${_cpu_flags}")

                # Fake vendor
                if(_cpu_implementer STREQUAL "0x7" OR _cpu_implementer STREQUAL "7")
                    set(_vendor_id "GenuineIntel")
                else()
                    set(_vendor_id "Unknown")
                endif()

                # Fake stepping
                set(_cpu_stepping "Unknown")

                # Fake model
                # Taken from /Library/Developer/CommandLineTools/SDKs/MacOSX12.sdk/System/Library/Frameworks/Kernel.framework/Versions/A/Headers/mach/machine.h
                if(    _cpu_model STREQUAL "0x78ea4fbc" OR _cpu_model STREQUAL "2028621756") # Penryn
                    set(_cpu_model    "23")
                elseif(_cpu_model STREQUAL "0x6b5a4cd2" OR _cpu_model STREQUAL "1801080018") # Nehalem
                    set(_cpu_model    "26")
                elseif(_cpu_model STREQUAL "0x573b5eec" OR _cpu_model STREQUAL "1463508716") # Westmere
                    set(_cpu_model    "37")
                elseif(_cpu_model STREQUAL "0x5490b78c" OR _cpu_model STREQUAL "1418770316") # Sandybridge
                    set(_cpu_model    "42")
                elseif(_cpu_model STREQUAL "0x1f65e835" OR _cpu_model STREQUAL "526772277")  # Ivybridge
                    set(_cpu_model    "58")
                elseif(_cpu_model STREQUAL "0x10b282dc" OR _cpu_model STREQUAL "280134364")  # Haswell
                    set(_cpu_model    "60")
                elseif(_cpu_model STREQUAL "0x582ed09c" OR _cpu_model STREQUAL "1479463068") # Broadwell
                    set(_cpu_model    "61")
                elseif(_cpu_model STREQUAL "0x37fc219f" OR _cpu_model STREQUAL "939270559")  # Skylake
                    set(_cpu_model    "78")
                elseif(_cpu_model STREQUAL "0x0f817246" OR _cpu_model STREQUAL "260141638")  # Kabylake
                    set(_cpu_model    "142")
                elseif(_cpu_model STREQUAL "0x38435547" OR _cpu_model STREQUAL "943936839")  # Icelake
                    set(_cpu_model    "125")
                elseif(_cpu_model STREQUAL "0x1cf8a03e" OR _cpu_model STREQUAL "486055998")  # Cometlake
                    set(_cpu_model    "142")
                else()
                    set(_cpu_model    "Unknown")
                endif()
            endif()
        endif()
        if(_error)
            message(FATAL_ERROR "[OptimizeForArchitecture] OptimizeForArchitecture.cmake does not implement support for CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
        endif()

    elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")

        # Windows
        get_filename_component(_vendor_id "[HKEY_LOCAL_MACHINE\\Hardware\\Description\\System\\CentralProcessor\\0;VendorIdentifier]" NAME CACHE)
        get_filename_component(_cpu_id "[HKEY_LOCAL_MACHINE\\Hardware\\Description\\System\\CentralProcessor\\0;Identifier]" NAME CACHE)
        mark_as_advanced(_vendor_id _cpu_id)
        string(REGEX REPLACE ".* Family ([0-9]+) .*" "\\1" _cpu_family "${_cpu_id}")
        string(REGEX REPLACE ".* Model ([0-9]+) .*" "\\1" _cpu_model "${_cpu_id}")
        string(REGEX REPLACE ".* Stepping ([0-9]+) .*" "\\1" _cpu_mstepping "${_cpu_id}")

    else()

        # Try to retrieve CPUID directly
        try_run(_exit _ok
      ${CMAKE_CURRENT_BINARY_DIR}
      ${CMAKE_SOURCE_DIR}/CMake/OFA/cpuinfo_x86.cxx
      RUN_OUTPUT_VARIABLE _cpuinfo)

        if(_ok AND ${_exit} EQUAL 0)
            string(REGEX REPLACE ".*vendor_id[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _vendor_id "${_cpuinfo}")
            string(REGEX REPLACE ".*cpu family[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_family "${_cpuinfo}")
            string(REGEX REPLACE ".*model[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_model "${_cpuinfo}")
            string(REGEX REPLACE ".*stepping[ \t]*:[ \t]+([a-zA-Z0-9_-]+).*" "\\1" _cpu_stepping "${_cpuinfo}")
            string(REGEX REPLACE ".*flags[ \t]*:[ \t]+([^\n]+).*" "\\1" _cpu_flags "${_cpuinfo}")

        else()

            message(FATAL_ERROR "[OptimizeForArchitecture] OptimizeForArchitecture.cmake does not implement support for CMAKE_SYSTEM_NAME: ${CMAKE_SYSTEM_NAME}")
        endif()
    endif()

    # Determine CPU from CPUID
    if(_vendor_id STREQUAL "GenuineIntel")
        if(_cpu_family EQUAL 19)

            # MIC architecture
            if(_cpu_model EQUAL 1)
                set(TARGET_ARCHITECTURE "diamondrapids")
            elseif(_cpu_model EQUAL 0)
                set(TARGET_ARCHITECTURE "novalake")

            else()
                message(WARNING
          " [OptimizeForArchitecture] Your CPU is not known.\n"
          " \tAuto-detection of optimization flags failed and will use the 65nm Core 2 CPU settings.\n"
          " \tPlease send an email to gismo@inria.fr with the following content so that we can update the OFA script:\n"
          " \tVendor id:    ${_vendor_id}\n"
          " \tCPU family:   ${_cpu_family}\n"
          " \tCPU mode:     ${_cpu_model}\n"
          " \tCPU stepping: ${_cpu_stepping}\n"
          " \tCPU flags:    ${_cpu_flags}")
                set(TARGET_ARCHITECTURE "merom")
            endif()

        elseif(_cpu_family EQUAL 6)
            # taken from the Intel ORM
            # http://www.intel.com/content/www/us/en/processors/architectures-software-developer-manuals.html
            # CPUID Signature Values Of Recent Intel Microarchitectures
            # 4E 5E       | Skylake microarchitecture
            # 3D 47 56    | Broadwell microarchitecture
            # 3C 45 46 3F | Haswell microarchitecture
            # 3A 3E       | Ivy Bridge microarchitecture
            # 2A 2D       | Sandy Bridge microarchitecture
            # 25 2C 2F    | Intel microarchitecture Westmere
            # 1A 1E 1F 2E | Intel microarchitecture Nehalem
            # 17 1D       | Enhanced Intel Core microarchitecture
            # 0F          | Intel Core microarchitecture
            #
            # Intel SDM Vol. 3C 35-1 / December 2016:
            # 57          | Xeon Phi 3200, 5200, 7200  [Knights Landing]
            # 85          | Future Xeon Phi
            # 8E 9E       | 7th gen. Core              [Kaby Lake]
            # 55          | Future Xeon                [Skylake w/ AVX512]
            # 4E 5E       | 6th gen. Core / E3 v5      [Skylake w/o AVX512]
            # 56          | Xeon D-1500                [Broadwell]
            # 4F          | Xeon E5 v4, E7 v4, i7-69xx [Broadwell]
            # 47          | 5th gen. Core / Xeon E3 v4 [Broadwell]
            # 3D          | M-5xxx / 5th gen.          [Broadwell]
            # 3F          | Xeon E5 v3, E7 v3, i7-59xx [Haswell-E]
            # 3C 45 46    | 4th gen. Core, Xeon E3 v3  [Haswell]
            # 3E          | Xeon E5 v2, E7 v2, i7-49xx [Ivy Bridge-E]
            # 3A          | 3rd gen. Core, Xeon E3 v2  [Ivy Bridge]
            # 2D          | Xeon E5, i7-39xx           [Sandy Bridge]
            # 2F          | Xeon E7
            # 2A          | Xeon E3, 2nd gen. Core     [Sandy Bridge]
            # 2E          | Xeon 7500, 6500 series
            # 25 2C       | Xeon 3600, 5600 series, Core i7, i5 and i3
            #
            # Values from the Intel SDE:
            # 5C | Goldmont
            # 5A | Silvermont
            # 57 | Knights Landing
            # 66 | Cannonlake
            # 55 | Skylake Server
            # 4E | Skylake Client
            # 3C | Broadwell (likely a bug in the SDE)
            # 3C | Haswell
            #
            # Latest updates taken from https://en.wikichip.org/wiki/intel/cpuid

            # MIC architecture
            if(_cpu_model EQUAL 133)
                set(TARGET_ARCHITECTURE "knm")  # Knights Mill

            elseif(_cpu_model EQUAL 87)
                set(TARGET_ARCHITECTURE "knl")  # Knights Landing

                # Small cores
            elseif(_cpu_model EQUAL 138 OR _cpu_model EQUAL 150 OR _cpu_model EQUAL 156)
                set(TARGET_ARCHITECTURE "tremont")

            elseif(_cpu_model EQUAL 122)
                set(TARGET_ARCHITECTURE "goldmont-plus")

            elseif(_cpu_model EQUAL 92 OR _cpu_model EQUAL 95)
                set(TARGET_ARCHITECTURE "goldmont")

            elseif(_cpu_model EQUAL 55 OR _cpu_model EQUAL 74 OR _cpu_model EQUAL 76 OR _cpu_model EQUAL 77 OR _cpu_model EQUAL 90 OR _cpu_model EQUAL 93)
                set(TARGET_ARCHITECTURE "silvermont")

            elseif(_cpu_model EQUAL 28 OR _cpu_model EQUAL 38 OR _cpu_model EQUAL 39 OR _cpu_model EQUAL 53 OR _cpu_model EQUAL 54)
                set(TARGET_ARCHITECTURE "bonnell")

                # Big cores (server)
            elseif(_cpu_model EQUAL 221)
                set(TARGET_ARCHITECTURE "clearwaterforest")

            elseif(_cpu_model EQUAL 175)
                set(TARGET_ARCHITECTURE "sierraforest")

            elseif(_cpu_model EQUAL 173 OR _cpu_model EQUAL 174)
                set(TARGET_ARCHITECTURE "graniterapids")

            elseif(_cpu_model EQUAL 207)
                set(TARGET_ARCHITECTURE "emeraldrapids")

            elseif(_cpu_model EQUAL 143)
                set(TARGET_ARCHITECTURE "sapphirerapids")

            elseif(_cpu_model EQUAL 106 OR _cpu_model EQUAL 108)
                set(TARGET_ARCHITECTURE "icelake-avx512")

            elseif(_cpu_model EQUAL 85)
                if(_cpu_stepping LESS 5)
                    set(TARGET_ARCHITECTURE "skylake-avx512")
                elseif(_cpu_stepping LESS 8)
                    set(TARGET_ARCHITECTURE "cascadelake")
                else()
                    set(TARGET_ARCHITECTURE "cooperlake")
                endif()

            elseif(_cpu_model EQUAL 79 OR _cpu_model EQUAL 86)
                set(TARGET_ARCHITECTURE "broadwell")

            elseif(_cpu_model EQUAL 63)
                set(TARGET_ARCHITECTURE "haswell")

            elseif(_cpu_model EQUAL 62)
                set(TARGET_ARCHITECTURE "ivybridge")

            elseif(_cpu_model EQUAL 45)
                set(TARGET_ARCHITECTURE "sandybridge")

            elseif(_cpu_model EQUAL 44 OR _cpu_model EQUAL 47)
                set(TARGET_ARCHITECTURE "westmere")

            elseif(_cpu_model EQUAL 26 OR _cpu_model EQUAL 30 OR _cpu_model EQUAL 46)
                set(TARGET_ARCHITECTURE "nehalem")

            elseif(_cpu_model EQUAL 23 OR _cpu_model EQUAL 29)
                set(TARGET_ARCHITECTURE "penryn")

                # Big cores (client)
            elseif(_cpu_model EQUAL 204)
                set(TARGET_ARCHITECTURE "pantherlake")

            elseif(_cpu_model EQUAL 188 OR _cpu_model EQUAL 189)
                set(TARGET_ARCHITECTURE "lunarlake")

            elseif(_cpu_model EQUAL 181 OR _cpu_model EQUAL 197 OR _cpu_model EQUAL 198)
                set(TARGET_ARCHITECTURE "arrowlake")

            elseif(_cpu_model EQUAL 170 OR _cpu_model EQUAL 171 OR _cpu_model EQUAL 172)
                set(TARGET_ARCHITECTURE "meteorlake")

            elseif(_cpu_model EQUAL 183 OR _cpu_model EQUAL 186 OR _cpu_model EQUAL 190 OR _cpu_model EQUAL 191)
                set(TARGET_ARCHITECTURE "raptorlake") # Raptor Lake refresh = Bartlett Lake

            elseif(_cpu_model EQUAL 151 OR _cpu_model EQUAL 154)
                set(TARGET_ARCHITECTURE "alderlake")

            elseif(_cpu_model EQUAL 167)
                set(TARGET_ARCHITECTURE "rocketlake")

            elseif(_cpu_model EQUAL 165 OR _cpu_model EQUAL 166)
                set(TARGET_ARCHITECTURE "cometlake")

            elseif(_cpu_model EQUAL 140 OR _cpu_model EQUAL 141)
                set(TARGET_ARCHITECTURE "tigerlake")

            elseif(_cpu_model EQUAL 125 OR _cpu_model EQUAL 126)
                set(TARGET_ARCHITECTURE "icelake")

            elseif(_cpu_model EQUAL 102)
                set(TARGET_ARCHITECTURE "cannonlake")

            elseif(_cpu_model EQUAL 142 OR _cpu_model EQUAL 158)
                set(TARGET_ARCHITECTURE "kabylake")

            elseif(_cpu_model EQUAL 78 OR _cpu_model EQUAL 94)
                set(TARGET_ARCHITECTURE "skylake")

            elseif(_cpu_model EQUAL 61 OR _cpu_model EQUAL 71)
                set(TARGET_ARCHITECTURE "broadwell")

            elseif(_cpu_model EQUAL 60 OR _cpu_model EQUAL 69 OR _cpu_model EQUAL 70)
                set(TARGET_ARCHITECTURE "haswell")

            elseif(_cpu_model EQUAL 58)
                set(TARGET_ARCHITECTURE "ivybridge")

            elseif(_cpu_model EQUAL 42)
                set(TARGET_ARCHITECTURE "sandybridge")

            elseif(_cpu_model EQUAL 37)
                set(TARGET_ARCHITECTURE "westmere")

            elseif(_cpu_model EQUAL 30 OR _cpu_model EQUAL 31)
                set(TARGET_ARCHITECTURE "nehalem")

            elseif(_cpu_model EQUAL 23 OR _cpu_model EQUAL 29)
                set(TARGET_ARCHITECTURE "penryn")

            elseif(_cpu_model EQUAL 15 OR _cpu_model EQUAL 22)
                set(TARGET_ARCHITECTURE "merom")

            elseif(_cpu_model EQUAL 28)
                set(TARGET_ARCHITECTURE "atom")

            elseif(_cpu_model EQUAL 14)
                set(TARGET_ARCHITECTURE "core")

            elseif(_cpu_model LESS 14)
                message(WARNING
          " [OptimizeForArchitecture] Your CPU is not known.\n"
          " \tAuto-detection of optimization flags failed and will use the generic CPU settings with SSE2.\n"
          " \tPlease send an email to gismo@inria.fr with the following content so that we can update the OFA script:\n"
          " \tVendor id:    ${_vendor_id}\n"
          " \tCPU family:   ${_cpu_family}\n"
          " \tCPU mode:     ${_cpu_model}\n"
          " \tCPU stepping: ${_cpu_stepping}\n"
          " \tCPU flags:    ${_cpu_flags}")
                set(TARGET_ARCHITECTURE "generic")
            else()
                message(WARNING
          " [OptimizeForArchitecture] Your CPU is not known.\n"
          " \tAuto-detection of optimization flags failed and will use the 65nm Core 2 CPU settings.\n"
          " \tPlease send an email to gismo@inria.fr with the following content so that we can update the OFA script:\n"
          " \tVendor id:    ${_vendor_id}\n"
          " \tCPU family:   ${_cpu_family}\n"
          " \tCPU mode:     ${_cpu_model}\n"
          " \tCPU stepping: ${_cpu_stepping}\n"
          " \tCPU flags:    ${_cpu_flags}")
                set(TARGET_ARCHITECTURE "merom")
            endif()

        elseif(_cpu_family EQUAL 7) # Itanium (not supported)
            message(WARNING "[OptimizeForArchitecture] Your CPU (Itanium: family ${_cpu_family}, model ${_cpu_model}) is not supported by OptimizeForArchitecture.cmake.")

        elseif(_cpu_family EQUAL 15) # NetBurst
            list(APPEND _available_vector_units_list "sse" "sse2")
            if(_cpu_model GREATER 2) # Not sure whether this must be 3 or even 4 instead
                list(APPEND _available_vector_units_list "sse" "sse2" "sse3")
            endif()

        endif()

    elseif(_vendor_id STREQUAL "AuthenticAMD")
        # taken from the list of AMD CPU microarchitectures
        # https://en.wikipedia.org/wiki/List_of_AMD_CPU_microarchitectures
        # CPUID Signature Values Of Recent AMD Microarchitectures
        # 05 05h      | K6
        # 06 06h      | K7
        # 15 0Fh      | K8 / Hammer
        # 16 10h      | K10
        # 17 11h      | K8 & K10 "hybrid"
        # 18 12h      | K10 (Llano) / K12 (ARM based AMD cpu)
        # 20 14h      | Bobcat
        # 21 15h      | Bulldozer / Piledriver / Steamroller / Excavator
        # 22 16h      | Jaguar / Puma
        # 23 17h      | Zen / Zen+ / Zen 2
        # 24 18h      | Hygon Dhyana
        # 25 19h      | Zen 3 / Zen 3+ / Zen 4
        # 26 1Ah      | Zen 5

        if(_cpu_family EQUAL 25) # 19h
            set(TARGET_ARCHITECTURE "zen3") # Some newer models will be Zen 4

        elseif(_cpu_family EQUAL 24) # 18h
            set(TARGET_ARCHITECTURE "zen")

        elseif(_cpu_family EQUAL 23) # 17h
            if(_cpu_model LESS 49)
                set(TARGET_ARCHITECTURE "zen")
            else()
                set(TARGET_ARCHITECTURE "zen2")
            endif()

        elseif(_cpu_family EQUAL 22) # 16h
            set(TARGET_ARCHITECTURE "amd16h")

        elseif(_cpu_family EQUAL 21) # 15h
            if(_cpu_model LESS 16)
                set(TARGET_ARCHITECTURE "bulldozer")
            elseif(_cpu_model LESS 32)
                set(TARGET_ARCHITECTURE "piledriver")
            elseif(_cpu_model LESS 64)
                set(TARGET_ARCHITECTURE "steamroller")
            else()
                set(TARGET_ARCHITECTURE "excavator")
            endif()

        elseif(_cpu_family EQUAL 20) # 14h
            set(TARGET_ARCHITECTURE "amd14h")

        elseif(_cpu_family EQUAL 18) # 12h (K10 / K12)

        elseif(_cpu_family EQUAL 17) # 12h (K8 & K10 hybrid)

        elseif(_cpu_family EQUAL 16) # 10h (K10)
            set(TARGET_ARCHITECTURE "barcelona")

        elseif(_cpu_family EQUAL 15) # 0Fh (K8 / Hammer)
            if(_cpu_model LESS 39)
                set(TARGET_ARCHITECTURE "k8")
            else()
                set(TARGET_ARCHITECTURE "k8-sse3")
            endif()

        elseif(_cpu_family EQUAL 6) # 06h (K7)
        elseif(_cpu_family EQUAL 5) # 05h (K6)

        endif()

    else()
        message(WARNING "[OptimizeForArchitecture] Auto-detection of optimization flags failed and will use the generic CPU settings.")
        return()
    endif()

    if(OFA_VERBOSE)
        message(STATUS "[OptimizeForArchitecture] Vendor id:    ${_vendor_id}")
        message(STATUS "[OptimizeForArchitecture] CPU family:   ${_cpu_family}")
        message(STATUS "[OptimizeForArchitecture] CPU mode:     ${_cpu_model}")
        message(STATUS "[OptimizeForArchitecture] CPU stepping: ${_cpu_stepping}")
    endif()
endmacro(OFA_AutodetectX86)
