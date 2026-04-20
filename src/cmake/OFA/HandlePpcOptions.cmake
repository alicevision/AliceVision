#=============================================================================
# Handling of PPC / PPC64 options
#
# This is a three-step process:
#
# 1. Generate a list of available compiler flags for the specific CPU
#
# 2. Enable/disable feature flags based on available CPU features,
#    used-defined USE_<feature> variables and the capabilities of the
#    host system's compiler and linker
#
# 3. Set compiler-specific flags (e.g., -m<feature>/-mno-<feature>)
#=============================================================================

include(OFA/CommonMacros)

macro(OFA_HandlePpcOptions)

  # Special treatment for "native" flag
  if(TARGET_ARCHITECTURE STREQUAL "native")
    if(CMAKE_CXX_COMPILER_ID MATCHES "NVHPC" OR
       CMAKE_CXX_COMPILER_ID MATCHES "PGI")
      # NVidia HPC / PGI
      AddCXXCompilerFlag("-tp=native" FLAGS ARCHITECTURE_CXX_FLAGS RESULT _ok)
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "XL")
      # IBM XL (on Linux/AIX)
      AddCXXCompilerFlag("-qarch=auto" FLAGS ARCHITECTURE_CXX_FLAGS RESULT _ok)
    else()
      # Others: GNU, Clang and variants
      AddCXXCompilerFlag("-march=native" FLAGS ARCHITECTURE_CXX_FLAGS RESULT _ok)
    endif()

    if(NOT _ok)
      message(FATAL_ERROR "[OptimizeForArchitecture] An error occured while setting the \"native\" flag.")
    endif()

  elseif(NOT TARGET_ARCHITECTURE STREQUAL "none")
    
    # Step 1: Generate a list of compiler flags for the specific CPU
    set(_march_flag_list)
    
    # Define macros for PowerPC64
    macro(_power3)
      list(APPEND _march_flag_list "power3")
    endmacro()
    macro(_power4)
      list(APPEND _march_flag_list "power4")
      _power3()
    endmacro()
    macro(_power5)
      list(APPEND _march_flag_list "power5")
      _power4()
    endmacro()
    macro(_power5plus)
      list(APPEND _march_flag_list "power5+")
      _power5()
    endmacro()
    macro(_power6)
      list(APPEND _march_flag_list "power6")
      _power5()
    endmacro()
    macro(_power6x)
      list(APPEND _march_flag_list "power6x")
      _power6()
    endmacro()
    macro(_power7)
      list(APPEND _march_flag_list "power7")
      _power6()
    endmacro()
    macro(_power8)
      list(APPEND _march_flag_list "pwr8")
      list(APPEND _march_flag_list "power8")
      _power7()
    endmacro()
    macro(_power9)
      list(APPEND _march_flag_list "pwr9")
      list(APPEND _march_flag_list "power9")
      _power8()
    endmacro()
    macro(_power10)
      list(APPEND _march_flag_list "pwr10")
      list(APPEND _march_flag_list "power10")
      _power9()
    endmacro()
  
    # PowerPC64
    if(TARGET_ARCHITECTURE STREQUAL "power3")
      _power3()
    elseif(TARGET_ARCHITECTURE STREQUAL "power4")
      _power4()
    elseif(TARGET_ARCHITECTURE STREQUAL "power5")
      _power5()
    elseif(TARGET_ARCHITECTURE STREQUAL "power5+")
      _power5plus()
    elseif(TARGET_ARCHITECTURE STREQUAL "power6")
      _power6()
    elseif(TARGET_ARCHITECTURE STREQUAL "power6x")
      _power6x()
    elseif(TARGET_ARCHITECTURE STREQUAL "power7")
      _power7()
    elseif(TARGET_ARCHITECTURE STREQUAL "power8")
      _power8()
    elseif(TARGET_ARCHITECTURE STREQUAL "power9")
      _power9()
    elseif(TARGET_ARCHITECTURE STREQUAL "power10")
      _power10()

      # Others
    elseif(TARGET_ARCHITECTURE STREQUAL "generic")
      list(APPEND _march_flag_list "generic")
    elseif(TARGET_ARCHITECTURE STREQUAL "none")
      # add this clause to remove it from the else clause

    else()
      message(FATAL_ERROR "[OptimizeForArchitecture] Unknown target architecture: \"${TARGET_ARCHITECTURE}\". Please set TARGET_ARCHITECTURE to a supported value.")
    endif()

    # Step 2: We do not enable/disable feature flags for PPC/PPC64 CPUs

    # Step 3: Set compiler-specific flags (e.g., -m<feature>/-mno-<feature>)
    if(CMAKE_CXX_COMPILER_ID MATCHES "XL")

      # Set -qarch flag
      foreach(_flag ${_march_flag_list})
        AddCXXCompilerFlag("-mcpu=${_flag}" FLAGS ARCHITECTURE_CXX_FLAGS RESULT _good)
        AddCXXCompilerFlag("-qarch=${_flag}" FLAGS ARCHITECTURE_CXX_FLAGS RESULT _good)
        if(_good)
          break()
        endif(_good)
      endforeach(_flag)

    elseif(CMAKE_CXX_COMPILER_ID MATCHES "NVHPC"
        OR CMAKE_CXX_COMPILER_ID MATCHES "PGI")

      # Set -tp flag
      foreach(_flag ${_march_flag_list})
        AddCXXCompilerFlag("-tp=${_flag}" FLAGS ARCHITECTURE_CXX_FLAGS RESULT _good)
        if(_good)
          break()
        endif(_good)
      endforeach(_flag)

    else()
      # Others: GNU, Clang and variants
      
      # Set -march flag
      foreach(_flag ${_march_flag_list})
        AddCXXCompilerFlag("-march=${_flag}" FLAGS ARCHITECTURE_CXX_FLAGS RESULT _good)
        if(_good)
          break()
        endif(_good)
      endforeach(_flag)

    endif()
  endif()

  # Compile code with profiling instrumentation
  if(TARGET_PROFILER STREQUAL "gprof")
    AddCXXCompilerFlag("-pg" FLAGS ARCHITECTURE_CXX_FLAGS)
  endif()

  # Remove duplicate flags
  list(REMOVE_DUPLICATES ARCHITECTURE_CXX_FLAGS)

  if(OFA_VERBOSE)
    string(REPLACE ";"  ", " _str "${ARCHITECTURE_CXX_FLAGS}")
    message(STATUS "[OptimizeForArchitecture] ARCHITECTURE_CXX_FLAGS: " ${_str})
  endif()

endmacro(OFA_HandlePpcOptions)
