# =============================================================================
# deps/boost.cmake
# Dependencies: zlib
# Provides:     BOOST_TARGET, BOOST_CMAKE_FLAGS
# =============================================================================

if(AV_BUILD_BOOST)
    set(BOOST_TARGET boost)

    set(DEP_BOOST_LIBS
      accumulators
      atomic
      container
      date_time
      exception
      foreach
      format
      geometry
      graph
      iostreams
      json
      log
      math
      multi_array
      program_options
      ptr_container
      regex
      serialization
      system
      test
      thread
      timer
      stacktrace
    )

    # Helper to build a semicolon seperated list which survives the
    # ExternProject_Add call
    set(DEP_BOOST_LIBS_SEMICOLON_ESCAPED "")
    foreach(DEP_BOOST_LIB ${DEP_BOOST_LIBS})
        if(DEP_BOOST_LIBS_SEMICOLON_ESCAPED STREQUAL "")
            set(DEP_BOOST_LIBS_SEMICOLON_ESCAPED "${DEP_BOOST_LIB}")
        else()
            set(DEP_BOOST_LIBS_SEMICOLON_ESCAPED "${DEP_BOOST_LIBS_SEMICOLON_ESCAPED}$<SEMICOLON>${DEP_BOOST_LIB}")
        endif()
    endforeach()

    # We might cross-compile architectures on Apple platforms and Boost.Context
    # cannot work with CMAKE_OSX_ARCHITECTURES yet, so we must pass some flags
    # manually.
    if(APPLE)
      if(${CMAKE_OSX_ARCHITECTURES} STREQUAL "arm64")
        set(DEP_BOOST_CONTEXT_APPLE_FLAGS -DBOOST_CONTEXT_ARCHITECTURE=arm64 -DBOOST_CONTEXT_ABI=aapcs -DBOOST_IOSTREAMS_ENABLE_ZSTD=OFF)
      else()
        set(DEP_BOOST_CONTEXT_APPLE_FLAGS -DBOOST_CONTEXT_ARCHITECTURE=x86_64 -DBOOST_CONTEXT_ABI=sysv -DBOOST_IOSTREAMS_ENABLE_ZSTD=OFF)
      endif()
    endif()

    av_add_cmake_dep(
        TARGET         ${BOOST_TARGET}
        SOURCE_DIR     boost
        URL            ${DEP_BOOST_URL}
        URL_HASH       ${DEP_BOOST_HASH}
        EXTRA_CMAKE_FLAGS
          -DBOOST_INCLUDE_LIBRARIES=${DEP_BOOST_LIBS_SEMICOLON_ESCAPED}
          -DBUILD_TESTING=OFF
          ${DEP_BOOST_CONTEXT_APPLE_FLAGS}
        DEPENDS ${ZLIB_TARGET}
    )

    set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX})
endif()
