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

    av_add_cmake_dep(
        TARGET         ${BOOST_TARGET}
        SOURCE_DIR     boost
        URL            ${DEP_BOOST_URL}
        URL_HASH       ${DEP_BOOST_HASH}
        EXTRA_CMAKE_FLAGS
          -DBOOST_INCLUDE_LIBRARIES=${DEP_BOOST_LIBS_SEMICOLON_ESCAPED}
          -DBUILD_TESTING=OFF
        DEPENDS ${ZLIB_TARGET}
    )

    set(BOOST_CMAKE_FLAGS -DBOOST_ROOT=${CMAKE_INSTALL_PREFIX})
endif()
