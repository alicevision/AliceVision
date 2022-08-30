## AliceVision
## CMake Helpers

# Add library function
function(alicevision_add_library library_name)
  set(options USE_CUDA)
  set(singleValues "")
  set(multipleValues SOURCES PUBLIC_LINKS PRIVATE_LINKS PUBLIC_INCLUDE_DIRS PRIVATE_INCLUDE_DIRS PUBLIC_DEFINITIONS PRIVATE_DEFINITIONS)

  cmake_parse_arguments(LIBRARY "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

  if(NOT library_name)
    message(FATAL_ERROR "You must provide the library name in 'alicevision_add_library'")
  endif()

  if(NOT LIBRARY_SOURCES)
    message(FATAL_ERROR "You must provide the library SOURCES in 'alicevision_add_library'")
  endif()

  # Generate Windows versioning information
  if(MSVC)
    set(ALICEVISION_INSTALL_VERSION_MAJOR ${ALICEVISION_VERSION_MAJOR})
    set(ALICEVISION_INSTALL_VERSION_MINOR ${ALICEVISION_VERSION_MINOR})
    set(ALICEVISION_INSTALL_VERSION_REVISION ${ALICEVISION_VERSION_REVISION})
    set(ALICEVISION_INSTALL_NAME ${library_name})
    set(ALICEVISION_INSTALL_LIBRARY 1)
    configure_file(
      "${CMAKE_SOURCE_DIR}/src/cmake/version.rc.in"
      "${CMAKE_CURRENT_BINARY_DIR}/${library_name}_version.rc"
      @ONLY
    )
    list(APPEND LIBRARY_SOURCES "${CMAKE_CURRENT_BINARY_DIR}/${library_name}_version.rc")
  endif()

  if(NOT LIBRARY_USE_CUDA)
    add_library(${library_name} ${LIBRARY_SOURCES})
  elseif(BUILD_SHARED_LIBS)
    cuda_add_library(${library_name} SHARED ${LIBRARY_SOURCES} OPTIONS --compiler-options "-fPIC")
  else()
    cuda_add_library(${library_name} ${LIBRARY_SOURCES})
  endif()

  # FindCUDA.cmake implicit	target_link_libraries() can not be mixed with new signature (CMake < 3.9.0)
  if(NOT LIBRARY_USE_CUDA)
    target_link_libraries(${library_name}
      PUBLIC ${LIBRARY_PUBLIC_LINKS}
      PRIVATE ${LIBRARY_PRIVATE_LINKS}
    )
  else()
    target_link_libraries(${library_name}
       ${LIBRARY_PUBLIC_LINKS}
       ${LIBRARY_PRIVATE_LINKS}
    )
  endif()

  target_include_directories(${library_name}
    PUBLIC $<BUILD_INTERFACE:${ALICEVISION_INCLUDE_DIR}>
           $<BUILD_INTERFACE:${generatedDir}>
           $<INSTALL_INTERFACE:include>
           ${LIBRARY_PUBLIC_INCLUDE_DIRS}

    PRIVATE ${LIBRARY_PRIVATE_INCLUDE_DIRS}
  )

  target_compile_definitions(${library_name}
    PUBLIC ${LIBRARY_PUBLIC_DEFINITIONS}
    PRIVATE ${LIBRARY_PRIVATE_DEFINITIONS}
  )

  set_property(TARGET ${library_name}
    PROPERTY FOLDER "AliceVision"
  )

  set_target_properties(${library_name}
    PROPERTIES SOVERSION ${ALICEVISION_VERSION_MAJOR}
    VERSION "${ALICEVISION_VERSION_MAJOR}.${ALICEVISION_VERSION_MINOR}"
  )

  install(TARGETS ${library_name}
    EXPORT aliceVision-targets
    ARCHIVE
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY
        DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME
        DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endfunction()

# Add interface function
function(alicevision_add_interface interface_name)
  set(options "")
  set(singleValues NAME)
  set(multipleValues SOURCES LINKS)

  cmake_parse_arguments(LIBRARY "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

  if(NOT interface_name)
    message(FATAL_ERROR "You must provide the interface name in 'alicevision_add_interface'")
  endif()

  if(NOT LIBRARY_SOURCES)
    message(FATAL_ERROR "You must provide the interface SOURCES in 'alicevision_add_interface'")
  endif()

  add_library(${interface_name} INTERFACE)

  target_link_libraries(${interface_name}
    INTERFACE ${LIBRARY_LINKS}
  )

  install(TARGETS ${interface_name}
    EXPORT aliceVision-targets
  )

  set(LIBRARY_NAME_INTERFACE "${interface_name}_interface")
  add_custom_target(${LIBRARY_NAME_INTERFACE} SOURCES ${LIBRARY_SOURCES})

  set_property(TARGET ${LIBRARY_NAME_INTERFACE}
    PROPERTY FOLDER "AliceVision"
  )
endfunction()

# Add software function
function(alicevision_add_software software_name)
  set(options "")
  set(singleValues FOLDER)
  set(multipleValues SOURCE LINKS INCLUDE_DIRS)

  cmake_parse_arguments(SOFTWARE "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

  if(NOT software_name)
    message(FATAL_ERROR "You must provide the software name in 'alicevision_add_software'")
  endif()

  if(NOT SOFTWARE_SOURCE)
    message(FATAL_ERROR "You must provide the software SOURCE in 'alicevision_add_software'")
  endif()

  if(NOT SOFTWARE_FOLDER)
    message(FATAL_ERROR "You must provide the software FOLDER in 'alicevision_add_software'")
  endif()

  list(GET SOFTWARE_SOURCE 0 SOFTWARE_MAIN_CPP)
  file(STRINGS ${SOFTWARE_MAIN_CPP} _ALICEVISION_SOFTWARE_CONTENTS REGEX "#define ALICEVISION_SOFTWARE_VERSION_")

  foreach(v MAJOR MINOR)
    if("${_ALICEVISION_SOFTWARE_CONTENTS}" MATCHES "#define ALICEVISION_SOFTWARE_VERSION_${v} ([0-9]+)")
      set(ALICEVISION_SOFTWARE_VERSION_${v} "${CMAKE_MATCH_1}")
    else()
      message(FATAL_ERROR "Failed to retrieve the AliceVision software version the source code. Missing ALICEVISION_SOFTWARE_VERSION_${v}.")
    endif()
  endforeach()

  # Generate Windows versioning information
  if(MSVC)
    set(ALICEVISION_INSTALL_VERSION_MAJOR ${ALICEVISION_SOFTWARE_VERSION_MAJOR})
    set(ALICEVISION_INSTALL_VERSION_MINOR ${ALICEVISION_SOFTWARE_VERSION_MINOR})
    set(ALICEVISION_INSTALL_VERSION_REVISION 0)
    set(ALICEVISION_INSTALL_NAME ${software_name})
    set(ALICEVISION_INSTALL_LIBRARY 0) # software
    configure_file(
      "${CMAKE_SOURCE_DIR}/src/cmake/version.rc.in"
      "${CMAKE_CURRENT_BINARY_DIR}/${software_name}_version.rc"
      @ONLY
    )
    list(APPEND SOFTWARE_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/${software_name}_version.rc")
  endif()

  # Declare a static library with all the code of the executable. End users may link this static
  # library to their executables to use AliceVision on e.g. mobile platforms where stuff like
  # dlopen and dlsym are not well supported.
  #
  # The aliceVision_main symbol is redefined to unique name that depends on ${software_name} so
  # that multiple such static libraries can be linked into single executable.
  add_library(${software_name}_static_lib STATIC ${SOFTWARE_SOURCE})

  target_compile_definitions(${software_name}_static_lib
    PRIVATE -DaliceVision_main=aliceVision_main_${software_name})

  target_link_libraries(${software_name}_static_lib
    PUBLIC ${SOFTWARE_LINKS}
  )

  target_include_directories(${software_name}_static_lib
    PUBLIC ${SOFTWARE_INCLUDE_DIRS}
  )

  get_property(all_static_libs GLOBAL PROPERTY global_all_static_libs)
  set(all_static_libs ${all_static_libs} ${software_name}_static_lib)
  set_property(GLOBAL PROPERTY global_all_static_libs ${all_static_libs})

  # The executable will depend on the static library and will only include a main() function that
  # calls the aliceVision_main symbol.
  add_executable(${software_name}_exe ${SOFTWARE_SOURCE}
                 "${CMAKE_SOURCE_DIR}/src/aliceVision/system/main.cpp")

  target_compile_definitions(${software_name}_exe
    PRIVATE -DaliceVision_main=aliceVision_main_${software_name})

  set_target_properties(${software_name}_exe PROPERTIES
    OUTPUT_NAME ${software_name}
    )

  target_link_libraries(${software_name}_exe
    PRIVATE ${software_name}_static_lib
  )

  set_property(TARGET ${software_name}_exe
    PROPERTY FOLDER ${SOFTWARE_FOLDER}
  )

  set_target_properties(${software_name}_exe
     PROPERTIES SOVERSION ${ALICEVISION_SOFTWARE_VERSION_MAJOR}
     VERSION "${ALICEVISION_SOFTWARE_VERSION_MAJOR}.${ALICEVISION_SOFTWARE_VERSION_MINOR}"
  )

  install(TARGETS ${software_name}_exe
    RUNTIME
      DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endfunction()

# Add test function
function(alicevision_add_test test_file)
  set(options "")
  set(singleValues NAME)
  set(multipleValues LINKS INCLUDE_DIRS)

  cmake_parse_arguments(TEST "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

  if(NOT test_file)
    message(FATAL_ERROR "You must provide the test file in 'alicevision_add_test'")
  endif()

  if(NOT TEST_NAME)
    message(FATAL_ERROR "You must provide the NAME in 'alicevision_add_test'")
  endif()

  if(NOT ALICEVISION_BUILD_TESTS)
    return()
  endif()

  set(TEST_EXECUTABLE_NAME "aliceVision_test_${TEST_NAME}")

  add_executable(${TEST_EXECUTABLE_NAME} ${test_file})

  target_link_libraries(${TEST_EXECUTABLE_NAME}
    PUBLIC ${TEST_LINKS}
           ${ALICEVISION_LIBRARY_DEPENDENCIES}
           Boost::unit_test_framework
           Boost::log
  )

  target_include_directories(${TEST_EXECUTABLE_NAME}
    PUBLIC ${TEST_INCLUDE_DIRS}
  )

  set_property(TARGET ${TEST_EXECUTABLE_NAME}
    PROPERTY FOLDER Test
  )

  add_test(NAME test_${TEST_EXECUTABLE_NAME}
           WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
           COMMAND $<TARGET_FILE:${TEST_EXECUTABLE_NAME}> --catch_system_error=yes --log_level=all
  )

  if(UNIX)
    # setup LD_LIBRARY_PATH for running tests
    get_property(TEST_LINK_DIRS TARGET ${TEST_EXECUTABLE_NAME} PROPERTY LINK_DIRECTORIES)

    set_property(TEST test_${TEST_EXECUTABLE_NAME} PROPERTY ENVIRONMENT "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}:${TEST_LINK_DIRS}:$ENV{LD_LIBRARY_PATH}")
  endif()

endfunction()
