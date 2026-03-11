## AliceVision
## CMake Helpers

# Add library function
function(alicevision_add_library library_name)
    set(options USE_CUDA)
    set(singleValues "")
    set(multipleValues SOURCES PUBLIC_LINKS PRIVATE_LINKS PUBLIC_INCLUDE_DIRS PRIVATE_INCLUDE_DIRS PUBLIC_DEFINITIONS PRIVATE_DEFINITIONS)

    cmake_parse_arguments(LIBRARY "${options}" "${singleValues}" "${multipleValues}" ${ARGN})

    if (NOT library_name)
        message(FATAL_ERROR "You must provide the library name in 'alicevision_add_library'")
    endif()

    if(NOT LIBRARY_SOURCES)
        message(FATAL_ERROR "You must provide the library SOURCES in 'alicevision_add_library'")
    endif()

    # Generate Windows versioning information
    if (MSVC)
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

    if (NOT LIBRARY_USE_CUDA)
        add_library(${library_name} ${LIBRARY_SOURCES})
    
        if (ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
            append_coverage_compiler_flags_to_target(${library_name})
        endif()

    elseif (BUILD_SHARED_LIBS)
        if(MSVC)
            if(CMAKE_BUILD_TYPE MATCHES "Debug")
                set(CUDA_LIB_OPTIONS --compiler-options "/MDd")
            else()
                set(CUDA_LIB_OPTIONS --compiler-options "/MD")
            endif()
        else()
            set(CUDA_LIB_OPTIONS --compiler-options "-fPIC")
        endif()
        cuda_add_library(${library_name} SHARED ${LIBRARY_SOURCES} OPTIONS ${CUDA_LIB_OPTIONS})
    else()
        if(MSVC)
            if(CMAKE_BUILD_TYPE MATCHES "Debug")
                set(CUDA_LIB_OPTIONS --compiler-options "/MTd")
            else()
                set(CUDA_LIB_OPTIONS --compiler-options "/MT")
            endif()
        else()
            set(CUDA_LIB_OPTIONS --compiler-options "-fPIC")
        endif()
        cuda_add_library(${library_name} ${LIBRARY_SOURCES} OPTIONS ${CUDA_LIB_OPTIONS})
    endif()

    if (ALICEVISION_REMOVE_ABSOLUTE)
        foreach (item ${LIBRARY_PUBLIC_LINKS})
            get_filename_component(nameItem ${item} NAME)
            list(APPEND TRANSFORMED_LIBRARY_PUBLIC_LINKS ${nameItem})
        endforeach()
    else()
        set(TRANSFORMED_LIBRARY_PUBLIC_LINKS ${LIBRARY_PUBLIC_LINKS})
    endif()

    # FindCUDA.cmake implicit	target_link_libraries() can not be mixed with new signature (CMake < 3.9.0)
    if (NOT LIBRARY_USE_CUDA)
        target_link_libraries(${library_name}
            PUBLIC ${TRANSFORMED_LIBRARY_PUBLIC_LINKS}
            PRIVATE ${LIBRARY_PRIVATE_LINKS}
        )
    else()
        target_link_libraries(${library_name}
            ${TRANSFORMED_LIBRARY_PUBLIC_LINKS}
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

    if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1914))
        target_compile_options(${library_name} PUBLIC "/Zc:__cplusplus")
    endif()

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

    if (NOT interface_name)
        message(FATAL_ERROR "You must provide the interface name in 'alicevision_add_interface'")
    endif()

    if (NOT LIBRARY_SOURCES)
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

    if (NOT software_name)
        message(FATAL_ERROR "You must provide the software name in 'alicevision_add_software'")
    endif()

    if (NOT SOFTWARE_SOURCE)
        message(FATAL_ERROR "You must provide the software SOURCE in 'alicevision_add_software'")
    endif()

    if (NOT SOFTWARE_FOLDER)
        message(FATAL_ERROR "You must provide the software FOLDER in 'alicevision_add_software'")
    endif()

    list(GET SOFTWARE_SOURCE 0 SOFTWARE_MAIN_CPP)
    file(STRINGS ${SOFTWARE_MAIN_CPP} _ALICEVISION_SOFTWARE_CONTENTS REGEX "#define ALICEVISION_SOFTWARE_VERSION_")

    foreach (v MAJOR MINOR)
        if ("${_ALICEVISION_SOFTWARE_CONTENTS}" MATCHES "#define ALICEVISION_SOFTWARE_VERSION_${v} ([0-9]+)")
            set(ALICEVISION_SOFTWARE_VERSION_${v} "${CMAKE_MATCH_1}")
        else()
            message(FATAL_ERROR "Failed to retrieve the AliceVision software version the source code. Missing ALICEVISION_SOFTWARE_VERSION_${v}.")
        endif()
    endforeach()

    # Generate Windows versioning information
    if (MSVC)
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

    add_executable(${software_name}_exe ${SOFTWARE_SOURCE})
    set_target_properties(${software_name}_exe PROPERTIES
        OUTPUT_NAME ${software_name}
    )

    if (ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
        append_coverage_compiler_flags_to_target(${software_name}_exe)
    endif()

    target_link_libraries(${software_name}_exe
        PUBLIC ${SOFTWARE_LINKS}
    )

    target_include_directories(${software_name}_exe
        PUBLIC ${SOFTWARE_INCLUDE_DIRS}
    )

    if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1914))
        target_compile_options(${software_name}_exe PUBLIC "/Zc:__cplusplus")
    endif()

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

    if (NOT test_file)
        message(FATAL_ERROR "You must provide the test file in 'alicevision_add_test'")
    endif()

    if (NOT TEST_NAME)
        message(FATAL_ERROR "You must provide the NAME in 'alicevision_add_test'")
    endif()

    if (NOT ALICEVISION_BUILD_TESTS)
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

    if ((MSVC) AND (MSVC_VERSION GREATER_EQUAL 1914))
        target_compile_options(${TEST_EXECUTABLE_NAME} PUBLIC "/Zc:__cplusplus")
    endif()

    add_test(NAME test_${TEST_EXECUTABLE_NAME}
        WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
        COMMAND $<TARGET_FILE:${TEST_EXECUTABLE_NAME}> --catch_system_error=yes --log_level=all
    )
   
    if (ALICEVISION_BUILD_COVERAGE AND CMAKE_COMPILER_IS_GNUCXX)
        append_coverage_compiler_flags_to_target(${TEST_EXECUTABLE_NAME})
    endif()

    if (UNIX)
        # setup LD_LIBRARY_PATH for running tests
        get_property(TEST_LINK_DIRS TARGET ${TEST_EXECUTABLE_NAME} PROPERTY LINK_DIRECTORIES)
    endif()

    if (WIN32)
        set_property(TEST test_${TEST_EXECUTABLE_NAME} PROPERTY ENVIRONMENT "ALICEVISION_ROOT=${CMAKE_INSTALL_PREFIX}")
    endif()
endfunction()


# Add SWIG library function
function(alicevision_swig_add_library module_name)
    set(multipleValues SOURCES PUBLIC_LINKS PRIVATE_INCLUDE_DIRS)
    cmake_parse_arguments(SWIG_MODULE "" "" "${multipleValues}" ${ARGN})

    set_property(SOURCE ${SWIG_MODULE_SOURCES} PROPERTY CPLUSPLUS ON)
    set_property(SOURCE ${SWIG_MODULE_SOURCES} PROPERTY SWIG_MODULE_NAME ${module_name})

    swig_add_library(${module_name}
        TYPE MODULE
        LANGUAGE python
        SOURCES ${SWIG_MODULE_SOURCES}
    )

    if (CMAKE_SYSTEM_NAME MATCHES "Linux" OR UNIX)
        list(APPEND SWIG_EXTRA_COMPILE_OPTIONS "-DLINUXPLATFORM")
    endif()

    set_property(
        TARGET ${module_name}
        PROPERTY SWIG_COMPILE_OPTIONS -doxygen ${SWIG_EXTRA_COMPILE_OPTIONS}
    )

    target_include_directories(${module_name}
        PRIVATE ${SWIG_MODULE_PRIVATE_INCLUDE_DIRS}
    )
    set_property(
        TARGET ${module_name}
        PROPERTY SWIG_USE_TARGET_INCLUDE_DIRECTORIES ON
    )
    set_property(
        TARGET ${module_name}
        PROPERTY COMPILE_OPTIONS -std=c++20
    )

    target_link_libraries(${module_name}
        PUBLIC ${SWIG_MODULE_PUBLIC_LINKS}
    )

    install(
        TARGETS
            ${module_name}
        DESTINATION
            ${ALICEVISION_PYTHON_INSTALL_DIR}
    )

    install(
        FILES
            ${CMAKE_CURRENT_BINARY_DIR}/${module_name}.py
        DESTINATION
            ${ALICEVISION_PYTHON_INSTALL_DIR}
    )
endfunction()


# -----------------------------------------------------------------------------
# av_register_dep(TARGET)
#
# Appends TARGET to the global AV_DEPS list, which is later passed as the
# DEPENDS list of the top-level aliceVision ExternalProject.
# -----------------------------------------------------------------------------
macro(av_register_dep TARGET)
    list(APPEND AV_DEPS ${TARGET})
endmacro()

# -----------------------------------------------------------------------------
# av_add_cmake_dep(
#     TARGET             <target-name>
#     SOURCE_DIR         <relative-subdir-name>   # under CMAKE_CURRENT_BINARY_DIR
#     [URL               <url>]
#     [URL_HASH          <ALGO=hash>]
#     [GIT_REPOSITORY    <repo>]
#     [GIT_TAG           <tag-or-commit>]
#     [EXTRA_CMAKE_FLAGS <flag> ...]   # appended after CMAKE_CORE_BUILD_FLAGS
#     [DEPENDS           <dep> ...]
#     [DOWNLOAD_DIR      <absolute-path>]
#     [NO_REGISTER]                   # skip av_register_dep (rare)
# )
#
# Registers a standard CMake-based ExternalProject with sensible defaults:
#   - PREFIX / BINARY_DIR derived from BUILD_DIR and TARGET name
#   - INSTALL_DIR = CMAKE_INSTALL_PREFIX
#   - BUILD_COMMAND = cmake --build (generator-agnostic: works with Ninja, Make, etc.)
#   - UPDATE_COMMAND = ""  (reproducible builds)
# -----------------------------------------------------------------------------
macro(av_add_cmake_dep)
    cmake_parse_arguments(
        _D                          # prefix
        "NO_REGISTER"               # options
        "TARGET;SOURCE_DIR;URL;URL_HASH;GIT_REPOSITORY;GIT_TAG;DOWNLOAD_DIR"
        "EXTRA_CMAKE_FLAGS;DEPENDS"
        ${ARGN}
    )

    if(NOT _D_DOWNLOAD_DIR)
        set(_D_DOWNLOAD_DIR "${BUILD_DIR}/download/${_D_TARGET}")
    endif()

    # Build source arguments depending on fetch method
    set(_D_SRC_ARGS "")
    if(_D_URL)
        list(APPEND _D_SRC_ARGS
            URL              ${_D_URL}
            DOWNLOAD_DIR     ${_D_DOWNLOAD_DIR}
            DOWNLOAD_NO_PROGRESS TRUE
        )
        if(_D_URL_HASH)
            list(APPEND _D_SRC_ARGS URL_HASH ${_D_URL_HASH})
        endif()
    elseif(_D_GIT_REPOSITORY)
        list(APPEND _D_SRC_ARGS GIT_REPOSITORY ${_D_GIT_REPOSITORY})
        if(_D_GIT_TAG)
            list(APPEND _D_SRC_ARGS GIT_TAG ${_D_GIT_TAG})
        endif()
    endif()

    ExternalProject_Add(${_D_TARGET}
        ${_D_SRC_ARGS}
        PREFIX          ${BUILD_DIR}
        BUILD_IN_SOURCE 0
        BUILD_ALWAYS    0
        UPDATE_COMMAND  ""
        SOURCE_DIR      ${CMAKE_CURRENT_BINARY_DIR}/${_D_SOURCE_DIR}
        BINARY_DIR      ${BUILD_DIR}/${_D_TARGET}_build
        INSTALL_DIR     ${CMAKE_INSTALL_PREFIX}
        CONFIGURE_COMMAND
            ${CMAKE_COMMAND}
            ${CMAKE_CORE_BUILD_FLAGS}
            ${_D_EXTRA_CMAKE_FLAGS}
            -DCMAKE_POLICY_VERSION_MINIMUM=3.5
            -DCMAKE_INSTALL_PREFIX:PATH=<INSTALL_DIR>
            <SOURCE_DIR>
        BUILD_COMMAND
            ${CMAKE_COMMAND} --build <BINARY_DIR>
                --config ${DEPS_CMAKE_BUILD_TYPE}
                --parallel ${AV_BUILD_DEPENDENCIES_PARALLEL}
        DEPENDS         ${_D_DEPENDS}
    )

    if(NOT _D_NO_REGISTER)
        av_register_dep(${_D_TARGET})
    endif()

    # Clean up local variables to avoid leaking between macro calls
    unset(_D_SRC_ARGS)
    unset(_D_DOWNLOAD_DIR)
endmacro()
