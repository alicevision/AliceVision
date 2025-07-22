# Headers
set(system_files_headers
    cpu.hpp
    main.hpp
    MemoryInfo.hpp
    system.hpp
    Timer.hpp
    Logger.hpp
    ProgressDisplay.hpp
    nvtx.hpp
    hardwareContext.hpp
)

# Sources
set(system_files_sources
    cpu.cpp
    MemoryInfo.cpp
    Timer.cpp
    Logger.cpp
    ProgressDisplay.cpp
    nvtx.cpp
    hardwareContext.cpp
)

alicevision_add_library(aliceVision_system
    SOURCES ${system_files_headers} ${system_files_sources}
    PUBLIC_LINKS
        Boost::log
        Boost::log_setup
        Boost::thread
        Boost::system
        Boost::date_time
        ${ALICEVISION_NVTX_LIBRARY}
    PRIVATE_LINKS
        Boost::boost
)

alicevision_add_test(Logger_test.cpp NAME "system_Logger" LINKS aliceVision_system)

# SWIG Binding
if (ALICEVISION_BUILD_SWIG_BINDING)
    alicevision_swig_add_library(system
        SOURCES System.i
        PUBLIC_LINKS
            aliceVision_system
            ${Python3_LIBRARIES}
        PRIVATE_INCLUDE_DIRS
            ../include
            ${ALICEVISION_ROOT}/include
            ${Python3_INCLUDE_DIRS}
            ${Python3_NumPy_INCLUDE_DIRS}
    )
endif()
