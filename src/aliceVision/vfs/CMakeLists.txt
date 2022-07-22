# Headers
set(vfs_files_headers
    boost_common.hpp
    filesystem.hpp
    path.hpp
)

# Sources
set(vfs_files_sources
    filesystem.cpp
    path.cpp
)

alicevision_add_library(aliceVision_vfs
  SOURCES ${vfs_files_headers} ${vfs_files_sources}
  PUBLIC_LINKS
    Boost::filesystem
)
