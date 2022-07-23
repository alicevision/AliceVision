# Headers
set(vfs_files_headers
    boost_common.hpp
    directory_entry.hpp
    directory_iterator.hpp
    filesystem.hpp
    fwd.hpp
    istream.hpp
    ostream.hpp
    path.hpp
)

# Sources
set(vfs_files_sources
    directory_entry.cpp
    directory_iterator.cpp
    filesystem.cpp
    istream.cpp
    ostream.cpp
    path.cpp
)

alicevision_add_library(aliceVision_vfs
  SOURCES ${vfs_files_headers} ${vfs_files_sources}
  PUBLIC_LINKS
    Boost::filesystem
)
